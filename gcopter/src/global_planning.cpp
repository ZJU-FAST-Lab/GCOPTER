#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

struct Config
{
    std::string mapTopic;
    std::string targetTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    bool enableObstacle;   // true=RRT+SFC, false=仅地面约束
    double timeoutRRT;
    double maxVelMag;      // 最大空速 (m/s)
    double minVelMag;      // 最小空速 (m/s)
    double maxBdrMag;      // 最大 ω_x (rad/s)
    double nnMax;          // V-N 图结构过载上限 (g)
    double muMax;          // 最大滚转角 μ (rad)
    double CLmax;          // 最大升力系数
    double wingArea;       // 翼面积 (m²)
    double rhoAir;         // 空气密度 (kg/m³)
    double nnMin;          // V-N 图结构过载下限 (g, 负值)
    double softMinK;       // V-N 软过渡锐度
    double ntMargin;       // SEP 安全裕度 (g)
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;  // 6 元素: [pos, vel, omg, nt, nn, mu]
    double smoothingEps;
    int integralIntervs;
    double relCostTol;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("VoxelWidth", voxelWidth);
        nh_priv.getParam("MapBound", mapBound);
        nh_priv.param("EnableObstacle", enableObstacle, true);
        nh_priv.getParam("TimeoutRRT", timeoutRRT);
        nh_priv.getParam("MinVelMag", minVelMag);
        nh_priv.getParam("MaxVelMag", maxVelMag);
        nh_priv.getParam("MaxBdrMag", maxBdrMag);
        nh_priv.getParam("NnMax", nnMax);
        nh_priv.getParam("MuMax", muMax);
        nh_priv.param("CLmax", CLmax, 1.2);
        nh_priv.param("WingArea", wingArea, 0.45);
        nh_priv.param("RhoAir", rhoAir, 1.225);
        nh_priv.getParam("NnMin", nnMin);
        nh_priv.param("SoftMinK", softMinK, 20.0);
        nh_priv.param("NtMargin", ntMargin, 0.05);
        nh_priv.getParam("VehicleMass", vehicleMass);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("HorizDrag", horizDrag);
        nh_priv.getParam("VertDrag", vertDrag);
        nh_priv.getParam("ParasDrag", parasDrag);
        nh_priv.getParam("SpeedEps", speedEps);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("IntegralIntervs", integralIntervs);
        nh_priv.getParam("RelCostTol", relCostTol);
    }
};

class GlobalPlanner
{
private:
    struct GoalState
    {
        Eigen::Vector3d pos;
        double yaw;
    };

    Config config;

    ros::NodeHandle nh;
    ros::Subscriber mapSub;
    ros::Subscriber targetSub;

    bool mapInitialized;
    voxel_map::VoxelMap voxelMap;
    Visualizer visualizer;
    std::vector<GoalState> startGoal;

    Trajectory<5> traj;
    double trajStamp;

public:
        GlobalPlanner(const Config &conf,
                                    ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(!config.enableObstacle),  // 无障碍模式跳过地图等待
          visualizer(nh)
    {
                ROS_WARN("GlobalPlanner Initialized!");
                ROS_WARN("TargetTopic: [%s], EnableObstacle: %s, mapInitialized: %s",
                                 config.targetTopic.c_str(),
                                 config.enableObstacle ? "true" : "false",
                                 mapInitialized ? "true" : "false");

        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        if (config.enableObstacle)
        {
            mapSub = nh.subscribe(config.mapTopic, 1, &GlobalPlanner::mapCallBack, this,
                                  ros::TransportHints().tcpNoDelay());
        }

        targetSub = nh.subscribe(config.targetTopic, 1, &GlobalPlanner::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
    }

    inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (!mapInitialized)
        {
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *)(&msg->data[0]);
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }
                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));
            }

            voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));

            mapInitialized = true;
        }
    }

    // ================================================================
    //  辅助: 生成「仅地面+天花板」的 H-rep 凸多面体 (无障碍模式)
    //  6 个面: 地面(z≥zMin), 天花板(z≤zMax), 4个垂直面包围起止点
    // ================================================================
    static inline Eigen::MatrixX4d makeGroundHrep(
        const Eigen::Vector3d &start,
        const Eigen::Vector3d &goal,
        double zMin, double zMax)
    {
        Eigen::MatrixX4d hPoly(6, 4);
        // 面格式: a·x + b·y + c·z + d ≤ 0
        // ground:   z ≥ zMin  → -z + zMin ≤ 0
        hPoly.row(0) << 0, 0, -1, zMin;
        // ceiling:  z ≤ zMax  →  z - zMax ≤ 0
        hPoly.row(1) << 0, 0, 1, -zMax;
        // 4 垂直面: 包围起止点 + 宽裕度
        double margin = 5.0;
        double xMin = std::min(start.x(), goal.x()) - margin;
        double xMax = std::max(start.x(), goal.x()) + margin;
        double yMin = std::min(start.y(), goal.y()) - margin;
        double yMax = std::max(start.y(), goal.y()) + margin;
        hPoly.row(2) << -1, 0, 0, xMin;   // -x + xMin ≤ 0 → x ≥ xMin
        hPoly.row(3) << 1, 0, 0, -xMax;   //  x - xMax ≤ 0 → x ≤ xMax
        hPoly.row(4) << 0, -1, 0, yMin;   // -y + yMin ≤ 0 → y ≥ yMin
        hPoly.row(5) << 0, 1, 0, -yMax;   //  y - yMax ≤ 0 → y ≤ yMax
        return hPoly;
    }

    inline void plan()
    {
        ROS_WARN("plan() called. startGoal.size() = %zu", startGoal.size());
        if (startGoal.size() == 2)
        {
            std::vector<Eigen::Vector3d> route;
            std::vector<Eigen::MatrixX4d> hPolys;

            if (config.enableObstacle)
            {
                // ---- 有障碍模式: RRT* + SFC 走廊 ----
                sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0].pos,
                                                   startGoal[1].pos,
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap, 0.01,
                                                   route);
                std::vector<Eigen::Vector3d> pc;
                voxelMap.getSurf(pc);

                sfc_gen::convexCover(route,
                                     pc,
                                     voxelMap.getOrigin(),
                                     voxelMap.getCorner(),
                                     7.0,
                                     3.0,
                                     hPolys);
                sfc_gen::shortCut(hPolys);
            }
            else
            {
                // ---- 无障碍模式: 仅地面约束 + 边界框 ----
                route = {startGoal[0].pos, startGoal[1].pos};
                hPolys = {makeGroundHrep(startGoal[0].pos, startGoal[1].pos, 0.0, 100.0)};
            }

            if (route.size() > 1)
            {
                visualizer.visualizePolytope(hPolys);

                const double initSpeed = std::max(config.minVelMag, 0.1);
                const Eigen::Vector3d initVel(std::cos(startGoal[0].yaw) * initSpeed,
                                              std::sin(startGoal[0].yaw) * initSpeed,
                                              0.0);
                const Eigen::Vector3d finalVel(std::cos(startGoal[1].yaw) * initSpeed,
                                               std::sin(startGoal[1].yaw) * initSpeed,
                                               0.0);

                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                iniState << route.front(), initVel, Eigen::Vector3d::Zero();
                finState << route.back(), finalVel, Eigen::Vector3d::Zero();

                gcopter::GCOPTER_PolytopeSFC gcopter;

                // magnitudeBounds = [vMax, omgXMax, nnMax, nnMin, muMax, ntMargin, vMin]^T
                // penaltyWeights = [pos_weight, vel_weight, omg_weight, nt_weight, nn_weight, mu_weight]^T
                // physicalParams  = [mass, grav, dh, dv, cp, veps, kCL, softMinK]^T
                Eigen::VectorXd magnitudeBounds(7);
                Eigen::VectorXd penaltyWeights(6);
                Eigen::VectorXd physicalParams(8);
                magnitudeBounds(0) = config.maxVelMag;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.nnMax;         // nnMax
                magnitudeBounds(3) = config.nnMin;         // nnMin
                magnitudeBounds(4) = config.muMax;
                magnitudeBounds(5) = config.ntMargin;
                magnitudeBounds(6) = config.minVelMag;     // vMin
                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                penaltyWeights(3) = (config.chiVec)[3];
                penaltyWeights(4) = (config.chiVec)[4];
                penaltyWeights(5) = (config.chiVec)[5];
                physicalParams(0) = config.vehicleMass;
                physicalParams(1) = config.gravAcc;
                physicalParams(2) = config.horizDrag;
                physicalParams(3) = config.vertDrag;
                physicalParams(4) = config.parasDrag;
                physicalParams(5) = config.speedEps;
                // kCL = ρ·S·CLmax / (2·m·g)
                physicalParams(6) = config.rhoAir * config.wingArea * config.CLmax
                                   / (2.0 * config.vehicleMass * config.gravAcc);
                physicalParams(7) = config.softMinK;
                const int quadratureRes = config.integralIntervs;

                traj.clear();
                
                ROS_WARN("gcopter.setup() starting...");
                ROS_WARN("hPolys size: %zu", hPolys.size());
                if (hPolys.size() > 0) {
                    std::cout << hPolys[0] << std::endl;
                }
                
                if (!gcopter.setup(config.weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams))
                {
                    ROS_WARN("gcopter.setup() returned false! Check states, hPolys, bounds.");
                    return;
                }

                ROS_WARN("gcopter.optimize() starting...");
                if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
                {
                    ROS_WARN("gcopter.optimize() failed (returned INF)!");
                    return;
                }

                ROS_WARN("gcopter.optimize() success! PieceNum: %d", traj.getPieceNum());
                if (traj.getPieceNum() > 0)
                {
                    trajStamp = ros::Time::now().toSec();
                    visualizer.visualize(traj, route);
                }
            }
        }
    }

    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mapInitialized)
        {
            if (startGoal.size() >= 2)
            {
                startGoal.clear();
            }
            const double qx = msg->pose.orientation.x;
            const double qy = msg->pose.orientation.y;
            const double qz = msg->pose.orientation.z;
            const double qw = msg->pose.orientation.w;
            const double siny_cosp = 2.0 * (qw * qz + qx * qy);
            const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            const double yaw = std::atan2(siny_cosp, cosy_cosp);

            double zGoal = msg->pose.position.z;
            const double zMin = config.mapBound[4] + config.dilateRadius;
            const double zMax = config.mapBound[5] - config.dilateRadius;
            if (zGoal < zMin)
            {
                zGoal = zMin;
            }
            if (zGoal > zMax)
            {
                zGoal = zMax;
            }

            GoalState goalState;
            goalState.pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, zGoal);
            goalState.yaw = yaw;

            if (!config.enableObstacle || voxelMap.query(goalState.pos) == 0)
            {
                ROS_WARN("Goal Selected: (%f, %f, %f), yaw: %f. Current startGoal size (before adding): %zu",
                         goalState.pos.x(), goalState.pos.y(), goalState.pos.z(), goalState.yaw, startGoal.size());
                visualizer.visualizeStartGoal(goalState.pos, 0.5, startGoal.size());
                startGoal.emplace_back(goalState);
            }
            else
            {
                ROS_WARN("Infeasible Position Selected !!!\n");
            }

            plan();
        }
        return;
    }

    inline void process()
    {
        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;

        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));

        if (traj.getPieceNum() > 0)
        {
            const double delta = ros::Time::now().toSec() - trajStamp;
            if (delta > 0.0 && delta < traj.getTotalDuration())
            {
                Eigen::Vector4d quat;
                Eigen::Vector3d omg;
                double n_t, n_n, mu;

                flatmap.forward(traj.getVel(delta),
                                traj.getAcc(delta),
                                traj.getJer(delta),
                                quat, omg, &n_t, &n_n, &mu);
                double speed = traj.getVel(delta).norm();
                double bodyratemag = omg.norm();
                std_msgs::Float64 speedMsg, ntMsg, nnMsg, muMsg, bdrMsg;
                speedMsg.data = speed;
                ntMsg.data = n_t;
                nnMsg.data = n_n;
                muMsg.data = mu;
                bdrMsg.data = bodyratemag;
                visualizer.speedPub.publish(speedMsg);
                visualizer.ntPub.publish(ntMsg);
                visualizer.nnPub.publish(nnMsg);
                visualizer.muPub.publish(muMsg);
                visualizer.bdrPub.publish(bdrMsg);

                visualizer.visualizeSphere(traj.getPos(delta),
                                           config.dilateRadius);
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh_;

    ROS_WARN("global_planning_node main start");
    GlobalPlanner global_planner(Config(ros::NodeHandle("~")), nh_);

    ros::Rate lr(1000);
    while (ros::ok())
    {
        global_planner.process();
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
