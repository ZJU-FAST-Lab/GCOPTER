/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

/**
 * @file global_planning.cpp
 * @brief GCOPTER 全局规划 ROS 节点
 *
 * 该文件实现固定翼无人机全局轨迹规划的 ROS 节点。
 * 主要功能：
 *   1. 订阅点云地图和 3D 导航目标 (PoseStamped)
 *   2. 管理起止航点 (支持两个 3D 目标点分别作为起点和终点)
 *   3. 调用 GCOPTER 优化器进行时空联合轨迹优化
 *   4. 实时发布速度、过载、滚转角、角速度等遥测数据
 *
 * 工作流 (详见 plan()):
 *   RViz 3D Click (PoseStamped) → targetCallBack
 *       │
 *       2 个目标点已收到? ──否──→ 等待
 *       │是
 *   ┌───┴─── enableObstacle? ───┐
 *   │ 是: RRT* + SFC 走廊        │ 否: 仅地面约束 (makeGroundHrep)
 *   └───┬────────────────────────┘
 *       │
 *   GCOPTER_PolytopeSFC.setup()
 *       │
 *   GCOPTER_PolytopeSFC.optimize()
 *       │
 *   visualizer.visualize(traj)   ← RViz 显示轨迹
 *       │
 *   process() (1000 Hz loop)
 *       ├── flatmap.forward() → n_t, n_n, μ, ω
 *       └── 发布 /speed, /nt, /nn, /mu, /bdr 话题
 *
 * @note 固定翼适配: 起点/终点的速度方向由 3D 目标的 yaw 角确定，
 *       而非指向下一个航点。这保证了起止航向与用户期望一致。
 */

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

// ================================================================
//  Config: 全局规划器的 ROS 参数配置结构体
//
//  通过私有命名空间参数 (ros::NodeHandle("~")) 从 launch 文件读取。
//  所有物理单位均采用 SI 制 (m, s, rad, kg, g)。
//
//  参数分类:
//  ① 地图相关:   mapTopic, dilateRadius, voxelWidth, mapBound
//  ② 障碍/规划:  enableObstacle, timeoutRRT
//  ③ 固定翼约束: maxVelMag, minVelMag, maxBdrMag, nnMax, nnMin, muMax
//  ④ 气动参数:   CLmax, wingArea, rhoAir, vehicleMass, gravAcc
//  ⑤ 阻力模型:   horizDrag, vertDrag, parasDrag
//  ⑥ 优化参数:   weightT, chiVec, smoothingEps, integralIntervs, relCostTol
// ================================================================
struct Config
{
    // ---- 地图/通信 ----
    std::string mapTopic;      // 点云地图话题名
    std::string targetTopic;   // 目标点话题名
    double dilateRadius;       // 障碍物膨胀半径 (m)
    double voxelWidth;         // 体素地图分辨率 (m)
    std::vector<double> mapBound; // 地图边界 [xMin, xMax, yMin, yMax, zMin, zMax]

    // ---- 规划模式 ----
    bool enableObstacle;       // true=RRT+SFC 走廊; false=仅地面+边界框

    // ---- RRT* ----
    double timeoutRRT;         // RRT* 超时时间 (s)

    // ---- 固定翼动态约束 ----
    double maxVelMag;          // 最大空速 (m/s)
    double minVelMag;          // 最小空速 (m/s, 防失速)
    double maxBdrMag;          // 最大体轴滚转角速率 |ω_x| (rad/s)
    double nnMax;              // V-N 图法向过载结构上限 (g, 正值)
    double muMax;              // 最大滚转角 |μ| (rad)
    double CLmax;              // 最大升力系数 (无量纲)
    double wingArea;           // 翼面积 (m²)
    double rhoAir;             // 空气密度 (kg/m³)
    double nnMin;              // V-N 图法向过载结构下限 (g, 负值)
    double softMinK;           // V-N 上界 softMin 过渡锐度
    double ntMargin;           // SEP 切向过载安全裕度 (g)

    // ---- 无人机物理参数 ----
    double vehicleMass;        // 飞行器质量 (kg)
    double gravAcc;            // 重力加速度 (m/s², 通常 9.81)
    double horizDrag;          // 水平阻力系数 F_x = -dh·v_x
    double vertDrag;           // 垂直阻力系数 F_z = -dv·v_z
    double parasDrag;          // 寄生阻力系数 (零升阻力)
    double speedEps;           // 空速数值安全因子 (避免除零)

    // ---- 优化器参数 ----
    double weightT;            // 时间最小化权重 ρ (见 gcopter.hpp costFunctional)
    std::vector<double> chiVec; // 8 元素惩罚权重: [pos, vel, omg, nt, nn, mu, muBnd, omgBnd]
    double smoothingEps;       // 平滑 L1 惩罚的过渡区宽度
    int integralIntervs;       // 每段轨迹配点采样数 (积分分辨率)
    double relCostTol;         // L-BFGS 相对代价收敛容限

    /**
     * @brief 从私有 ROS 参数服务器加载配置
     * @param nh_priv "~" 命名空间的 NodeHandle
     */
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

/**
 * @brief 全局规划器主类
 *
 * 管理完整的全局轨迹规划流水线:
 *   ① 接收 RViz 3D 目标点 (PoseStamped) → 收集起止航点
 *   ② 构建安全飞行走廊 (RRT*+SFC 或无障碍地面约束)
 *   ③ 调用 GCOPTER_PolytopeSFC 进行时空联合优化
 *   ④ 实时发布固定翼遥测数据 (速度/过载/滚转角/角速度)
 *
 * @note 起止航点通过收集两个连续的 3D 目标确定:
 *       - 第 1 个目标 → startGoal[0] (起点)
 *       - 第 2 个目标 → startGoal[1] (终点)
 *       收到第 2 个目标后自动触发 plan()。
 */
class GlobalPlanner
{
private:
    /**
     * @brief 目标航点状态
     * 从 geometry_msgs::PoseStamped 提取的位置 + 偏航角
     */
    struct GoalState
    {
        Eigen::Vector3d pos; // 目标位置 (x, y, z) (m)
        double yaw;          // 偏航角 (rad), 由四元数 → 欧拉角转换
    };

    Config config;                   // ROS 参数配置

    ros::NodeHandle nh;              // ROS 节点句柄
    ros::Subscriber mapSub;          // 点云地图订阅器
    ros::Subscriber targetSub;       // 目标点 (PoseStamped) 订阅器

    bool mapInitialized;             // 体素地图是否已初始化
    voxel_map::VoxelMap voxelMap;    // 体素地图 (障碍物查询 + 膨胀)
    Visualizer visualizer;           // RViz 可视化 (轨迹/走廊/球体)
    std::vector<GoalState> startGoal; // 起止航点队列 (最多 2 个)

    Trajectory<5> traj;              // 最新优化的 5 阶多项式轨迹
    double trajStamp;                // 轨迹优化完成的时间戳 (s)

public:
    /**
     * @brief 构造函数
     *
     * 初始化流程:
     *   ① mapInitialized = !enableObstacle (无障碍模式下直接就绪)
     *   ② 根据地图边界和分辨率创建 VoxelMap
     *   ③ 订阅点云地图 (仅障碍模式) 和目标点话题
     *
     * @param conf 已加载的配置参数
     * @param nh_  全局 ROS 节点句柄
     */
    GlobalPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          visualizer(nh)
    {
        // 无障碍模式: 无需点云地图, 直接标记为已初始化
        mapInitialized = !config.enableObstacle;
        ROS_WARN("GlobalPlanner Initialized!");
        ROS_WARN("TargetTopic: [%s], EnableObstacle: %d (int), mapInitialized: %d",
                 config.targetTopic.c_str(),
                 (int)config.enableObstacle,
                 (int)mapInitialized);

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

    /**
     * @brief 点云地图回调函数 (仅障碍模式启用)
     *
     * 将 sensor_msgs::PointCloud2 中的点填充到体素地图中，
     * 然后执行膨胀操作 (补偿飞行器尺寸)。
     *
     * @note 仅执行一次 (首次收到地图时)，之后 mapInitialized = true
     *       跳过 NaN / Inf 无效点
     *
     * @param msg 输入点云消息
     */
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

                // ---- 过滤无效点 ----
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

            // ---- 障碍物膨胀 (保证轨迹与障碍有安全间隙) ----
            voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));

            mapInitialized = true;
        }
    }

    /**
     * @brief 生成「仅地面+天花板」的 H-rep 凸多面体 (无障碍模式)
     *
     * 当 enableObstacle = false 时使用此函数构造一个简单的
     * 轴对齐边界盒作为安全走廊。共 6 个半空间面:
     *   ① 地面:  z ≥ zMin
     *   ② 天花板: z ≤ zMax
     *   ③~⑥ 四周围墙: 包裹起止点并外扩 5m 裕度
     *
     * 半空间格式: a·x + b·y + c·z + d ≤ 0
     *
     * @param start 起点位置
     * @param goal  终点位置
     * @param zMin  地面高度下限
     * @param zMax  天花板高度上限
     * @return 6×4 矩阵, 每行为一个半空间 [a, b, c, d]
     */
    static inline Eigen::MatrixX4d makeGroundHrep(
        const Eigen::Vector3d &start,
        const Eigen::Vector3d &goal,
        double zMin, double zMax)
    {
        Eigen::MatrixX4d hPoly(6, 4);
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

    /**
     * @brief 执行完整的轨迹规划流水线
     *
     * 前提条件: startGoal 已包含 2 个目标点 (起点 + 终点)
     *
     * 规划流程:
     *   ┌─────────────────────────────────────┐
     *   │ startGoal.size() == 2?              │
     *   │ 是 → 继续; 否 → 返回                │
     *   └──────────┬──────────────────────────┘
     *              │
     *   ┌──────────▼──────────┐
     *   │ enableObstacle?     │
     *   │ 是: RRT* 路径搜索    │
     *   │     convexCover SFC │
     *   │     走廊生成         │
     *   │ 否: makeGroundHrep   │
     *   │     地面约束走廊     │
     *   └──────────┬──────────┘
     *              │
     *   ┌──────────▼──────────┐
     *   │ 构建起止 PVA 边界    │
     *   │ vel = [cos(yaw)...  │
     *   │ acc = [0,0,0]       │
     *   └──────────┬──────────┘
     *              │
     *   ┌──────────▼──────────┐
     *   │ gcopter.setup()     │
     *   │ ① H-rep→V-rep       │
     *   │ ② 段分配            │
     *   │ ③ 初始化 MINCO      │
     *   └──────────┬──────────┘
     *              │
     *   ┌──────────▼──────────┐
     *   │ gcopter.optimize()  │
     *   │ L-BFGS 时空优化     │
     *   └──────────┬──────────┘
     *              │
     *   ┌──────────▼──────────┐
     *   │ visualizer 发布轨迹  │
     *   │ 保存 trajStamp      │
     *   └─────────────────────┘
     *
     * @note 起点/终点的速度方向由 RViz 3D 目标的 yaw 角确定;
     *       加速度始终为零 (固定翼巡航假设)。
     */
    inline void plan()
    {
        ROS_WARN("plan() called. startGoal.size() = %zu", startGoal.size());
        if (startGoal.size() == 2)
        {
            std::vector<Eigen::Vector3d> route;  // RRT* 路径航点
            std::vector<Eigen::MatrixX4d> hPolys; // H-rep 安全走廊

            if (config.enableObstacle)
            {
                // ============================================================
                //  有障碍模式: RRT* 路径搜索 + 凸覆盖 SFC 走廊生成
                // ============================================================
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
                                     7.0,   // 最大走廊宽度
                                     3.0,   // 最小走廊宽度
                                     hPolys);
                sfc_gen::shortCut(hPolys);
            }
            else
            {
                // ============================================================
                //  无障碍模式: 仅地面约束 + 轴对齐边界框
                // ============================================================
                route = {startGoal[0].pos, startGoal[1].pos};
                hPolys = {makeGroundHrep(startGoal[0].pos, startGoal[1].pos, 0.0, 100.0)};
            }

            if (route.size() > 1)
            {
                // ---- 可视化安全走廊 ----
                visualizer.visualizePolytope(hPolys);

                // ---- 构建 PVA 边界条件 ----
                // 速度: 由 yaw 角确定水平方向, 幅度取 minVelMag
                const double initSpeed = std::max(config.minVelMag, 0.1);
                const Eigen::Vector3d initVel(std::cos(startGoal[0].yaw) * initSpeed,
                                              std::sin(startGoal[0].yaw) * initSpeed,
                                              0.0);
                const Eigen::Vector3d finalVel(std::cos(startGoal[1].yaw) * initSpeed,
                                               std::sin(startGoal[1].yaw) * initSpeed,
                                               0.0);

                Eigen::Matrix3d iniState;  // 3×3: [pos, vel, acc]
                Eigen::Matrix3d finState;
                iniState << route.front(), initVel, Eigen::Vector3d::Zero();
                finState << route.back(), finalVel, Eigen::Vector3d::Zero();

                gcopter::GCOPTER_PolytopeSFC gcopter;

                // ============================================================
                //  将 Config 参数打包为 GCOPTER 优化器所需的向量格式
                //
                //  magnitudeBounds = [vMax, omgXMax, nnMax, nnMin, muMax, ntMargin, vMin]^T
                //  penaltyWeights  = [pos_w, vel_w, omg_w, nt_w, nn_w, mu_w]^T
                //  physicalParams  = [mass, grav, dh, dv, cp, veps, kCL, softMinK]^T
                // ============================================================
                Eigen::VectorXd magnitudeBounds(7);
                Eigen::VectorXd penaltyWeights(8);
                Eigen::VectorXd physicalParams(8);
                magnitudeBounds(0) = config.maxVelMag;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.nnMax;
                magnitudeBounds(3) = config.nnMin;
                magnitudeBounds(4) = config.muMax;
                magnitudeBounds(5) = config.ntMargin;
                magnitudeBounds(6) = config.minVelMag;
                penaltyWeights(0) = (config.chiVec)[0];  // 位置权重
                penaltyWeights(1) = (config.chiVec)[1];  // 速度权重
                penaltyWeights(2) = (config.chiVec)[2];  // 角速度权重
                penaltyWeights(3) = (config.chiVec)[3];  // 切向过载权重
                penaltyWeights(4) = (config.chiVec)[4];  // 法向过载权重
                penaltyWeights(5) = (config.chiVec)[5];  // 滚转角权重
                penaltyWeights(6) = (config.chiVec)[6];  // 边界 μ→0 权重
                penaltyWeights(7) = (config.chiVec)[7];  // 边界 ω→0 权重
                physicalParams(0) = config.vehicleMass;
                physicalParams(1) = config.gravAcc;
                physicalParams(2) = config.horizDrag;
                physicalParams(3) = config.vertDrag;
                physicalParams(4) = config.parasDrag;
                physicalParams(5) = config.speedEps;
                // kCL = ρ·S·CLmax / (2·m·g)  → 升力曲线斜率
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

                // ---- 设置优化器 ----
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

                // ---- 执行 L-BFGS 时空联合优化 ----
                ROS_WARN("gcopter.optimize() starting...");
                if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
                {
                    ROS_WARN("gcopter.optimize() failed (returned INF)!");
                    return;
                }

                // ---- 可视化优化结果 ----
                ROS_WARN("gcopter.optimize() success! PieceNum: %d", traj.getPieceNum());
                if (traj.getPieceNum() > 0)
                {
                    trajStamp = ros::Time::now().toSec();
                    visualizer.visualize(traj, route);
                }
            }
        }
    }

    /**
     * @brief RViz 3D 导航目标回调函数
     *
     * 接收 geometry_msgs::PoseStamped 消息 (由 RViz Goal3DTool 发出),
     * 提取位置和偏航角, 并管理起止航点队列。
     *
     * 航点管理规则:
     *   - startGoal 为空 → 存入第 1 个目标 (起点)
     *   - startGoal 有 1 个 → 存入第 2 个目标 (终点), 触发 plan()
     *   - startGoal 已有 2 个 → 清空后重新开始
     *
     * 四元数 → 偏航角转换 (ZYX 欧拉角约定):
     *   ψ = atan2(2(qw·qz + qx·qy), 1 - 2(qy² + qz²))
     *
     * @note 无障碍模式下 targetCallBack 始终接收目标;
     *       有障碍模式下需等待 mapInitialized 后才接收。
     *       Z 坐标自动限制在 [mapBound[4] + dilateRadius,
     *       mapBound[5] - dilateRadius] 范围内。
     *
     * @param msg PoseStamped 消息 (含位置 + 四元数姿态)
     */
    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mapInitialized || !config.enableObstacle)
        {
            // ---- 已满 2 个目标则清空重来 ----
            if (startGoal.size() >= 2)
            {
                startGoal.clear();
            }

            // ---- 四元数 → 偏航角 ----
            const double qx = msg->pose.orientation.x;
            const double qy = msg->pose.orientation.y;
            const double qz = msg->pose.orientation.z;
            const double qw = msg->pose.orientation.w;
            const double siny_cosp = 2.0 * (qw * qz + qx * qy);
            const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            const double yaw = std::atan2(siny_cosp, cosy_cosp);

            // ---- 限制 Z 在地图范围内 ----
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

            // ---- 碰撞检测 (仅障碍模式) ----
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

    /**
     * @brief 运行时遥测发布与轨迹跟踪
     *
     * 在 1000 Hz 主循环中周期性调用, 实现:
     *   ① 微分平坦映射: (v, a, j) → (q, ω, n_t, n_n, μ)
     *   ② 发布固定翼遥测话题:
     *      - /speed:   空速标量 (m/s)
     *      - /nt:      切向过载 (g)
     *      - /nn:      法向过载 (g)
     *      - /mu:      滚转角 (rad)
     *      - /bdr:     体轴角速度模长 (rad/s)
     *   ③ 可视化轨迹跟踪球: 显示飞机当前位置
     *
     * @note 仅当 traj 有效且当前时间在轨迹持续时间内才发布数据。
     *       物理参数使用独立的 FlatnessMap 实例 (与优化器解耦)。
     */
    inline void process()
    {
        // ---- 初始化微分平坦映射 (仅需一次, 但放在循环外需成员变量) ----
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
                Eigen::Vector4d quat;    // [qw, qx, qy, qz]
                Eigen::Vector3d omg;     // 体轴角速度 [ω_x, ω_y, ω_z]
                double n_t, n_n, mu;     // 切向过载, 法向过载, 滚转角

                // ---- 微分平坦正向映射: 轨迹状态 → 控制量 ----
                flatmap.forward(traj.getVel(delta),
                                traj.getAcc(delta),
                                traj.getJer(delta),
                                quat, omg, &n_t, &n_n, &mu);
                double speed = traj.getVel(delta).norm();
                double bodyratemag = omg.norm();

                // ---- 发布遥测数据 ----
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

                // ---- 可视化跟踪球体 ----
                visualizer.visualizeSphere(traj.getPos(delta),
                                           config.dilateRadius);
            }
        }
    }
};

// ================================================================
//  main: 全局规划节点入口
//
//  启动流程:
//    ① ros::init → 节点名 "global_planning_node"
//    ② 创建 GlobalPlanner 实例 (加载参数 + 订阅话题)
//    ③ 1000 Hz 循环: process() + spinOnce()
//
//  节点退出: Ctrl+C  → ros::ok() = false → 正常退出
// ================================================================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh_;

    ROS_WARN("global_planning_node main start");
    // 构造全局规划器 (私有命名空间参数 + 全局句柄)
    GlobalPlanner global_planner(Config(ros::NodeHandle("~")), nh_);

    // ---- 1000 Hz 实时循环 ----
    ros::Rate lr(1000);
    while (ros::ok())
    {
        global_planner.process();
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
