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

#ifndef FLATNESS_HPP
#define FLATNESS_HPP

#include <Eigen/Eigen>

#include <cmath>

namespace flatness
{
    /**
     * @brief 差分平坦度映射类 (Differential Flatness Map)
     * 
     * 该类实现了考虑复杂空气阻力（水平、垂直及非线性寄生阻力）的四旋翼飞行器差分平坦度映射。
     * 主要提供正向映射（将轨迹上的平坦输出及其导数转换为控制系统的期望推力、姿态和角速度）
     * 和反向映射（用于轨迹优化过程中的基于链式法则的梯度反向计算传递）。
     * 相关推导详见: https://github.com/ZJU-FAST-Lab/GCOPTER/blob/main/misc/flatness.pdf
     */
    class FlatnessMap
    {
    public:
        /**
         * @brief 初始化四旋翼飞行器的物理及气动参数
         *
         * @param vehicle_mass 飞行器质量
         * @param gravitational_acceleration 重力加速度
         * @param horitonral_drag_coeff 水平气动阻力系数
         * @param vertical_drag_coeff 垂直气动阻力系数
         * @param parasitic_drag_coeff 寄生阻力系数（与速度的平方成正比）
         * @param speed_smooth_factor 速度平滑因子，用于避免奇异性或微小速度下的分母为零等情况
         */
        inline void reset(const double &vehicle_mass,
                          const double &gravitational_acceleration,
                          const double &horitonral_drag_coeff,
                          const double &vertical_drag_coeff,
                          const double &parasitic_drag_coeff,
                          const double &speed_smooth_factor)
        {
            mass = vehicle_mass;
            grav = gravitational_acceleration;
            dh = horitonral_drag_coeff;
            dv = vertical_drag_coeff;
            cp = parasitic_drag_coeff;
            veps = speed_smooth_factor;

            return;
        }

        /**
         * @brief 正向平坦度映射 (Forward Flatness Map)
         *
         * 已知轨迹当前状态下（基于给定的速度、加速度、加加速度和偏航等），根据包含了空气阻力的动力学模型，
         * 解析计算所需的期望推力、期望姿态（四元数表示）和期望体轴角速度。
         *
         * @param vel 速度向量 (Velocity)
         * @param acc 加速度向量 (Acceleration)
         * @param jer 加加速度向量 (Jerk, 加速度的导数)
         * @param psi 偏航角 (Yaw)
         * @param dpsi 偏航角速率 (Yaw rate)
         * @param thr [输出] 系统所需的期望总推力 (Thrust)
         * @param quat [输出] 系统所需的期望姿态四元数 (Quaternion [w, x, y, z])
         * @param omg [输出] 系统所需的期望体轴角速度 (Angular velocity)
         * @param n_t_out [可选输出] 切向过载 n_t
         * @param n_n_out [可选输出] 法向过载 n_n
         * @param mu_out [可选输出] 速度滚转角 mu
         */
        inline void forward(const Eigen::Vector3d &vel, // 大地系下的速度
                            const Eigen::Vector3d &acc, // 大地系下的加速度
                            const Eigen::Vector3d &jer, // 大地系下的加加速度
                            Eigen::Vector4d &quat, // 输出：期望姿态四元数
                            Eigen::Vector3d &omg, // 输出：期望气流系角速度
                            double *n_t_out = nullptr, // 可选输出：切向过载
                            double *n_n_out = nullptr, // 可选输出：法向过载
                            double *mu_out = nullptr) // 可选输出：速度滚转角
        {
            // ======= 1. 基向量计算 (气流系 Wind Frame：NED 标准，无风假设) =======
            // 通过大地系位置向量 [x y z]^T 推导出气流系的基向量在大地系中的投影（i_a, j_a, k_a）
            // 其中 i_a 沿着飞机的空速方向，j_a 垂直于速度与加速度构成的平面，k_a 则由右手定则补全

            // 默认无风，空速等于地速
            double v_norm = vel.norm();
            double v_norm_inv = 1.0 / (v_norm + 1e-6); // 防止奇异

            // 气流系 X 轴：沿空速（地速）方向
            Eigen::Vector3d i_a = vel * v_norm_inv;

            // 过载加速度：飞机的绝对加速度与重力加速度的差值
            // NED 坐标系下重力方向为 [0, 0, g]^T
            Eigen::Vector3d a_g = acc;
            a_g(2) -= grav; // 过载加速度 = 绝对加速度 - 重力加速度

            // 气流系 Y 轴：根据固定翼“协调转弯”无侧滑假设，y轴垂直于速度与气动加速度构成的对称平面
            // 利用右手定则 v x a_g 必定得到指向飞机右侧的 Y 轴矢量
            Eigen::Vector3d v_times_ag = vel.cross(a_g);
            double v_times_ag_norm = v_times_ag.norm();
            double v_times_ag_inv = 1.0 / (v_times_ag_norm + 1e-6);
            Eigen::Vector3d j_a = v_times_ag * v_times_ag_inv;

            // 气流系 Z 轴：在对称平面内指向下方，由右手法则 X x Y = Z 补全
            Eigen::Vector3d k_a = i_a.cross(j_a);

            // ======= 2. 姿态四元数计算 =======
            // R 为气流系到大地系的方向余弦矩阵，列向量分别为 i_a, j_a, k_a 在大地系中的投影
            // R 属于 SO(3)，可以通过 Eigen 的函数直接转换为四元数表示的姿态
            Eigen::Matrix3d R;
            R.col(0) = i_a;
            R.col(1) = j_a;
            R.col(2) = k_a;
            Eigen::Quaterniond q_obj(R);
            quat(0) = q_obj.w();
            quat(1) = q_obj.x();
            quat(2) = q_obj.y();
            quat(3) = q_obj.z();

            // ======= 3. 角速度计算 =======
            // 气流轴系 X 轴单位向量 (i_a) 的时间导数在大地系下的投影 (i_a_dot)
            double v_dot_a = vel.dot(acc);
            Eigen::Vector3d i_a_dot = (acc * v_norm - vel * v_dot_a * v_norm_inv) * (v_norm_inv * v_norm_inv);

            // 气流轴系 Y 轴 (j_a) 的时间导数在大地系下的投影 (a_g 的导数即为加加速度 jer)
            // d/dt (v x a_g) = a x a_g + v x jerk
            Eigen::Vector3d v_times_ag_dot = acc.cross(a_g) + vel.cross(jer);
            // c · c_dot, used by d/dt (c / ||c||) for the normalization term
            double cross_dot_inner = v_times_ag.dot(v_times_ag_dot);
            // d/dt (c / ||c||) = (c_dot * ||c|| - c * (c · c_dot) / ||c||) / ||c||^2
            Eigen::Vector3d j_a_dot = (v_times_ag_dot * v_times_ag_norm - v_times_ag * cross_dot_inner * v_times_ag_inv) * (v_times_ag_inv * v_times_ag_inv);

            // 气流轴系 Z 轴 (k_a) 的时间导数在大地系下的投影不需要，故不计算

            // 泊松方程反解气流系角速度在气流系下的投影 ^A\omega
            omg(0) = k_a.dot(j_a_dot);  // 滚转角速率 \omega_x
            omg(1) = -k_a.dot(i_a_dot); // 俯仰角速率 \omega_y
            omg(2) = j_a.dot(i_a_dot);  // 偏航角速率 \omega_z

            // ======= 4. 控制量计算 =======
            // 控制量为[n_t n_n mu]，n_t为切向过载，n_n为法向过载，mu为速度滚转角
            // n_t: a_g/g 在气流轴系 x 轴上的投影; n_n: a_g/g 在气流轴系 z 轴负方向的投影
            double n_t = a_g.dot(i_a) / grav;
            // n_n: a_g/g 在气流轴系 z 轴负方向的投影，因此前面有负号
            double n_n = -a_g.dot(k_a) / grav;
            // mu: 速度滚转角
            // 航迹系与气流系共用x轴(速度方向)，因此航迹系到气流系只需绕x轴转过速度滚转角 \mu。
            // 由于航迹系y轴始终在水平面内，气流系的 j_a 和 k_a 向量在大地系 Z 轴(重力铅锤方向)的投影
            // 即 j_a(2) 和 k_a(2) 天然正比于 \sin\mu 和 \cos\mu (比例系数为 \cos\gamma)。
            // 此方法优雅地将物理定义映射到了几何投影，也交由 atan2 安全处理了垂直俯冲/爬升 (\cos\gamma = 0) 的死锁点
            double mu = std::atan2(j_a(2), k_a(2));

            // ======= 5. 组装并返回平坦度输出转换结果 =======
            // 将从平坦输出(平动轨迹)推导出的飞机自身动力学控制量赋值给传入的可选指针参数(若指针不为空)
            // 分别对应: 切向过载 (Tangential load factor)
            //          法向过载 (Normal load factor)
            //          速度滚转角 (Velocity roll angle)
            if (n_t_out)
            {
                *n_t_out = n_t;
            }
            if (n_n_out)
            {
                *n_n_out = n_n;
            }
            if (mu_out)
            {
                *mu_out = mu;
            }

            return;
        }


        inline void backward(const Eigen::Vector3d &v,
                             const Eigen::Vector3d &a,
                             const Eigen::Vector3d &jer,
                             double dJ_dnt,
                             double dJ_dnn,
                             double dJ_dmu,
                             const Eigen::Vector3d &dJ_domg,
                             Eigen::Vector3d &grad_v,
                             Eigen::Vector3d &grad_a,
                             Eigen::Vector3d &grad_j) const
        {
            double in1[3] = {v.x(), v.y(), v.z()};
            double in2[3] = {a.x(), a.y(), a.z()};
            double in3[3] = {jer.x(), jer.y(), jer.z()};
            
            double dJ_domg0 = dJ_domg.x();
            double dJ_domg1 = dJ_domg.y();
            double dJ_domg2 = dJ_domg.z();
            
            double grad_v_arr[3] = {0.0};
            double grad_a_arr[3] = {0.0};
            double grad_j_arr[3] = {0.0};

            double b_grad_v_tmp, c_grad_v_tmp, d_grad_v_tmp, e_grad_v_tmp, f_grad_v_tmp, g_grad_v_tmp;
            double grad_v_tmp, grad_v_tmp_tmp;
            double t104, t105, t115, t122, t131, t132, t136, t137, t138, t139, t140, t141;
            double t142, t143, t144, t147, t148, t149, t150, t151, t152, t153, t154, t155;
            double t156, t157, t160, t161, t162, t163, t164, t165, t166, t167, t168, t17;
            double t171, t175, t177, t18, t180, t181, t182, t183, t184, t185, t186, t189;
            double t19, t201, t205, t205_tmp, t208, t209, t21, t210_tmp, t221, t223, t224;
            double t229, t238, t26, t269_tmp, t287, t288, t289, t29, t292_tmp, t294, t297;
            double t298, t30, t303, t304, t305, t307, t307_tmp, t31, t310, t310_tmp, t311;
            double t311_tmp, t312, t313, t314, t315, t316, t317, t318, t319, t32, t320;
            double t324, t325, t327, t328, t33, t38, t39, t4, t44, t45, t5, t50, t52, t53;
            double t57, t58, t59, t6, t60, t61, t7, t70, t71, t72, t73, t8, t82, t83, t84;
            double t85, t86, t87, t9, t92, t93, t94;

            double* grad_v_ptr = grad_v_arr;
            double* grad_a_ptr = grad_a_arr;
            double* grad_j_ptr = grad_j_arr;

  t4 = in2[0] * in1[0];
  t5 = in2[0] * in1[1];
  t6 = in1[0] * in2[1];
  t7 = in2[0] * in1[2];
  t8 = in2[1] * in1[1];
  t9 = in2[1] * in1[2];
  t17 = in1[0] * in1[0];
  t18 = in1[1] * in1[1];
  t19 = in1[2] * in1[2];
  t21 = 1.0 / grav;
  t26 = in2[2] - grav;
  t33 = (t4 + t8) + in2[2] * in1[2];
  t38 = (t17 + t18) + t19;
  t29 = t26 * in1[0];
  t30 = t26 * in1[1];
  t31 = t5 - t6;
  t32 = in3[0] * in1[1] - in1[0] * in3[1];
  t52 = 1.0 / t38;
  t57 = std::sqrt(t38);
  t39 = t31 * t31;
  t44 = t31 * in1[0];
  t45 = t31 * in1[1];
  t53 = t52 * t52;
  t60 = 1.0 / t57;
  t50 = t44 * 2.0;
  t58 = t7 - t29;
  t59 = t9 - t30;
  t61 = (t60 * t60 * t60);
  t83 = t5 * t60;
  t84 = t6 * t60;
  t85 = t7 * t60;
  t38 = in2[2] * t60;
  t86 = t38 * in1[0];
  t87 = t9 * t60;
  t9 = t38 * in1[1];
  t92 = t17 * t60;
  t93 = t18 * t60;
  t94 = t19 * t60;
  t104 = ((in2[0] * in2[2] + in3[0] * in1[2]) - in1[0] * in3[2]) - in2[0] * t26;
  t105 = ((in2[1] * in2[2] + in3[1] * in1[2]) - in1[1] * in3[2]) - in2[1] * t26;
  t115 = t33 * t60;
  t70 = t58 * in1[0];
  t71 = t58 * in1[2];
  t72 = t59 * in1[1];
  t73 = t59 * in1[2];
  t5 = t33 * t61;
  t38 = t5 * in1[0];
  t7 = t38 * in1[1];
  t38 *= in1[2];
  t5 = t5 * in1[1] * in1[2];
  t82 = t73 * 2.0;
  t122 = t45 + t71;
  t137 = t70 + t72;
  t138 = in2[1] * t31 * 2.0 + t26 * t58 * 2.0;
  t139 = in2[0] * t58 * 2.0 + in2[1] * t59 * 2.0;
  t141 = t57 - t92;
  t142 = t57 - t93;
  t143 = t57 - t94;
  t147 = (t39 + t58 * t58) + t59 * t59;
  t156 = t104 * in1[0] + t105 * in1[1];
  t164 = (t32 * in1[1] + grav * t58) + t104 * in1[2];
  t165 = (-(t32 * in1[0]) + grav * t59) + t105 * in1[2];
  t180 = (t83 + -t6 * t60) + t7;
  t181 = (t84 - t83) + t7;
  t182 = (t85 - t86) + t38;
  t183 = (t86 - t85) + t38;
  t184 = (t87 - t9) + t5;
  t185 = (t9 - t87) + t5;
  t189 = ((in3[1] * t31 + in2[1] * t32) + in3[2] * t58) + t26 * t104;
  t229 = ((in3[0] * t58 + in3[1] * t59) + in2[0] * t104) + in2[1] * t105;
  t238 = (t31 * t32 + t58 * t104) + t59 * t105;
  t131 = t44 - t73;
  t132 = t45 * 2.0 + t71 * 2.0;
  t140 = t70 * 2.0 + t72 * 2.0;
  t144 = in2[0] * t31 * 2.0 - t26 * t59 * 2.0;
  t148 = in2[0] * t57 - t115 * in1[0];
  t149 = in2[1] * t57 - t115 * in1[1];
  t150 = in2[2] * t57 - t115 * in1[2];
  t151 = 1.0 / t147;
  t153 = std::sqrt(t147);
  t166 = t115 - t17 * t33 * t61;
  t167 = t115 - t18 * t33 * t61;
  t168 = t115 - t19 * t33 * t61;
  t136 = t50 - t82;
  t152 = t151 * t151;
  t154 = 1.0 / t153;
  t157 = grav * t153;
  t160 = in3[2] * t153;
  t161 = t153 * in1[0];
  t162 = t153 * in1[1];
  t163 = t153 * in1[2];
  t155 = (t154 * t154 * t154);
  t171 = t4 * t60 * t154;
  t175 = t8 * t60 * t154;
  t5 = t60 * t154;
  t38 = t5 * in1[0];
  t177 = t38 * in1[1];
  t7 = t38 * in1[2];
  t57 = t5 * in1[1] * in1[2];
  t186 = t92 * t154;
  t93 *= t154;
  t147 = t94 * t154;
  t201 = t26 * t60 * t154 * in1[2];
  t205_tmp = t44 * t60;
  t205 = t205_tmp * t154;
  t38 = t44 * t61 * t154;
  t94 = t38 * in1[1];
  t208 = t38 * in1[2];
  t209 = t45 * t61 * t154 * in1[2];
  t44 = t59 * t60 * t154;
  t38 = t61 * t70 * t154;
  t221 = t38 * in1[1];
  t9 = t38 * in1[2];
  t38 = t61 * t72 * t154;
  t223 = t38 * in1[0];
  t224 = t61 * t71 * t154 * in1[1];
  t86 = t38 * in1[2];
  t38 = t154 * t238;
  t287 = t38 * in1[0];
  t288 = t38 * in1[1];
  t289 = t38 * in1[2];
  t294 = t26 * t154 * t238;
  t210_tmp = t31 * t60 * t154;
  t115 = t58 * t60 * t154;
  t33 = t60 * t70;
  t92 = t60 * t72;
  t269_tmp = t60 * t73;
  t292_tmp = in2[0] * t154;
  t38 = t45 * t60;
  t5 = t60 * t71;
  t297 = t38 * t154 + t5 * t154;
  t298 = t205 + t60 * -t73 * t154;
  t303 = t33 * t154 + t92 * t154;
  t304 = t303 * t303;
  t305 = 1.0 / t303;
  t307_tmp = t31 * t154;
  t307 = t32 * t153 - t307_tmp * t238;
  t310_tmp = t58 * t154;
  t310 = t104 * t153 - t310_tmp * t238;
  t311_tmp = t59 * t154;
  t311 = t105 * t153 - t311_tmp * t238;
  t313 = (-t177 + t38 * t136 * t155 / 2.0) + t5 * t136 * t155 / 2.0;
  t314 = (-t7 + t33 * t132 * t155 / 2.0) + t92 * t132 * t155 / 2.0;
  t315 = (t57 + t33 * t136 * t155 / 2.0) + t92 * t136 * t155 / 2.0;
  t317 = (-t7 + t38 * t140 * t155 / 2.0) + t5 * t140 * t155 / 2.0;
  t324 = ((((-t171 - t175) + t9) + t86) + t33 * t139 * t155 / 2.0) +
         t92 * t139 * t155 / 2.0;
  t325 = ((((-t83 * t154 - t210_tmp) + t18 * t31 * t61 * t154) + t224) +
          t38 * t144 * t155 / 2.0) +
         t5 * t144 * t155 / 2.0;
  t327 = ((((-t85 * t154 + t209) - t115) + t19 * t58 * t61 * t154) +
          t38 * t139 * t155 / 2.0) +
         t5 * t139 * t155 / 2.0;
  t328 = ((((t30 * t60 * t154 + t221) - t44) + t18 * t59 * t61 * t154) +
          t33 * t144 * t155 / 2.0) +
         t92 * t144 * t155 / 2.0;
  t312 = (t177 - t205_tmp * t132 * t155 / 2.0) + t269_tmp * t132 * t155 / 2.0;
  t316 = (t57 + t205_tmp * t140 * t155 / 2.0) - t269_tmp * t140 * t155 / 2.0;
  t318 = ((t93 + t147) - t38 * t132 * t155 / 2.0) - t5 * t132 * t155 / 2.0;
  t319 = ((t186 + t147) - t205_tmp * t136 * t155 / 2.0) +
         t269_tmp * t136 * t155 / 2.0;
  t320 = ((t186 + t93) - t33 * t140 * t155 / 2.0) - t92 * t140 * t155 / 2.0;
  t38 = ((((t175 + t201) + t94) + t9) - t38 * t138 * t155 / 2.0) -
        t5 * t138 * t155 / 2.0;
  t18 = ((((t171 + t201) - t94) + t86) - t205_tmp * t144 * t155 / 2.0) +
        t269_tmp * t144 * t155 / 2.0;
  t83 = ((((t87 * t154 + t208) + t44) - t19 * t59 * t61 * t154) +
         t205_tmp * t139 * t155 / 2.0) -
        t269_tmp * t139 * t155 / 2.0;
  t85 = t138 * t152;
  t175 = ((((t29 * t60 * t154 + t223) - t115) + t17 * t58 * t61 * t154) -
          t33 * t138 * t155 / 2.0) -
         t92 * t138 * t155 / 2.0;
  grad_v_tmp_tmp = t61 * -t73 * t154 * in1[0];
  t57 = ((((t84 * t154 - t210_tmp) + t17 * t31 * t61 * t154) -
          t205_tmp * t138 * t155 / 2.0) +
         t269_tmp * t138 * t155 / 2.0) +
        grad_v_tmp_tmp;
  t87 = t31 * t52;
  t73 = t52 * t58;
  t84 = t52 * t59;
  t17 = t31 * t138 * t155;
  grad_v_tmp = t52 * t149;
  b_grad_v_tmp = t52 * t148;
  t115 = t53 * t148 * t297;
  t33 = t53 * t149 * t298;
  t92 = t53 * t150 * t303;
  c_grad_v_tmp = t151 * t297;
  d_grad_v_tmp = t151 * t303;
  e_grad_v_tmp = t151 * t310;
  f_grad_v_tmp = t151 * t311;
  g_grad_v_tmp = t151 * t298;
  t94 = t26 * t52;
  t44 = t87 * t154;
  t177 = t73 * t154;
  t186 = t84 * t154;
  t201 = dJ_dnt * t21;
  t147 = t4 * t61;
  t205_tmp = dJ_dnn * t21;
  t171 = dJ_dmu * t304 * (1.0 / (t39 * t151 + t304));
  t210_tmp = t307_tmp * (1.0 / t304);
  t9 = in3[1] * t153 - in2[1] * t154 * t238;
  t86 = in2[1] * t52;
  grad_v_ptr[0] =
      (-dJ_domg0 *
           ((((((((-t151 * t307 * t175 -
                   d_grad_v_tmp *
                       (((t9 + t32 * t138 * t154 / 2.0) - t307_tmp * t189) +
                        t17 * t238 / 2.0)) +
                  c_grad_v_tmp * ((t105 * t138 * t154 / 2.0 - t311_tmp * t189) +
                                  t59 * t138 * t155 * t238 / 2.0)) +
                 e_grad_v_tmp * t57) +
                g_grad_v_tmp * ((((t160 - t294) + t104 * t138 * t154 / 2.0) -
                                 t310_tmp * t189) +
                                t58 * t138 * t155 * t238 / 2.0)) +
               f_grad_v_tmp * t38) -
              t85 * t297 * t311) -
             t85 * t298 * t310) +
            t85 * t303 * t307) +
       ((-dJ_domg1 * ((((((((-t52 * t150 * t175 - grad_v_tmp * t57) +
                            t52 * t166 * t297) +
                           b_grad_v_tmp * t38) +
                          t52 * t181 * t298) +
                         t52 * t183 * t303) +
                        t115 * in1[0] * 2.0) -
                       t33 * in1[0] * 2.0) -
                      t92 * in1[0] * 2.0) +
         dJ_domg2 * ((((((((((t86 * t150 * t154 - t94 * t149 * t154) +
                             t50 * t53 * t150 * t154) -
                            t44 * t183) -
                           t53 * t70 * t149 * t154 * 2.0) +
                          t186 * t166) +
                         t177 * t181) -
                        t87 * t138 * t150 * t155 / 2.0) +
                       t73 * t138 * t149 * t155 / 2.0) -
                      t84 * t138 * t148 * t155 / 2.0) +
                     t53 * t59 * t148 * t154 * in1[0] * 2.0)) -
        t201 * (((-in2[0] * t60 + t147 * in1[0]) + t6 * t61 * in1[1]) +
                t29 * t61 * in1[2]))) +
      (t205_tmp * ((-in2[0] * t38 + t26 * t175) + in2[1] * t57) -
       t171 * ((-in2[1] * t154 * t305 + t17 * t305 / 2.0) + t210_tmp * t175));
  t85 = ((in3[0] * t31 + in2[0] * t32) - in3[2] * t59) - t26 * t105;
  t175 = t144 * t152;
  t57 = t31 * t144 * t155;
  t17 = t52 * t150;
  t5 = in3[0] * t153 - t292_tmp * t238;
  t93 = t151 * t307;
  t7 = in2[0] * t52;
  t38 = t8 * t61;
  grad_v_ptr[1] =
      (dJ_domg1 * ((((((((t52 * t167 * t298 - grad_v_tmp * t18) -
                         b_grad_v_tmp * t325) +
                        t52 * t180 * t297) +
                       t17 * t328) -
                      t52 * t185 * t303) -
                     t115 * in1[1] * 2.0) +
                    t33 * in1[1] * 2.0) +
                   t92 * in1[1] * 2.0) +
       dJ_domg0 *
           ((((((((c_grad_v_tmp * ((((-t160 + t294) - t311_tmp * t85) +
                                    t105 * t144 * t154 / 2.0) +
                                   t59 * t144 * t155 * t238 / 2.0) -
                   d_grad_v_tmp *
                       (((t5 - t307_tmp * t85) + t32 * t144 * t154 / 2.0) +
                        t57 * t238 / 2.0)) +
                  e_grad_v_tmp * t18) +
                 t93 * t328) -
                f_grad_v_tmp * t325) +
               g_grad_v_tmp * ((-t58 * t154 * t85 + t104 * t144 * t154 / 2.0) +
                               t58 * t144 * t155 * t238 / 2.0)) -
              t175 * t297 * t311) -
             t175 * t298 * t310) +
            t175 * t303 * t307)) +
      (((-dJ_domg2 * ((((((((((t7 * t150 * t154 - t94 * t148 * t154) -
                              t45 * t53 * t150 * t154 * 2.0) +
                             t44 * t185) -
                            t53 * t72 * t148 * t154 * 2.0) +
                           t177 * t167) +
                          t186 * t180) -
                         t87 * t144 * t150 * t155 / 2.0) +
                        t73 * t144 * t149 * t155 / 2.0) -
                       t84 * t144 * t148 * t155 / 2.0) +
                      t53 * t58 * t149 * t154 * in1[1] * 2.0) -
         t201 * (((-in2[1] * t60 + t147 * in1[1]) + t38 * in1[1]) +
                 t30 * t61 * in1[2])) -
        t205_tmp * ((in2[1] * t18 + in2[0] * t325) - t26 * t328)) -
       t171 * ((t292_tmp * t305 - t57 * t305 / 2.0) + t210_tmp * t328));
  t85 = t139 * t152;
  t175 = t31 * t139 * t155;
  grad_v_ptr[2] =
      (dJ_domg1 *
           ((((((((t52 * t168 * t303 + t17 * t324) - b_grad_v_tmp * t327) +
                 grad_v_tmp * t83) +
                t52 * t182 * t297) -
               t52 * t184 * t298) -
              t115 * in1[2] * 2.0) +
             t33 * in1[2] * 2.0) +
            t92 * in1[2] * 2.0) -
       dJ_domg0 *
           ((((((((-t151 * t298 *
                       (((t5 + t104 * t139 * t154 / 2.0) - t310_tmp * t229) +
                        t58 * t139 * t155 * t238 / 2.0) -
                   c_grad_v_tmp *
                       (((t9 + t105 * t139 * t154 / 2.0) - t311_tmp * t229) +
                        t59 * t139 * t155 * t238 / 2.0)) +
                  d_grad_v_tmp * ((t32 * t139 * t154 / 2.0 - t307_tmp * t229) +
                                  t175 * t238 / 2.0)) -
                 t93 * t324) +
                e_grad_v_tmp * t83) +
               f_grad_v_tmp * t327) +
              t85 * t297 * t311) +
             t85 * t298 * t310) -
            t85 * t303 * t307)) +
      (((dJ_domg2 *
             ((((((((((t7 * t149 * t154 - t86 * t148 * t154) + t44 * t168) -
                     t53 * t71 * t149 * t154 * 2.0) +
                    t53 * t82 * t148 * t154) -
                   t186 * t182) +
                  t177 * t184) +
                 t87 * t139 * t150 * t155 / 2.0) -
                t73 * t139 * t149 * t155 / 2.0) +
               t84 * t139 * t148 * t155 / 2.0) +
              t31 * t53 * t150 * t154 * in1[2] * 2.0) -
         t201 * (((-t26 * t60 + t19 * t26 * t61) + t147 * in1[2]) +
                 t38 * in1[2])) +
        t205_tmp * ((-in2[0] * t327 + in2[1] * t83) + t26 * t324)) +
       t171 * (t175 * t305 / 2.0 - t210_tmp * t324));
  t57 = t132 * t152;
  t115 = t31 * t132 * t155;
  t33 = t61 * t303;
  t92 = t17 * t154;
  t7 = t201 * t60;
  t9 = t154 * t305;
  t86 = grad_v_tmp * t154;
  t147 = t61 * t298;
  grad_a_ptr[0] =
      (dJ_domg1 * (((((t52 * t141 * t297 - grad_v_tmp * t312) + t17 * t314) +
                     b_grad_v_tmp * t318) +
                    t147 * in1[0] * in1[1]) +
                   t33 * in1[0] * in1[2]) +
       dJ_domg0 * ((((((((c_grad_v_tmp *
                              ((-t59 * t154 * t164 + t105 * t132 * t154 / 2.0) +
                               t59 * t132 * t155 * t238 / 2.0) +
                          g_grad_v_tmp * ((((t157 - t289) - t310_tmp * t164) +
                                           t104 * t132 * t154 / 2.0) +
                                          t58 * t132 * t155 * t238 / 2.0)) +
                         t93 * t314) +
                        e_grad_v_tmp * t312) +
                       f_grad_v_tmp * t318) +
                      d_grad_v_tmp * (((t288 - t32 * t132 * t154 / 2.0) +
                                       t307_tmp * t164) -
                                      t115 * t238 / 2.0)) -
                     t57 * t297 * t311) -
                    t57 * t298 * t310) +
                   t57 * t303 * t307)) +
      (((dJ_domg2 * (((((((t208 - t221) - t84 * t141 * t154) + t86 * in1[2]) -
                        t92 * in1[1]) +
                       t87 * t132 * t150 * t155 / 2.0) -
                      t73 * t132 * t149 * t155 / 2.0) +
                     t84 * t132 * t148 * t155 / 2.0) +
         t205_tmp * (((t297 - in2[1] * t312) + in2[0] * t318) + t26 * t314)) +
        t7 * in1[0]) -
       t171 * ((t9 * in1[1] - t115 * t305 / 2.0) + t210_tmp * t314));
  t57 = t136 * t152;
  t115 = t31 * t136 * t155;
  t5 = b_grad_v_tmp * t154;
  t38 = t61 * t297 * in1[0];
  grad_a_ptr[1] =
      (-dJ_domg1 * (((((t52 * t142 * t298 - b_grad_v_tmp * t313) + t17 * t315) -
                      grad_v_tmp * t319) +
                     t38 * in1[1]) -
                    t33 * in1[1] * in1[2]) +
       dJ_domg2 * (((((((t209 + t223) + t73 * t142 * t154) - t5 * in1[2]) +
                      t92 * in1[0]) -
                     t87 * t136 * t150 * t155 / 2.0) +
                    t73 * t136 * t149 * t155 / 2.0) -
                   t84 * t136 * t148 * t155 / 2.0)) +
      (((-dJ_domg0 *
             ((((((((-t151 * t303 *
                         (((-t287 + t32 * t136 * t154 / 2.0) +
                           t307_tmp * t165) +
                          t115 * t238 / 2.0) +
                     g_grad_v_tmp *
                         ((t310_tmp * t165 + t104 * t136 * t154 / 2.0) +
                          t58 * t136 * t155 * t238 / 2.0)) +
                    c_grad_v_tmp * ((((-t157 + t289) + t311_tmp * t165) +
                                     t105 * t136 * t154 / 2.0) +
                                    t59 * t136 * t155 * t238 / 2.0)) +
                   t93 * t315) -
                  f_grad_v_tmp * t313) +
                 e_grad_v_tmp * t319) -
                t57 * t297 * t311) -
               t57 * t298 * t310) +
              t57 * t303 * t307) +
         t205_tmp *
             ((((-t205 + t269_tmp * t154) + in2[0] * t313) + in2[1] * t319) -
              t26 * t315)) +
        t7 * in1[1]) +
       t171 * ((t9 * in1[0] - t115 * t305 / 2.0) + t210_tmp * t315));
  t57 = t140 * t152;
  t115 = t31 * t140 * t155;
  grad_a_ptr[2] =
      (-dJ_domg2 *
           (((((((t224 + t87 * t143 * t154) + grad_v_tmp_tmp) - t5 * in1[1]) +
               t86 * in1[0]) +
              t87 * t140 * t150 * t155 / 2.0) -
             t73 * t140 * t149 * t155 / 2.0) +
            t84 * t140 * t148 * t155 / 2.0) -
       dJ_domg1 *
           (((((t52 * t143 * t303 - b_grad_v_tmp * t317) + grad_v_tmp * t316) -
              t17 * t320) +
             t38 * in1[2]) -
            t147 * in1[1] * in1[2])) +
      (((dJ_domg0 * ((((((((d_grad_v_tmp *
                                ((t32 * t140 * t154 / 2.0 - t307_tmp * t156) +
                                 t115 * t238 / 2.0) +
                            e_grad_v_tmp * t316) +
                           t93 * t320) +
                          f_grad_v_tmp * t317) +
                         g_grad_v_tmp * (((t287 + t310_tmp * t156) -
                                          t104 * t140 * t154 / 2.0) -
                                         t58 * t140 * t155 * t238 / 2.0)) +
                        c_grad_v_tmp * (((t288 + t311_tmp * t156) -
                                         t105 * t140 * t154 / 2.0) -
                                        t59 * t140 * t155 * t238 / 2.0)) +
                       t57 * t297 * t311) +
                      t57 * t298 * t310) -
                     t57 * t303 * t307) -
         t205_tmp * (((t303 - in2[0] * t317) + in2[1] * t316) - t26 * t320)) +
        t7 * in1[2]) -
       t171 * (t115 * t305 / 2.0 + t210_tmp * t320));
  grad_j_ptr[0] = -dJ_domg0 * ((d_grad_v_tmp * (t162 - t31 * t122 * t154) -
                            g_grad_v_tmp * (t163 - t58 * t122 * t154)) +
                           t59 * t122 * t155 * t297);
  grad_j_ptr[1] = dJ_domg0 * ((d_grad_v_tmp * (t161 - t31 * t131 * t154) +
                           c_grad_v_tmp * (t163 + t59 * t131 * t154)) +
                          t58 * t131 * t155 * t298);
  grad_j_ptr[2] = -dJ_domg0 * ((g_grad_v_tmp * (t161 - t58 * t137 * t154) +




            grad_v.x() = grad_v_arr[0];
            grad_v.y() = grad_v_arr[1];
            grad_v.z() = grad_v_arr[2];

            grad_a.x() = grad_a_arr[0];
            grad_a.y() = grad_a_arr[1];
            grad_a.z() = grad_a_arr[2];

            grad_j.x() = grad_j_arr[0];
            grad_j.y() = grad_j_arr[1];
            grad_j.z() = grad_j_arr[2];
        }

        /**
         * @brief 反向梯度映射 (Backward Map / Gradient Backpropagation)
         *
         * 在飞行轨迹优化框架（如MINCO框架）中，用于反推梯度。
         * 给定代价函数（Loss Function）对推力、姿态四元数、角速度等控制和状态变量的梯度，
         * 通过链式法则求出代价函数对平坦输出（偏航、位置、速度、加速度和加加速度等）产生的总梯度。
         * 这部分通常是基于分析求导（Analytical Differentiation）获得。
         *
         * @param pos_grad 目标函数对位置的现有梯度
         * @param vel_grad 目标函数对速度的现有梯度
         * @param thr_grad 目标函数对推力的导数/梯度
         * @param quat_grad 目标函数对四元数姿态的梯度
         * @param omg_grad 目标函数对角速度的梯度
         * @param pos_total_grad [输出] 传递到当前点下，目标函数关于位置的总梯度
         * @param vel_total_grad [输出] 传递到当前点下，目标函数关于速度的总梯度
         * @param acc_total_grad [输出] 传递到当前点下，目标函数关于加速度的总梯度
         * @param jer_total_grad [输出] 传递到当前点下，目标函数关于加加速度的总梯度
         * @param psi_total_grad [输出] 传递到当前点下，目标函数关于偏航角的总梯度
         * @param dpsi_total_grad [输出] 传递到当前点下，目标函数关于偏航角速率的总梯度
         */
        inline void backward(const Eigen::Vector3d &pos_grad,
                             const Eigen::Vector3d &vel_grad,
                             const double &thr_grad,
                             const Eigen::Vector4d &quat_grad,
                             const Eigen::Vector3d &omg_grad,
                             Eigen::Vector3d &pos_total_grad,
                             Eigen::Vector3d &vel_total_grad,
                             Eigen::Vector3d &acc_total_grad,
                             Eigen::Vector3d &jer_total_grad,
                             double &psi_total_grad,
                             double &dpsi_total_grad) const
        {
            double w0b, w1b, w2b, dw0b, dw1b, dw2b;
            double z0b, z1b, z2b, dz0b, dz1b, dz2b;
            double v_sqr_normb, cp_termb, w_termb;
            double zu_sqr_normb, zu_normb, zu0b, zu1b, zu2b;
            double zu_sqr0b, zu_sqr1b, zu_sqr2b, zu01b, zu12b, zu02b;
            double ng00b, ng01b, ng02b, ng11b, ng12b, ng22b, ng_denb;
            double dz_term0b, dz_term1b, dz_term2b, f_term0b, f_term1b, f_term2b;
            double tilt_denb, tilt0b, tilt1b, tilt2b, head0b, head3b;
            double cpsib, spsib, omg_denb, omg_termb;
            double tempb, tilt_den_sqr;

            tilt0b = s_half_psi * (quat_grad(3)) + c_half_psi * (quat_grad(0));
            head3b = tilt0 * (quat_grad(3)) + tilt2 * (quat_grad(1)) - tilt1 * (quat_grad(2));
            tilt2b = c_half_psi * (quat_grad(2)) + s_half_psi * (quat_grad(1));
            head0b = tilt2 * (quat_grad(2)) + tilt1 * (quat_grad(1)) + tilt0 * (quat_grad(0));
            tilt1b = c_half_psi * (quat_grad(1)) - s_half_psi * (quat_grad(2));
            tilt_den_sqr = tilt_den * tilt_den;
            tilt_denb = (z1 * tilt1b - z0 * tilt2b) / tilt_den_sqr + 0.5 * tilt0b;
            omg_termb = -((z0 * c_psi + z1 * s_psi) * (omg_grad(1))) -
                        (z0 * s_psi - z1 * c_psi) * (omg_grad(0));
            tempb = omg_grad(2) / omg_den;
            dpsi_total_grad = omg_grad(2);
            z1b = dz0 * tempb;
            dz0b = z1 * tempb + c_psi * (omg_grad(1)) + s_psi * (omg_grad(0));
            z0b = -(dz1 * tempb);
            dz1b = s_psi * (omg_grad(1)) - z0 * tempb - c_psi * (omg_grad(0));
            omg_denb = -((z1 * dz0 - z0 * dz1) * tempb / omg_den) -
                       dz2 * omg_termb / (omg_den * omg_den);
            tempb = -(omg_term * (omg_grad(1)));
            cpsib = dz0 * (omg_grad(1)) + z0 * tempb;
            spsib = dz1 * (omg_grad(1)) + z1 * tempb;
            z0b += c_psi * tempb;
            z1b += s_psi * tempb;
            tempb = -(omg_term * (omg_grad(0)));
            spsib += dz0 * (omg_grad(0)) + z0 * tempb;
            cpsib += -dz1 * (omg_grad(0)) - z1 * tempb;
            z0b += s_psi * tempb + tilt2b / tilt_den + f_term0 * (thr_grad);
            z1b += -c_psi * tempb - tilt1b / tilt_den + f_term1 * (thr_grad);
            dz2b = omg_termb / omg_den;
            z2b = omg_denb + tilt_denb / tilt_den + f_term2 * (thr_grad);
            psi_total_grad = c_psi * spsib + 0.5 * c_half_psi * head3b -
                             s_psi * cpsib - 0.5 * s_half_psi * head0b;
            f_term0b = z0 * (thr_grad);
            f_term1b = z1 * (thr_grad);
            f_term2b = z2 * (thr_grad);
            ng02b = dz_term0 * dz2b + dz_term2 * dz0b;
            dz_term0b = ng02 * dz2b + ng01 * dz1b + ng00 * dz0b;
            ng12b = dz_term1 * dz2b + dz_term2 * dz1b;
            dz_term1b = ng12 * dz2b + ng11 * dz1b + ng01 * dz0b;
            ng22b = dz_term2 * dz2b;
            dz_term2b = ng22 * dz2b + ng12 * dz1b + ng02 * dz0b;
            ng01b = dz_term0 * dz1b + dz_term1 * dz0b;
            ng11b = dz_term1 * dz1b;
            ng00b = dz_term0 * dz0b;
            jer_total_grad(2) = dz_term2b;
            dw2b = dh_over_m * dz_term2b;
            jer_total_grad(1) = dz_term1b;
            dw1b = dh_over_m * dz_term1b;
            jer_total_grad(0) = dz_term0b;
            dw0b = dh_over_m * dz_term0b;
            tempb = cp * (v2 * dw2b + v1 * dw1b + v0 * dw0b) / cp_term;
            acc_total_grad(2) = mass * f_term2b + w_term * dw2b + v2 * tempb;
            acc_total_grad(1) = mass * f_term1b + w_term * dw1b + v1 * tempb;
            acc_total_grad(0) = mass * f_term0b + w_term * dw0b + v0 * tempb;
            vel_total_grad(2) = dw_term * dw2b + a2 * tempb;
            vel_total_grad(1) = dw_term * dw1b + a1 * tempb;
            vel_total_grad(0) = dw_term * dw0b + a0 * tempb;
            cp_termb = -(v_dot_a * tempb / cp_term);
            tempb = ng22b / ng_den;
            zu_sqr0b = tempb;
            zu_sqr1b = tempb;
            ng_denb = -((zu_sqr0 + zu_sqr1) * tempb / ng_den);
            zu12b = -(ng12b / ng_den);
            tempb = ng11b / ng_den;
            ng_denb += zu12 * ng12b / (ng_den * ng_den) -
                       (zu_sqr0 + zu_sqr2) * tempb / ng_den;
            zu_sqr0b += tempb;
            zu_sqr2b = tempb;
            zu02b = -(ng02b / ng_den);
            zu01b = -(ng01b / ng_den);
            tempb = ng00b / ng_den;
            ng_denb += zu02 * ng02b / (ng_den * ng_den) +
                       zu01 * ng01b / (ng_den * ng_den) -
                       (zu_sqr1 + zu_sqr2) * tempb / ng_den;
            zu_normb = zu_sqr_norm * ng_denb -
                       (zu2 * z2b + zu1 * z1b + zu0 * z0b) / zu_sqr_norm;
            zu_sqr_normb = zu_norm * ng_denb + zu_normb / (2.0 * zu_norm);
            tempb += zu_sqr_normb;
            zu_sqr1b += tempb;
            zu_sqr2b += tempb;
            zu2b = z2b / zu_norm + zu0 * zu02b + zu1 * zu12b + 2 * zu2 * zu_sqr2b;
            w2b = dv * f_term2b + dh_over_m * zu2b;
            zu1b = z1b / zu_norm + zu2 * zu12b + zu0 * zu01b + 2 * zu1 * zu_sqr1b;
            w1b = dv * f_term1b + dh_over_m * zu1b;
            zu_sqr0b += zu_sqr_normb;
            zu0b = z0b / zu_norm + zu2 * zu02b + zu1 * zu01b + 2 * zu0 * zu_sqr0b;
            w0b = dv * f_term0b + dh_over_m * zu0b;
            w_termb = a2 * dw2b + a1 * dw1b + a0 * dw0b +
                      v2 * w2b + v1 * w1b + v0 * w0b;
            acc_total_grad(2) += zu2b;
            acc_total_grad(1) += zu1b;
            acc_total_grad(0) += zu0b;
            cp_termb += cp * w_termb;
            v_sqr_normb = cp_termb / (2.0 * cp_term);
            vel_total_grad(2) += w_term * w2b + 2 * v2 * v_sqr_normb + vel_grad(2);
            vel_total_grad(1) += w_term * w1b + 2 * v1 * v_sqr_normb + vel_grad(1);
            vel_total_grad(0) += w_term * w0b + 2 * v0 * v_sqr_normb + vel_grad(0);
            pos_total_grad(2) = pos_grad(2);
            pos_total_grad(1) = pos_grad(1);
            pos_total_grad(0) = pos_grad(0);

            return;
        }

    private:
        double mass, grav, dh, dv, cp, veps;

        double v0, v1, v2, a0, a1, a2, v_dot_a;
        double z0, z1, z2, dz0, dz1, dz2;
        double cp_term, w_term, dh_over_m;
        double zu_sqr_norm, zu_norm, zu0, zu1, zu2;
        double zu_sqr0, zu_sqr1, zu_sqr2, zu01, zu12, zu02;
        double ng00, ng01, ng02, ng11, ng12, ng22, ng_den;
        double dw_term, dz_term0, dz_term1, dz_term2, f_term0, f_term1, f_term2;
        double tilt_den, tilt0, tilt1, tilt2, c_half_psi, s_half_psi;
        double c_psi, s_psi, omg_den, omg_term;
    };
}

#endif
