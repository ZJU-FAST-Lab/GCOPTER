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
         */
        inline void forward(const Eigen::Vector3d &vel, // 大地系下的速度
                            const Eigen::Vector3d &acc, // 大地系下的加速度
                            const Eigen::Vector3d &jer, // 大地系下的加加速度
                            double &thr, // 输出：期望前向推力
                            Eigen::Vector4d &quat, // 输出：期望姿态四元数
                            Eigen::Vector3d &omg) // 输出：期望气流系角速度
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

            // ======= 3. 角速度与导向量计算 =======
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

            // ======= 4. 期望推力计算 =======
            // 在质点模型下，沿着速度方向的总气体力即为推力减去阻力
            thr = mass * a_g.dot(i_a) + cp * v_norm * v_norm;

            return;
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
