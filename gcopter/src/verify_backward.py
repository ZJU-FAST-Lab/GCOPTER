#!/usr/bin/env python3
# =========================================================================
# 固定翼平坦度反向梯度 正确性验证脚本
# 使用有限差分 (Finite Difference) 验证 MATLAB Coder 生成的解析梯度
# =========================================================================

import numpy as np
from numpy.linalg import norm

# =========================================================================
# 1. 正向平坦度映射 (Python 复现，与 C++ forward() 完全等价)
# =========================================================================
def forward_flatness(v, a, jer, grav=9.8):
    """
    正向平坦度映射：已知(v, a, j)，计算(n_t, n_n, mu, omg)
    与 flatness.hpp::forward() 算法一致
    """
    v_norm = norm(v)
    v_norm_inv = 1.0 / (v_norm + 1e-6)
    i_a = v * v_norm_inv

    a_g = a.copy()
    a_g[2] -= grav  # NED: 重力方向为 [0,0,g]

    v_times_ag = np.cross(v, a_g)
    v_times_ag_norm = norm(v_times_ag)
    v_times_ag_inv = 1.0 / (v_times_ag_norm + 1e-6)
    j_a = v_times_ag * v_times_ag_inv

    k_a = np.cross(i_a, j_a)

    # 姿态四元数（本次验证不需要）
    # R = np.column_stack([i_a, j_a, k_a])
    # quat = R_to_quat(R)

    # 角速度
    v_dot_a = np.dot(v, a)
    i_a_dot = (a * v_norm - v * v_dot_a * v_norm_inv) * (v_norm_inv * v_norm_inv)

    cross_dot = np.cross(a, a_g) + np.cross(v, jer)
    cross_dot_inner = np.dot(v_times_ag, cross_dot)
    j_a_dot = (cross_dot * v_times_ag_norm - v_times_ag * cross_dot_inner * v_times_ag_inv) * (v_times_ag_inv * v_times_ag_inv)

    omg = np.array([
        np.dot(k_a, j_a_dot),    # omg_x
        -np.dot(k_a, i_a_dot),   # omg_y
        np.dot(j_a, i_a_dot)     # omg_z
    ])

    # 控制量
    n_t = np.dot(a_g, i_a) / grav
    n_n = -np.dot(a_g, k_a) / grav
    mu = np.arctan2(j_a[2], k_a[2])

    return n_t, n_n, mu, omg


# =========================================================================
# 2. 有限差分梯度计算
# =========================================================================
def finite_difference_gradient(func, x, eps=1e-6):
    """
    对标量函数 func(x) 使用中心差分计算梯度
    func: (ndarray) -> float
    x: ndarray
    返回: ndarray, 与 x 同形状
    """
    grad = np.zeros_like(x, dtype=np.float64)
    for i in range(len(x)):
        x_plus = x.copy()
        x_minus = x.copy()
        x_plus[i] += eps
        x_minus[i] -= eps
        grad[i] = (func(x_plus) - func(x_minus)) / (2.0 * eps)
    return grad


def numerical_backward(v, a, jer, dJ_dnt, dJ_dnn, dJ_dmu, dJ_domg, grav=9.8):
    """
    使用有限差分计算 ∂J/∂v, ∂J/∂a, ∂J/∂jer
    基于链式法则: J = dJ_dnt * n_t(v,a,j) + dJ_dnn * n_n(v,a,j) + dJ_dmu * mu(v,a,j) + dJ_domg · omg(v,a,j)
    """

    def J_v(x):
        n_t, n_n, mu, omg = forward_flatness(x, a, jer, grav)
        return float(dJ_dnt * n_t + dJ_dnn * n_n + dJ_dmu * mu + np.dot(dJ_domg, omg))

    def J_a(x):
        n_t, n_n, mu, omg = forward_flatness(v, x, jer, grav)
        return float(dJ_dnt * n_t + dJ_dnn * n_n + dJ_dmu * mu + np.dot(dJ_domg, omg))

    def J_j(x):
        n_t, n_n, mu, omg = forward_flatness(v, a, x, grav)
        return float(dJ_dnt * n_t + dJ_dnn * n_n + dJ_dmu * mu + np.dot(dJ_domg, omg))

    grad_v_num = finite_difference_gradient(J_v, v)
    grad_a_num = finite_difference_gradient(J_a, a)
    grad_j_num = finite_difference_gradient(J_j, jer)

    return grad_v_num, grad_a_num, grad_j_num


# =========================================================================
# 3. 解析反向梯度 (从 MATLAB Coder 闭式解复现)
# =========================================================================
def analytical_backward(v, a, jer, dJ_dnt, dJ_dnn, dJ_dmu, dJ_domg, grav=9.8):
    """
    解析反向梯度 —— 从 backward_gradients.cpp 完全等价翻译
    使用与 MATLAB Coder 相同的 CSE 变量名以保持可追溯性
    """
    in1 = v.astype(np.float64)
    in2 = a.astype(np.float64)
    in3 = jer.astype(np.float64)

    # -----------------------------------------------------------
    #  阶段 1: 公共子表达式消除 (CSE)
    # -----------------------------------------------------------
    t4 = in2[0] * in1[0]
    t5 = in2[0] * in1[1]
    t6 = in1[0] * in2[1]
    t7 = in2[0] * in1[2]
    t8 = in2[1] * in1[1]
    t9 = in2[1] * in1[2]
    t17 = in1[0] * in1[0]
    t18 = in1[1] * in1[1]
    t19 = in1[2] * in1[2]
    t21 = 1.0 / grav
    t26 = in2[2] - grav
    t33 = (t4 + t8) + in2[2] * in1[2]
    t38 = (t17 + t18) + t19
    t29 = t26 * in1[0]
    t30 = t26 * in1[1]
    t31 = t5 - t6
    t32 = in3[0] * in1[1] - in1[0] * in3[1]
    t52 = 1.0 / t38
    t57 = np.sqrt(t38)
    t39 = t31 * t31
    t44 = t31 * in1[0]
    t45 = t31 * in1[1]
    t53 = t52 * t52
    t60 = 1.0 / t57
    t50 = t44 * 2.0
    t58 = t7 - t29
    t59 = t9 - t30
    t61 = t60 * t60 * t60   # rt_powd_snf(t60, 3.0)
    t83 = t5 * t60
    t84 = t6 * t60
    t85 = t7 * t60
    t38_ = in2[2] * t60
    t86 = t38_ * in1[0]
    t87 = t9 * t60
    t9_ = t38_ * in1[1]
    t92 = t17 * t60
    t93 = t18 * t60
    t94 = t19 * t60
    t104 = ((in2[0] * in2[2] + in3[0] * in1[2]) - in1[0] * in3[2]) - in2[0] * t26
    t105 = ((in2[1] * in2[2] + in3[1] * in1[2]) - in1[1] * in3[2]) - in2[1] * t26
    t115 = t33 * t60
    t70 = t58 * in1[0]
    t71 = t58 * in1[2]
    t72 = t59 * in1[1]
    t73 = t59 * in1[2]
    t5_ = t33 * t61
    t38__ = t5_ * in1[0]
    t7_ = t38__ * in1[1]
    t38__ *= in1[2]
    t5__ = t5_ * in1[1] * in1[2]
    t82 = t73 * 2.0
    t122 = t45 + t71
    t137_ = t70 + t72
    t138 = in2[1] * t31 * 2.0 + t26 * t58 * 2.0
    t139 = in2[0] * t58 * 2.0 + in2[1] * t59 * 2.0
    t141 = t57 - t92
    t142 = t57 - t93
    t143 = t57 - t94
    t147 = (t39 + t58 * t58) + t59 * t59
    t156 = t104 * in1[0] + t105 * in1[1]
    t164 = (t32 * in1[1] + grav * t58) + t104 * in1[2]
    t165 = (-(t32 * in1[0]) + grav * t59) + t105 * in1[2]
    t180 = (t83 + -t6 * t60) + t7_
    t181 = (t84 - t83) + t7_
    t182 = (t85 - t86) + t38__
    t183 = (t86 - t85) + t38__
    t184 = (t87 - t9_) + t5__
    t185 = (t9_ - t87) + t5__
    t189 = ((in3[1] * t31 + in2[1] * t32) + in3[2] * t58) + t26 * t104
    t229 = ((in3[0] * t58 + in3[1] * t59) + in2[0] * t104) + in2[1] * t105
    t238 = (t31 * t32 + t58 * t104) + t59 * t105
    t131 = t44 - t73
    t132 = t45 * 2.0 + t71 * 2.0
    t140 = t70 * 2.0 + t72 * 2.0
    t144 = in2[0] * t31 * 2.0 - t26 * t59 * 2.0
    t148 = in2[0] * t57 - t115 * in1[0]
    t149 = in2[1] * t57 - t115 * in1[1]
    t150 = in2[2] * t57 - t115 * in1[2]
    t151 = 1.0 / t147
    t153 = np.sqrt(t147)
    t166 = t115 - t17 * t33 * t61
    t167 = t115 - t18 * t33 * t61
    t168 = t115 - t19 * t33 * t61
    t136 = t50 - t82
    t152 = t151 * t151
    t154 = 1.0 / t153
    t157 = grav * t153
    t160 = in3[2] * t153
    t161 = t153 * in1[0]
    t162 = t153 * in1[1]
    t163 = t153 * in1[2]
    t155 = t154 * t154 * t154   # rt_powd_snf(t154, 3.0)
    t171 = t4 * t60 * t154
    t175 = t8 * t60 * t154
    t5___ = t60 * t154
    t38___ = t5___ * in1[0]
    t177 = t38___ * in1[1]
    t7__ = t38___ * in1[2]
    t57_ = t5___ * in1[1] * in1[2]
    t186_ = t92 * t154
    t93_ = t93 * t154
    t147_ = t94 * t154
    t201 = t26 * t60 * t154 * in1[2]
    t205_tmp = t44 * t60
    t205 = t205_tmp * t154
    t38____ = t44 * t61 * t154
    t94_ = t38____ * in1[1]
    t208 = t38____ * in1[2]
    t209 = t45 * t61 * t154 * in1[2]
    t44_ = t59 * t60 * t154
    t38_____ = t61 * t70 * t154
    t221 = t38_____ * in1[1]
    t9__ = t38_____ * in1[2]
    t38______ = t61 * t72 * t154
    t223 = t38______ * in1[0]
    t224 = t61 * t71 * t154 * in1[1]
    t86_ = t38______ * in1[2]
    t38_______ = t154 * t238
    t287 = t38_______ * in1[0]
    t288 = t38_______ * in1[1]
    t289 = t38_______ * in1[2]
    t294 = t26 * t154 * t238
    t210_tmp = t31 * t60 * t154
    t115_ = t58 * t60 * t154
    t33_ = t60 * t70
    t92_ = t60 * t72
    t269_tmp = t60 * t73
    t292_tmp = in2[0] * t154
    t38________ = t45 * t60
    t5____ = t60 * t71
    t297 = t38________ * t154 + t5____ * t154
    t298 = t205 + t60 * -t73 * t154
    t303 = t33_ * t154 + t92_ * t154
    t304 = t303 * t303
    t305 = 1.0 / t303
    t307_tmp = t31 * t154
    t307 = t32 * t153 - t307_tmp * t238
    t310_tmp = t58 * t154
    t310 = t104 * t153 - t310_tmp * t238
    t311_tmp = t59 * t154
    t311 = t105 * t153 - t311_tmp * t238
    t313 = (-t177 + t38________ * t136 * t155 / 2.0) + t5____ * t136 * t155 / 2.0
    t314 = (-t7__ + t33_ * t132 * t155 / 2.0) + t92_ * t132 * t155 / 2.0
    t315 = (t57_ + t33_ * t136 * t155 / 2.0) + t92_ * t136 * t155 / 2.0
    t317 = (-t7__ + t38________ * t140 * t155 / 2.0) + t5____ * t140 * t155 / 2.0
    t324 = ((((-t171 - t175) + t9__) + t86_) + t33_ * t139 * t155 / 2.0) + t92_ * t139 * t155 / 2.0
    t325 = ((((-t83 * t154 - t210_tmp) + t18 * t31 * t61 * t154) + t224) + t38________ * t144 * t155 / 2.0) + t5____ * t144 * t155 / 2.0
    t327 = ((((-t85 * t154 + t209) - t115_) + t19 * t58 * t61 * t154) + t38________ * t139 * t155 / 2.0) + t5____ * t139 * t155 / 2.0
    t328 = ((((t30 * t60 * t154 + t221) - t44_) + t18 * t59 * t61 * t154) + t33_ * t144 * t155 / 2.0) + t92_ * t144 * t155 / 2.0
    t312 = (t177 - t205_tmp * t132 * t155 / 2.0) + t269_tmp * t132 * t155 / 2.0
    t316 = (t57_ + t205_tmp * t140 * t155 / 2.0) - t269_tmp * t140 * t155 / 2.0
    t318 = ((t93_ + t147_) - t38________ * t132 * t155 / 2.0) - t5____ * t132 * t155 / 2.0
    t319 = ((t186_ + t147_) - t205_tmp * t136 * t155 / 2.0) + t269_tmp * t136 * t155 / 2.0
    t320 = ((t186_ + t93_) - t33_ * t140 * t155 / 2.0) - t92_ * t140 * t155 / 2.0
    t38__a = ((((t175 + t201) + t94_) + t9__) - t38________ * t138 * t155 / 2.0) - t5____ * t138 * t155 / 2.0
    t18_ = ((((t171 + t201) - t94_) + t86_) - t205_tmp * t144 * t155 / 2.0) + t269_tmp * t144 * t155 / 2.0
    t83_ = ((((t87 * t154 + t208) + t44_) - t19 * t59 * t61 * t154) + t205_tmp * t139 * t155 / 2.0) - t269_tmp * t139 * t155 / 2.0
    t85_ = t138 * t152
    t175_ = ((((t29 * t60 * t154 + t223) - t115_) + t17 * t58 * t61 * t154) - t33_ * t138 * t155 / 2.0) - t92_ * t138 * t155 / 2.0
    grad_v_tmp_tmp = t61 * -t73 * t154 * in1[0]
    t57__ = ((((t84 * t154 - t210_tmp) + t17 * t31 * t61 * t154) - t205_tmp * t138 * t155 / 2.0) + t269_tmp * t138 * t155 / 2.0) + grad_v_tmp_tmp
    t87_ = t31 * t52
    t73_ = t52 * t58
    t84_ = t52 * t59
    t17_ = t31 * t138 * t155
    grad_v_tmp = t52 * t149
    b_grad_v_tmp = t52 * t148
    t115__ = t53 * t148 * t297
    t33__ = t53 * t149 * t298
    t92__ = t53 * t150 * t303
    c_grad_v_tmp = t151 * t297
    d_grad_v_tmp = t151 * t303
    e_grad_v_tmp = t151 * t310
    f_grad_v_tmp = t151 * t311
    g_grad_v_tmp = t151 * t298
    t94__ = t26 * t52
    t44__ = t87_ * t154
    t177_ = t73_ * t154
    t186__ = t84_ * t154
    t201_ = dJ_dnt * t21
    t147__ = t4 * t61
    t205_tmp_ = dJ_dnn * t21
    t171_ = dJ_dmu * t304 * (1.0 / (t39 * t151 + t304))
    t210_tmp_ = t307_tmp * (1.0 / t304)
    t9___ = in3[1] * t153 - in2[1] * t154 * t238
    t86__ = in2[1] * t52

    # -----------------------------------------------------------
    #  阶段 2: grad_v[0], grad_v[1], grad_v[2]
    # -----------------------------------------------------------
    grad_v = np.zeros(3)
    grad_v[0] = (
        -dJ_domg[0] * (
            (-t151 * t307 * t175_ -
             d_grad_v_tmp * (
                 ((t9___ + t32 * t138 * t154 / 2.0) - t307_tmp * t189) + t17_ * t238 / 2.0
             ) +
             c_grad_v_tmp * ((t105 * t138 * t154 / 2.0 - t311_tmp * t189) +
                             t59 * t138 * t155 * t238 / 2.0) +
             e_grad_v_tmp * t57__ +
             g_grad_v_tmp * ((((t160 - t294) + t104 * t138 * t154 / 2.0) - t310_tmp * t189) +
                             t58 * t138 * t155 * t238 / 2.0) +
             f_grad_v_tmp * t38__a) -
            t85_ * t297 * t311 -
            t85_ * t298 * t310 +
            t85_ * t303 * t307
        ) +
        (-dJ_domg[1] * ((-t52 * t150 * t175_ - grad_v_tmp * t57__ +
                         t52 * t166 * t297 + b_grad_v_tmp * t38__a +
                         t52 * t181 * t298 + t52 * t183 * t303 +
                         t115__ * in1[0] * 2.0 -
                         t33__ * in1[0] * 2.0 - t92__ * in1[0] * 2.0)) +
         dJ_domg[2] * (
             (t86__ * t150 * t154 - t94__ * t149 * t154 +
              t50 * t53 * t150 * t154 -
              t44__ * t183 -
              t53 * t70 * t149 * t154 * 2.0 +
              t186__ * t166 +
              t177_ * t181 -
              t87_ * t138 * t150 * t155 / 2.0 +
              t73_ * t138 * t149 * t155 / 2.0 -
              t84_ * t138 * t148 * t155 / 2.0 +
              t53 * t59 * t148 * t154 * in1[0] * 2.0)
         ) -
         t201_ * (((-in2[0] * t60 + t147__ * in1[0]) + t6 * t61 * in1[1]) + t29 * t61 * in1[2]))
    ) + (
        t205_tmp_ * ((-in2[0] * t38__a + t26 * t175_) + in2[1] * t57__) -
        t171_ * ((-in2[1] * t154 * t305 + t17_ * t305 / 2.0) + t210_tmp_ * t175_)
    )

    t85__ = ((in3[0] * t31 + in2[0] * t32) - in3[2] * t59) - t26 * t105
    t175__ = t144 * t152
    t57___ = t31 * t144 * t155
    t17__ = t52 * t150
    t5_____ = in3[0] * t153 - t292_tmp * t238
    t93__ = t151 * t307
    t7___ = in2[0] * t52
    t38__b = t8 * t61
    grad_v[1] = (
        dJ_domg[1] * ((t52 * t167 * t298 - grad_v_tmp * t18_ -
                       b_grad_v_tmp * t325 +
                       t52 * t180 * t297 + t17__ * t328 -
                       t52 * t185 * t303 -
                       t115__ * in1[1] * 2.0 +
                       t33__ * in1[1] * 2.0 + t92__ * in1[1] * 2.0)) +
        dJ_domg[0] * (
            (c_grad_v_tmp * ((((-t160 + t294) - t311_tmp * t85__) + t105 * t144 * t154 / 2.0) +
                             t59 * t144 * t155 * t238 / 2.0) -
             d_grad_v_tmp * (((t5_____ - t307_tmp * t85__) + t32 * t144 * t154 / 2.0) +
                             t57___ * t238 / 2.0) +
             e_grad_v_tmp * t18_ + t93__ * t328 -
             f_grad_v_tmp * t325 +
             g_grad_v_tmp * ((-t58 * t154 * t85__ + t104 * t144 * t154 / 2.0) +
                             t58 * t144 * t155 * t238 / 2.0)) -
            t175__ * t297 * t311 - t175__ * t298 * t310 + t175__ * t303 * t307
        )
    ) + (
        -dJ_domg[2] * (
            (t7___ * t150 * t154 - t94__ * t148 * t154 -
             t45 * t53 * t150 * t154 * 2.0 +
             t44__ * t185 - t53 * t72 * t148 * t154 * 2.0 +
             t177_ * t167 + t186__ * t180 -
             t87_ * t144 * t150 * t155 / 2.0 +
             t73_ * t144 * t149 * t155 / 2.0 -
             t84_ * t144 * t148 * t155 / 2.0 +
             t53 * t58 * t149 * t154 * in1[1] * 2.0)
        ) -
        t201_ * (((-in2[1] * t60 + t147__ * in1[1]) + t38__b * in1[1]) + t30 * t61 * in1[2]) -
        t205_tmp_ * ((in2[1] * t18_ + in2[0] * t325) - t26 * t328) -
        t171_ * ((t292_tmp * t305 - t57___ * t305 / 2.0) + t210_tmp_ * t328)
    )

    t85___ = t139 * t152
    t175___ = t31 * t139 * t155
    grad_v[2] = (
        dJ_domg[1] * ((t52 * t168 * t303 + t17__ * t324 - b_grad_v_tmp * t327 +
                       grad_v_tmp * t83_ +
                       t52 * t182 * t297 - t52 * t184 * t298 -
                       t115__ * in1[2] * 2.0 +
                       t33__ * in1[2] * 2.0 + t92__ * in1[2] * 2.0)) -
        dJ_domg[0] * (
            -t151 * t298 * (
                ((t5_____ + t104 * t139 * t154 / 2.0) - t310_tmp * t229) + t58 * t139 * t155 * t238 / 2.0
            ) -
             c_grad_v_tmp * (
                 ((t9___ + t105 * t139 * t154 / 2.0) - t311_tmp * t229) + t59 * t139 * t155 * t238 / 2.0
             ) +
             d_grad_v_tmp * ((t32 * t139 * t154 / 2.0 - t307_tmp * t229) + t175___ * t238 / 2.0) -
             t93__ * t324 +
             e_grad_v_tmp * t83_ + f_grad_v_tmp * t327 +
             t85___ * t297 * t311 + t85___ * t298 * t310 - t85___ * t303 * t307)
    ) + (
        dJ_domg[2] * (
            (t7___ * t149 * t154 - t86__ * t148 * t154 + t44__ * t168 -
             t53 * t71 * t149 * t154 * 2.0 + t53 * t82 * t148 * t154 -
             t186__ * t182 + t177_ * t184 +
             t87_ * t139 * t150 * t155 / 2.0 -
             t73_ * t139 * t149 * t155 / 2.0 +
             t84_ * t139 * t148 * t155 / 2.0 +
             t31 * t53 * t150 * t154 * in1[2] * 2.0)
        ) -
        t201_ * (((-t26 * t60 + t19 * t26 * t61) + t147__ * in1[2]) + t38__b * in1[2]) +
        t205_tmp_ * ((-in2[0] * t327 + in2[1] * t83_) + t26 * t324)
    ) + t171_ * (t175___ * t305 / 2.0 - t210_tmp_ * t324)

    # -----------------------------------------------------------
    #  阶段 3: grad_a[0], grad_a[1], grad_a[2]
    # -----------------------------------------------------------
    t57____ = t132 * t152
    t115___ = t31 * t132 * t155
    t33___ = t61 * t303
    t92___ = t17__ * t154
    t7____ = t201_ * t60
    t9____ = t154 * t305
    t86___ = grad_v_tmp * t154
    t147___ = t61 * t298
    grad_a = np.zeros(3)
    grad_a[0] = (
        dJ_domg[1] * ((t52 * t141 * t297 - grad_v_tmp * t312 + t17__ * t314 +
                       b_grad_v_tmp * t318 + t147___ * in1[0] * in1[1]) +
                      t33___ * in1[0] * in1[2]) +
        dJ_domg[0] * (
            (c_grad_v_tmp * ((-t59 * t154 * t164 + t105 * t132 * t154 / 2.0) +
                             t59 * t132 * t155 * t238 / 2.0) +
             g_grad_v_tmp * ((((t157 - t289) - t310_tmp * t164) + t104 * t132 * t154 / 2.0) +
                             t58 * t132 * t155 * t238 / 2.0) +
             t93__ * t314 + e_grad_v_tmp * t312 + f_grad_v_tmp * t318 +
             d_grad_v_tmp * (((t288 - t32 * t132 * t154 / 2.0) + t307_tmp * t164) -
                             t115___ * t238 / 2.0)) -
            t57____ * t297 * t311 - t57____ * t298 * t310 + t57____ * t303 * t307)
    ) + (
        dJ_domg[2] * ((t208 - t221) - t84_ * t141 * t154 + t86___ * in1[2] -
                      t92___ * in1[1] + t87_ * t132 * t150 * t155 / 2.0 -
                      t73_ * t132 * t149 * t155 / 2.0 + t84_ * t132 * t148 * t155 / 2.0) +
        t205_tmp_ * (((t297 - in2[1] * t312) + in2[0] * t318) + t26 * t314) + t7____ * in1[0]
    ) - t171_ * ((t9____ * in1[1] - t115___ * t305 / 2.0) + t210_tmp_ * t314)

    t57_____ = t136 * t152
    t115____ = t31 * t136 * t155
    t5______ = b_grad_v_tmp * t154
    t38__c = t61 * t297 * in1[0]
    grad_a[1] = (
        -dJ_domg[1] * ((t52 * t142 * t298 - b_grad_v_tmp * t313 + t17__ * t315 -
                        grad_v_tmp * t319 + t38__c * in1[1]) -
                        t33___ * in1[1] * in1[2]) +
        dJ_domg[2] * ((t209 + t223) + t73_ * t142 * t154 - t5______ * in1[2] +
                      t92___ * in1[0] - t87_ * t136 * t150 * t155 / 2.0 +
                      t73_ * t136 * t149 * t155 / 2.0 - t84_ * t136 * t148 * t155 / 2.0)
    ) + (
        -dJ_domg[0] * (
            (-t151 * t303 * (
                ((-t287 + t32 * t136 * t154 / 2.0) + t307_tmp * t165) + t115____ * t238 / 2.0
            ) +
             g_grad_v_tmp * (
                 (t310_tmp * t165 + t104 * t136 * t154 / 2.0) + t58 * t136 * t155 * t238 / 2.0
             ) +
             c_grad_v_tmp * ((((-t157 + t289) + t311_tmp * t165) + t105 * t136 * t154 / 2.0) +
                             t59 * t136 * t155 * t238 / 2.0) +
             t93__ * t315 - f_grad_v_tmp * t313 + e_grad_v_tmp * t319) -
            t57_____ * t297 * t311 - t57_____ * t298 * t310 + t57_____ * t303 * t307
        ) +
        t205_tmp_ * ((((-t205 + t269_tmp * t154) + in2[0] * t313) + in2[1] * t319) -
                     t26 * t315) + t7____ * in1[1]
    ) + t171_ * ((t9____ * in1[0] - t115____ * t305 / 2.0) + t210_tmp_ * t315)

    t57______ = t140 * t152
    t115_____ = t31 * t140 * t155
    grad_a[2] = (
        -dJ_domg[2] * ((t224 + t87_ * t143 * t154 + grad_v_tmp_tmp - t5______ * in1[1] +
                        t86___ * in1[0] + t87_ * t140 * t150 * t155 / 2.0 -
                        t73_ * t140 * t149 * t155 / 2.0 + t84_ * t140 * t148 * t155 / 2.0)) -
        dJ_domg[1] * ((t52 * t143 * t303 - b_grad_v_tmp * t317 + grad_v_tmp * t316 -
                       t17__ * t320 + t38__c * in1[2]) -
                       t147___ * in1[1] * in1[2])
    ) + (
        dJ_domg[0] * (
            (d_grad_v_tmp * ((t32 * t140 * t154 / 2.0 - t307_tmp * t156) + t115_____ * t238 / 2.0) +
             e_grad_v_tmp * t316 + t93__ * t320 + f_grad_v_tmp * t317 +
             g_grad_v_tmp * (((t287 + t310_tmp * t156) - t104 * t140 * t154 / 2.0) -
                             t58 * t140 * t155 * t238 / 2.0) +
             c_grad_v_tmp * (((t288 + t311_tmp * t156) - t105 * t140 * t154 / 2.0) -
                             t59 * t140 * t155 * t238 / 2.0)) +
            t57______ * t297 * t311 + t57______ * t298 * t310 - t57______ * t303 * t307
        ) -
        t205_tmp_ * (((t303 - in2[0] * t317) + in2[1] * t316) - t26 * t320) + t7____ * in1[2]
    ) - t171_ * (t115_____ * t305 / 2.0 + t210_tmp_ * t320)

    # -----------------------------------------------------------
    #  阶段 4: grad_j[0], grad_j[1], grad_j[2]
    # -----------------------------------------------------------
    grad_j = np.zeros(3)
    grad_j[0] = -dJ_domg[0] * (
        d_grad_v_tmp * (t162 - t31 * t122 * t154) -
        g_grad_v_tmp * (t163 - t58 * t122 * t154) +
        t59 * t122 * t155 * t297
    )
    grad_j[1] = dJ_domg[0] * (
        d_grad_v_tmp * (t161 - t31 * t131 * t154) +
        c_grad_v_tmp * (t163 + t59 * t131 * t154) +
        t58 * t131 * t155 * t298
    )
    grad_j[2] = -dJ_domg[0] * (
        g_grad_v_tmp * (t161 - t58 * t137_ * t154) +
        c_grad_v_tmp * (t162 - t59 * t137_ * t154) +
        t31 * t137_ * t155 * t303
    )

    return grad_v, grad_a, grad_j


# =========================================================================
# 4. 验证主函数
# =========================================================================
def verify_random_tests(num_tests=20, tolerance_rel=1e-4, tolerance_abs=1e-6):
    """
    随机生成多组 (v, a, j)，对比有限差分与解析梯度
    """
    np.random.seed(42)
    grav = 9.8

    print("=" * 80)
    print("固定翼平坦度反向梯度 正确性验证")
    print("有限差分 (Finite Difference) vs 解析梯度 (MATLAB Coder CSE)")
    print("=" * 80)

    max_errors = {'grad_v': 0.0, 'grad_a': 0.0, 'grad_j': 0.0}
    all_passed = True

    for test_idx in range(num_tests):
        # 生成随机测试点
        v = np.random.uniform(2.0, 20.0, 3)
        a = np.random.uniform(-8.0, 8.0, 3)
        jer = np.random.uniform(-4.0, 4.0, 3)

        # 随机上游梯度
        dJ_dnt = np.random.uniform(-2.0, 2.0)
        dJ_dnn = np.random.uniform(-2.0, 2.0)
        dJ_dmu = np.random.uniform(-2.0, 2.0)
        dJ_domg = np.random.uniform(-2.0, 2.0, 3)

        # 数值梯度 (有限差分)
        grad_v_num, grad_a_num, grad_j_num = numerical_backward(v, a, jer, dJ_dnt, dJ_dnn, dJ_dmu, dJ_domg, grav)

        # 解析梯度 (MATLAB Coder)
        grad_v_ana, grad_a_ana, grad_j_ana = analytical_backward(v, a, jer, dJ_dnt, dJ_dnn, dJ_dmu, dJ_domg, grav)

        # 误差分析
        def compute_error(num, ana, name):
            abs_err = np.max(np.abs(num - ana))
            norm_num = np.maximum(np.linalg.norm(num), 1e-10)
            rel_err = np.linalg.norm(num - ana) / norm_num
            return abs_err, rel_err

        for grad_num, grad_ana, name in [
            (grad_v_num, grad_v_ana, 'grad_v'),
            (grad_a_num, grad_a_ana, 'grad_a'),
            (grad_j_num, grad_j_ana, 'grad_j')
        ]:
            abs_err, rel_err = compute_error(grad_num, grad_ana, name)
            max_errors[name] = max(max_errors[name], abs_err)

            # 检查是否通过
            passed = (abs_err < tolerance_abs) or (rel_err < tolerance_rel)
            if not passed:
                all_passed = False
                print(f"\n❌ 测试 #{test_idx}: {name} 未通过!")
                print(f"   输入: v={v}, a={a}, j={jer}")
                print(f"   上游: dJ_dnt={dJ_dnt:.4f}, dJ_dnn={dJ_dnn:.4f}, dJ_dmu={dJ_dmu:.4f}, dJ_domg={dJ_domg}")
                print(f"   数值梯度: {grad_num}")
                print(f"   解析梯度: {grad_ana}")
                print(f"   绝对误差: {abs_err:.2e}, 相对误差: {rel_err:.2e}")

        if (test_idx + 1) % 5 == 0:
            print(f"  ✓ 已完成 {test_idx + 1}/{num_tests} 组测试...")

    print(f"\n{'=' * 80}")
    print(f"验证结果汇总 (共 {num_tests} 组随机测试)")
    print(f"{'=' * 80}")
    print(f"  最大绝对误差:")
    print(f"    grad_v: {max_errors['grad_v']:.2e}")
    print(f"    grad_a: {max_errors['grad_a']:.2e}")
    print(f"    grad_j: {max_errors['grad_j']:.2e}")

    if all_passed:
        print(f"\n✅ 所有测试通过! (容差: 相对 < {tolerance_rel}, 绝对 < {tolerance_abs})")
    else:
        print(f"\n❌ 部分测试未通过!")

    return all_passed


# =========================================================================
# 5. 定向测试 (检查各个分量)
# =========================================================================
def verify_individual_channels():
    """
    对每个控制量单独激活梯度，验证雅可比矩阵的正确性
    """
    np.random.seed(123)
    v = np.array([8.0, 3.0, -2.0])
    a = np.array([1.5, -2.0, 0.5])
    jer = np.array([0.5, 0.3, -0.2])
    grav = 9.8

    print(f"\n{'=' * 80}")
    print("逐分量验证 (y 轴: ∂(output)/∂(input), 每个通道单独激活)")
    print(f"{'=' * 80}")

    channels = [
        ("dJ_dnt", 1.0, 0.0, 0.0, np.zeros(3)),
        ("dJ_dnn", 0.0, 1.0, 0.0, np.zeros(3)),
        ("dJ_dmu", 0.0, 0.0, 1.0, np.zeros(3)),
        ("dJ_domg_x", 0.0, 0.0, 0.0, np.array([1.0, 0.0, 0.0])),
        ("dJ_domg_y", 0.0, 0.0, 0.0, np.array([0.0, 1.0, 0.0])),
        ("dJ_domg_z", 0.0, 0.0, 0.0, np.array([0.0, 0.0, 1.0])),
    ]

    all_ok = True
    for name, dnt, dnn, dmu, domg in channels:
        grad_v_num, grad_a_num, grad_j_num = numerical_backward(v, a, jer, dnt, dnn, dmu, domg, grav)
        grad_v_ana, grad_a_ana, grad_j_ana = analytical_backward(v, a, jer, dnt, dnn, dmu, domg, grav)

        err_v = np.max(np.abs(grad_v_num - grad_v_ana))
        err_a = np.max(np.abs(grad_a_num - grad_a_ana))
        err_j = np.max(np.abs(grad_j_num - grad_j_ana))
        max_err = max(err_v, err_a, err_j)

        status = "✅" if max_err < 1e-6 else "❌"
        if max_err >= 1e-6:
            all_ok = False
        print(f"  {status} {name:12s} → 最大误差: {max_err:.2e}  "
              f"(∂v: {err_v:.2e}, ∂a: {err_a:.2e}, ∂j: {err_j:.2e})")

    if all_ok:
        print(f"\n✅ 所有通道验证通过!")
    else:
        print(f"\n❌ 部分通道验证失败!")

    return all_ok


if __name__ == '__main__':
    passed1 = verify_individual_channels()
    passed2 = verify_random_tests(num_tests=30)
    if passed1 and passed2:
        print(f"\n🎉 全部验证通过: 解析梯度与数值梯度一致，MATLAB Coder 生成的代码正确!")
    else:
        print(f"\n⚠️  存在验证失败项，需要检查代码")
