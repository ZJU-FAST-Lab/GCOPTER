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

#ifndef GCOPTER_HPP
#define GCOPTER_HPP

#include "gcopter/minco.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/lbfgs.hpp"
#include "gcopter/sep_tables.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>

namespace gcopter
{

    /**
     * @brief GCOPTER 多面体安全飞行走廊轨迹优化器 (Polytope SFC)
     *
     * 该类实现了基于凸多面体安全走廊的 MINCO 时空联合轨迹优化。
     * 采用基于梯度的无约束非线性规划 (L-BFGS) 框架，在保证平滑性的同时
     * 严格满足避障、速度、角速度、过载等物理约束。
     *
     * 完整优化流水线 (详见 costFunctional)：
     *
     *   x = [τ, ξ]                     ← L-BFGS 无约束优化变量
     *       │
     *   ┌───┴───── bijection ─────────┐
     *   │  forwardT:  τ → T > 0       │
     *   │  forwardP:  ξ → q ∈ 走廊     │
     *   └───┬─────────────────────────┘
     *       │
     *   MINCO (q, T) → C              ← 最优多项式系数 (闭式 QP)
     *       │
     *   ┌───┴── 配点采样 ──────────────────────────────────┐
     *   │  for 每段 × 每配点:                              │
     *   │    C + 时间基底 → pos, vel, acc, jer, snap        │
     *   │    flatmap.forward → n_t, n_n, mu, ω             │
     *   │    ┌── 约束惩罚 ──┐                               │
     *   │    │ violaPos → gradPos   (走廊穿墙)              │
     *   │    │ violaVel → gradVel   (限速)                  │
     *   │    │ violaOmg → gradOmg   (角速度)                │
     *   │    │ violaNt  → dJ_dnt    (切向过载)              │
     *   │    │ violaNn  → dJ_dnn    (法向过载)              │
     *   │    │ violaMu  → dJ_dmu    (滚转角)                │
     *   │    └──────────────────────┘                       │
     *   │    flatmap.backward → ∂J/∂p, ∂J/∂v, ∂J/∂a, ∂J/∂j │
     *   │    gradC += β_matrix × totalGrad   (多项式积分)   │
     *   │    gradT += ...                                  │
     *   └──────────────────────────────────────────────────┘
     *       │
     *   MINCO.propogateGrad: (∂J/∂C, ∂J/∂T) → (∂J/∂q, ∂J/∂T)
     *       │
     *   ┌───┴──── bijection 梯度 ──────┐
     *   │ backwardGradT: ∂J/∂T → ∂J/∂τ │
     *   │ backwardGradP: ∂J/∂q → ∂J/∂ξ │
     *   └───┬──────────────────────────┘
     *       │
     *   L-BFGS 更新 x = [τ, ξ] → 返回循环
     *
     * @note  固定翼适配：magnitudeBounds 含 6 个元素
     *        [vMax, omgMax, ntMax, ntMin, nnMax, muMax]
     *        penaltyWeights 含 6 个权重
     *        [pos, vel, omg, nt, nn, mu]
     */
    class GCOPTER_PolytopeSFC
    {
    public:
        typedef Eigen::Matrix3Xd PolyhedronV;
        typedef Eigen::MatrixX4d PolyhedronH;
        typedef std::vector<PolyhedronV> PolyhedraV;
        typedef std::vector<PolyhedronH> PolyhedraH;

    private:
        // ---- 核心模块 ----
        minco::MINCO_S3NU minco;           // MINCO 轨迹参数化 (q,T) ↔ C
        flatness::FlatnessMap flatmap;     // 微分平坦映射 (forward + backward)

        double rho;                        // 时间最小化权重
        Eigen::Matrix3d headPVA;           // 起点 PVA 边界条件
        Eigen::Matrix3d tailPVA;           // 终点 PVA 边界条件

        // ---- 安全走廊 ----
        PolyhedraV vPolytopes;             // 多面体 V-rep (用于 bijection)
        PolyhedraH hPolytopes;             // 多面体 H-rep (用于约束惩罚)
        Eigen::Matrix3Xd shortPath;        // RRT* 最短路径 (初始航点)

        // ---- 段分配 ----
        Eigen::VectorXi pieceIdx;          // 每段走廊包含的轨迹段数
        Eigen::VectorXi vPolyIdx;          // 每个航点对应的 V-多面体索引
        Eigen::VectorXi hPolyIdx;          // 每段轨迹对应的 H-多面体索引

        int polyN;                         // 走廊多面体数
        int pieceN;                        // 轨迹总段数

        int spatialDim;                    // ξ 的维度 (空间自由度)
        int temporalDim;                   // τ 的维度 (时间自由度)

        // ---- 优化参数 ----
        double smoothEps;                  // 平滑 L1 惩罚的平滑因子
        int integralRes;                   // 每段轨迹的采样点数
        Eigen::VectorXd magnitudeBd;       // [0:vMax, 1:omgXMax, 2:nnMax, 3:nnMin, 4:muMax, 5:ntMargin, 6:vMin]
        Eigen::VectorXd penaltyWt;         // [pos, vel, omg, nt, nn, mu] 各约束权重
        Eigen::VectorXd physicalPm;        // [0:mass, 1:grav, 2:dh, 3:dv, 4:cp, 5:veps, 6:kCL, 7:softMinK, 8:vMin?]
        double allocSpeed;                 // 初始时间分配速度

        // ---- L-BFGS 配置与中间缓存 ----
        lbfgs::lbfgs_parameter_t lbfgs_params;

        Eigen::Matrix3Xd points;           // 中间航点 q
        Eigen::VectorXd times;             // 段时间 T
        // 阶段 B 输出: MINCO 伴随求解后的梯度 (对航点 q 和时间 T)
        Eigen::Matrix3Xd gradByPoints;           // ∂J/∂q
        Eigen::VectorXd gradByTimes;             // ∂J/∂T
        // 阶段 A 输出: 采样点积分得到的梯度 (对系数 C 和时间 T, 伴随前)
        Eigen::MatrixX3d partialGradByCoeffs;    // ∂J/∂C
        Eigen::VectorXd partialGradByTimes;      // ∂J/∂T

    private:
        // ================================================================
        //  时间变量的 bijection 映射: T ∈ (0, +∞) ↔ τ ∈ (-∞, +∞)
        //
        //  目的: L-BFGS 要求无约束优化变量, 但段时间 T 必须 > 0。
        //  通过光滑双射 τ = f(T) 将 T ∈ (0,∞) 映射到 τ ∈ (-∞,∞),
        //  优化器可以自由调整 τ 而无需担心 T ≤ 0。
        //
        //  forwardT:  τ → T   (正向: 优化变量 → 物理时间)
        //  backwardT: T → τ   (反向: 物理时间 → 优化变量, 初始化用)
        //  backwardGradT: (τ, ∂J/∂T) → ∂J/∂τ  (梯度传递: 链式法则)
        // ================================================================

        static inline void forwardT(const Eigen::VectorXd &tau,
                                    Eigen::VectorXd &T)
        {
            const int sizeTau = tau.size();
            T.resize(sizeTau);
            for (int i = 0; i < sizeTau; i++)
            {
                T(i) = tau(i) > 0.0
                           ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                           : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
            }
            return;
        }

        template <typename EIGENVEC>
        static inline void backwardT(const Eigen::VectorXd &T,
                                     EIGENVEC &tau)
        {
            const int sizeT = T.size();
            tau.resize(sizeT);
            for (int i = 0; i < sizeT; i++)
            {
                tau(i) = T(i) > 1.0
                             ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                             : (1.0 - sqrt(2.0 / T(i) - 1.0));
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void backwardGradT(const Eigen::VectorXd &tau,
                                         const Eigen::VectorXd &gradT,
                                         EIGENVEC &gradTau)
        {
            const int sizeTau = tau.size();
            gradTau.resize(sizeTau);
            double denSqrt;
            for (int i = 0; i < sizeTau; i++)
            {
                if (tau(i) > 0)
                {
                    gradTau(i) = gradT(i) * (tau(i) + 1.0);
                }
                else
                {
                    denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                    gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
                }
            }

            return;
        }

        // ================================================================
        //  航点变量的 bijection 映射: q ∈ 凸多面体 ↔ ξ ∈ (-∞, +∞)
        //
        //  目的: L-BFGS 要求无约束变量, 但航点 q 必须落在安全走廊多面体内。
        //  利用多面体 V-rep 的凸组合表示 q = V₀ + Σ r_i²·(V_i - V₀),
        //  r 归一化后作为优化变量的一部分, 自动保证 q 在多面体内。
        //
        //  forwardP:  ξ → q   (正向: 优化变量 → 航点坐标)
        //  backwardP: q → ξ   (反向: 航点坐标 → 优化变量, 初始化用)
        //  backwardGradP: (ξ, ∂J/∂q) → ∂J/∂ξ  (梯度传递: 链式法则)
        //
        //  costTinyNLS: 内层 NLS 的代价函数 (用于 backwardP 初始化时
        //               将给定航点投影到多面体内)
        //  normRetrictionLayer: ξ 的范数硬约束惩罚 (‖q_seg‖² ≤ 1)
        // ================================================================

        static inline void forwardP(const Eigen::VectorXd &xi,
                                    const Eigen::VectorXi &vIdx,
                                    const PolyhedraV &vPolys,
                                    Eigen::Matrix3Xd &P)
        {
            const int sizeP = vIdx.size();
            P.resize(3, sizeP);
            Eigen::VectorXd q;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                q = xi.segment(j, k).normalized().head(k - 1);
                P.col(i) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) +
                           vPolys[l].col(0);
            }
            return;
        }

        static inline double costTinyNLS(void *ptr,
                                         const Eigen::VectorXd &xi,
                                         Eigen::VectorXd &gradXi)
        {
            const int n = xi.size();
            const Eigen::Matrix3Xd &ovPoly = *(Eigen::Matrix3Xd *)ptr;

            const double sqrNormXi = xi.squaredNorm();
            const double invNormXi = 1.0 / sqrt(sqrNormXi);
            const Eigen::VectorXd unitXi = xi * invNormXi;
            const Eigen::VectorXd r = unitXi.head(n - 1);
            const Eigen::Vector3d delta = ovPoly.rightCols(n - 1) * r.cwiseProduct(r) +
                                          ovPoly.col(1) - ovPoly.col(0);

            double cost = delta.squaredNorm();
            gradXi.head(n - 1) = (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
                                 r.array() * 2.0;
            gradXi(n - 1) = 0.0;
            gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

            const double sqrNormViolation = sqrNormXi - 1.0;
            if (sqrNormViolation > 0.0)
            {
                double c = sqrNormViolation * sqrNormViolation;
                const double dc = 3.0 * c;
                c *= sqrNormViolation;
                cost += c;
                gradXi += dc * 2.0 * xi;
            }

            return cost;
        }

        template <typename EIGENVEC>
        static inline void backwardP(const Eigen::Matrix3Xd &P,
                                     const Eigen::VectorXi &vIdx,
                                     const PolyhedraV &vPolys,
                                     EIGENVEC &xi)
        {
            const int sizeP = P.cols();

            double minSqrD;
            lbfgs::lbfgs_parameter_t tiny_nls_params;
            tiny_nls_params.past = 0;
            tiny_nls_params.delta = 1.0e-5;
            tiny_nls_params.g_epsilon = FLT_EPSILON;
            tiny_nls_params.max_iterations = 128;

            Eigen::Matrix3Xd ovPoly;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();

                ovPoly.resize(3, k + 1);
                ovPoly.col(0) = P.col(i);
                ovPoly.rightCols(k) = vPolys[l];
                Eigen::VectorXd x(k);
                x.setConstant(sqrt(1.0 / k));
                lbfgs::lbfgs_optimize(x,
                                      minSqrD,
                                      &GCOPTER_PolytopeSFC::costTinyNLS,
                                      nullptr,
                                      nullptr,
                                      &ovPoly,
                                      tiny_nls_params);

                xi.segment(j, k) = x;
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void backwardGradP(const Eigen::VectorXd &xi,
                                         const Eigen::VectorXi &vIdx,
                                         const PolyhedraV &vPolys,
                                         const Eigen::Matrix3Xd &gradP,
                                         EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double normInv;
            Eigen::VectorXd q, gradQ, unitQ;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                q = xi.segment(j, k);
                normInv = 1.0 / q.norm();
                unitQ = q * normInv;
                gradQ.resize(k);
                gradQ.head(k - 1) = (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                gradXi.segment(j, k) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void normRetrictionLayer(const Eigen::VectorXd &xi,
                                               const Eigen::VectorXi &vIdx,
                                               const PolyhedraV &vPolys,
                                               double &cost,
                                               EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double sqrNormQ, sqrNormViolation, c, dc;
            Eigen::VectorXd q;
            for (int i = 0, j = 0, k; i < sizeP; i++, j += k)
            {
                k = vPolys[vIdx(i)].cols();

                q = xi.segment(j, k);
                sqrNormQ = q.squaredNorm();
                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradXi.segment(j, k) += dc * 2.0 * q;
                }
            }

            return;
        }

        // ================================================================
        //  softMin: 平滑最小值函数
        //  softMin(a,b,k) = -1/k · ln(e^{-ka} + e^{-kb}), k>0 控制锐度
        //  dSoftMinDa: ∂(softMin)/∂a
        //  用途: V-N 图边界平滑过渡 (CLmax 限制 ↔ 结构限制)
        // ================================================================
        static inline double softMin(double a, double b, double k)
        {
            if (k * (b - a) > 50.0) return a;  // a << b, 数值安全
            if (k * (a - b) > 50.0) return b;  // b << a
            return -std::log(std::exp(-k * a) + std::exp(-k * b)) / k;
        }

        static inline double dSoftMinDa(double a, double b, double k)
        {
            if (k * (b - a) > 50.0) return 1.0;  // a dominates
            if (k * (a - b) > 50.0) return 0.0;  // b dominates
            double emka = std::exp(-k * a);
            double emkb = std::exp(-k * b);
            return emka / (emka + emkb);
        }

        // ================================================================
        //  interpSEP: 双线性插值 (O(1) 解析梯度)
        //  输入: V, n_n, 表引用
        //  输出: SEP 值 + 解析梯度 ∂SEP/∂V, ∂SEP/∂n_n
        //  边界处理: 超出表范围用最近邻 + 零梯度
        // ================================================================
        static inline double interpSEP(double V, double n_n,
                                        const double table[50][7],
                                        double &dSEPdV, double &dSEPdNn)
        {
            // ---- V 维度定位 ----
            int i0, i1;
            double tv;
            if (V <= V_grid[0])       { i0 = i1 = 0; tv = 0.0; dSEPdV = 0.0; }
            else if (V >= V_grid[49]) { i0 = i1 = 49; tv = 0.0; dSEPdV = 0.0; }
            else {
                // 线性搜索 (50 点很小, 无需二分)
                i0 = 0; while (i0 < 49 && V_grid[i0+1] < V) i0++;
                i1 = i0 + 1;
                double dV = V_grid[i1] - V_grid[i0];
                tv = (V - V_grid[i0]) / dV;
            }

            // ---- n_n 维度定位 ----
            int j0, j1;
            double tn;
            if (n_n <= Nn_grid[0])       { j0 = j1 = 0; tn = 0.0; dSEPdNn = 0.0; }
            else if (n_n >= Nn_grid[6])  { j0 = j1 = 6; tn = 0.0; dSEPdNn = 0.0; }
            else {
                j0 = 0; while (j0 < 6 && Nn_grid[j0+1] < n_n) j0++;
                j1 = j0 + 1;
                double dNn = Nn_grid[j1] - Nn_grid[j0];
                tn = (n_n - Nn_grid[j0]) / dNn;
            }

            // ---- 双线性插值 ----
            double f00 = table[i0][j0], f10 = table[i1][j0];
            double f01 = table[i0][j1], f11 = table[i1][j1];
            double v0 = f00 + tv * (f10 - f00);
            double v1 = f01 + tv * (f11 - f01);
            double f = v0 + tn * (v1 - v0);

            // ---- 解析梯度 (边界外为零) ----
            if (i0 < i1) {
                double dV = V_grid[i1] - V_grid[i0];
                dSEPdV = ((1.0 - tn) * (f10 - f00) + tn * (f11 - f01)) / dV;
            }
            if (j0 < j1) {
                double dNn = Nn_grid[j1] - Nn_grid[j0];
                dSEPdNn = ((1.0 - tv) * (f01 - f00) + tv * (f11 - f10)) / dNn;
            }
            return f;
        }

        // ================================================================
        //  getNtBounds: 从 SEP 表查询切向过载上下界
        //  ntMax = SEP_full(V,n_n)/V - margin (正=加速余量)
        //  ntMin = SEP_min(V,n_n)/V  - margin (最低油门)
        //  梯度: ∂nt/∂V = (dSEPdV·V - SEP)/V², ∂nt/∂n_n = dSEPdNn/V
        // ================================================================
        static inline void getNtBounds(double V, double n_n, double margin,
                                        double &ntMax, double &dNtMaxdV, double &dNtMaxdNn,
                                        double &ntMin, double &dNtMindV, double &dNtMindNn)
        {
            double sepF, dSdV, dSdN, sepM;
            // 满油门
            sepF = interpSEP(V, n_n, SEP_full, dSdV, dSdN);
            double invV = 1.0 / V;
            ntMax = sepF * invV - margin;
            dNtMaxdV = (dSdV * V - sepF) * invV * invV;
            dNtMaxdNn = dSdN * invV;
            // 最低油门
            sepM = interpSEP(V, n_n, SEP_min, dSdV, dSdN);
            ntMin = sepM * invV - margin;
            dNtMindV = (dSdV * V - sepM) * invV * invV;
            dNtMindNn = dSdN * invV;
        }

        // ================================================================
        //  getNnMax: V-N 图法向过载上界
        //  nnMax(V) = softMin(kCL·V², structMax, softMinK)
        //  dNnMaxdV 为解析梯度
        // ================================================================
        static inline double getNnMax(double V, double kCL, double structMax,
                                       double softMinK, double &dNnMaxdV)
        {
            double a = kCL * V * V;
            double b = structMax;
            double k = softMinK;
            double val = softMin(a, b, k);
            double da = dSoftMinDa(a, b, k);
            dNnMaxdV = da * 2.0 * kCL * V;
            return val;
        }

        // ================================================================
        //  smoothedL1: 平滑的 L1 惩罚函数 (Huber-like)
        //
        //  若 x ≤ 0:    未越界, f=0, df=0, 返回 false
        //  若 0 < x < μ: 过渡区, 三次多项式平滑, f ~ x³/μ², df ~ x²/μ
        //  若 x ≥ μ:    线性区, f = x - 0.5μ, df = 1.0
        //
        //  返回 true 表示产生了非零代价 (越界或过渡区)
        // ================================================================
        static inline bool smoothedL1(const double &x,  // 越界量 violation (= 实际值 - 边界值, 正=超限)
                                      const double &mu, // 平滑因子, 控制过渡区宽度, 越小越接近硬约束
                                      double &f,        // [输出] 惩罚值, 累加到 cost
                                      double &df)       // [输出] ∂(penalty)/∂(violation), 下一步乘以 ∂violation/∂(物理量) 后路由到各 grad
                                                        // “物理量”为与越界量相关的约束变量，如n_t的越界量与n_t、V、n_n相关
                                                        // 例如对于速度约束 violation = |v|² - vMax², 则 ∂violation/∂v = 2v, 最终 gradVel += df * 2v
        {
            // ---- x ≤ 0: 未超限, 无惩罚 ----
            if (x < 0.0)
            {
                return false;
            }
            // ---- x ≥ μ: 线性区, f = x - 0.5μ, df = 1.0 (标准 L1) ----
            else if (x > mu)
            {
                f = x - 0.5 * mu;
                df = 1.0;
                return true;
            }
            // ---- 0 < x < μ: 三次样条过渡区, 保证 f, df 在 x=0 和 x=μ 处 C¹ 连续 ----
            else
            {
                const double xdmu = x / mu;
                const double sqrxdmu = xdmu * xdmu;
                const double mumxd2 = mu - 0.5 * x;
                f = mumxd2 * sqrxdmu * xdmu;                                  // 惩罚值, 量级 ~ x³/μ²
                df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);          // ∂J/∂violation, 量级 ~ x²/μ²
                return true;
            }
        }

        // ================================================================
        //  attachPenaltyFunctional: 约束惩罚核心
        //
        //  在每条轨迹段上按 integralResolution 个配点采样,
        //  对每个采样点执行完整的 正向→惩罚→反向 流程:
        //
        //  ① 多项式求值: C → pos, vel, acc, jer (时间基底 β0~β4)
        //  ② forward(): (vel,acc,jer) → n_t, n_n, mu, ω
        //  ③ 约束惩罚: 检查 6 类约束, 越界则产生梯度
        //     - 走廊位置:   a·p + b > 0  → gradPos
        //     - 速度上限:   |v|² > vMax² → gradVel
        //     - 角速度上限:  |ω|² > ωMax² → gradOmg
        //     - 切向过载:   区间约束 → dJ_dnt
        //     - 法向过载:   n_n² > nnMax² → dJ_dnn
        //     - 滚转角:     μ² > μMax² → dJ_dmu
        //  ④ backward(): 控制/状态梯度 → 轨迹梯度 (∂J/∂p, ∂J/∂v, ∂J/∂a, ∂J/∂j)
        //  ⑤ 多项式积分:  totalGrad × β_matrix → gradC, gradT
        //
        //  @note magnitudeBounds = [vMax, omgXMax, nnMax, nnMin, muMax, ntMargin, vMin]
        //        penaltyWeights   = [pos, vel, omg, nt, nn, mu]
        //        physicalPm       = [mass, grav, dh, dv, cp, veps, kCL, softMinK]
        // ================================================================
        static inline void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                                   const Eigen::MatrixX3d &coeffs,
                                                   const Eigen::VectorXi &hIdx,
                                                   const PolyhedraH &hPolys,
                                                   const double &smoothFactor,
                                                   const int &integralResolution,
                                                   const Eigen::VectorXd &magnitudeBounds,
                                                   const Eigen::VectorXd &penaltyWeights,
                                                   const Eigen::VectorXd &physicalPm,
                                                   flatness::FlatnessMap &flatMap,
                                                   double &cost,
                                                   Eigen::VectorXd &gradT,
                                                   Eigen::MatrixX3d &gradC)
        {
            // ---- 约束上限拆箱 ----
            const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);
            const double omgXMax = magnitudeBounds(1);           // |ω_x| ≤ omgXMax
            const double omgXSqrMax = omgXMax * omgXMax;
            const double nnStructMax = magnitudeBounds(2);       // V-N 图结构上限 (正)
            const double nnStructMin = magnitudeBounds(3);       // V-N 图结构下限 (负)
            const double muMax = magnitudeBounds(4);
            const double ntMargin = magnitudeBounds(5);          // SEP 安全裕度
            const double velSqrMin = magnitudeBounds(6) * magnitudeBounds(6);  // vMin²

            // ---- 惩罚权重拆箱 ----
            const double weightPos = penaltyWeights(0);
            const double weightVel = penaltyWeights(1);
            const double weightOmg = penaltyWeights(2);
            const double weightNt = penaltyWeights(3);
            const double weightNn = penaltyWeights(4);
            const double weightMu = penaltyWeights(5);
            const double weightMuBnd = penaltyWeights(6);  // 边界 μ→0 权重
            const double weightOmgBnd = penaltyWeights(7); // 边界 ω→0 权重

            // ---- 气动参数拆箱 ----
            const double kCL = physicalPm(6);     // = ρ·S·CLmax/(2·m·g)
            const double softMinK = physicalPm(7); // V-N 软过渡锐度

            // 单个采样点的中间变量
            Eigen::Vector3d pos, vel, acc, jer, sna;         // ① 多项式求值: 位置/速度/加速度/加加速度/snap
            Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer; // ④ backward 输出: ∂J/∂(p,v,a,j)
            double n_t, n_n, mu;                             // ② forward 输出: 切向过载/法向过载/滚转角
            Eigen::Vector4d quat;                            // ② forward 输出: 姿态四元数 (当前未使用)
            Eigen::Vector3d omg;                             // ② forward 输出: 气流系角速度
            double dJ_dnt, dJ_dnn, dJ_dmu;                   // ③d~f 累积 / ④ backward 输入: 控制量上游梯度
            Eigen::Vector3d gradPos, gradVel, gradOmg;       // ③a~c 累积 / ④ backward 输入: 直接约束梯度

            // SEP / V-N 动态边界变量
            double V, invV;                                  // 空速标量, 1/V
            Eigen::Vector3d vel_dir;                         // 速度单位方向 v/|v|
            double nnMax, dNnMaxdV, nnMin, dNnMindV;         // V-N 动态边界 + 梯度
            double ntMax, dNtMaxdV, dNtMaxdNn;               // SEP nt 上界 + 梯度
            double ntMin, dNtMindV, dNtMindNn;               // SEP nt 下界 + 梯度
            double violaNtUpper, violaNtLower;               // SEP nt 越界量
            double violaNnUpper, violaNnLower;               // V-N 越界量

            double step, alpha;                              // 采样步长, 时间积分权重 (α = j/N)
            double s1, s2, s3, s4, s5;                       // 时间幂次 t¹~t⁵
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4; // 时间基底: [1,t,t²,t³,t⁴,t⁵] 的 0~4 阶导数的系数向量
            Eigen::Vector3d outerNormal;                     // ③a: 多面体面的外法向量
            int K, L;                                        // ③a: 面的个数, 多面体索引
            double violaPos, violaVel, violaOmg, violaMu; // ③: 各约束的越界量 (正=超限)
            double violaPosPenaD, violaVelPenaD, violaOmgPenaD, violaNtPenaD, violaNnPenaD, violaMuPenaD; // ③: 惩罚函数导数 df/dx
            double violaPosPena, violaVelPena, violaOmgPena, violaNtPena, violaNnPena, violaMuPena;       // ③: 惩罚函数值 f(x)
            double node, pena;                               // 辛普森积分节点权重 (0.5/1.0), 单点惩罚累计值

            // ---- 采样积分 ----
            const int pieceNum = T.size();
            const double integralFrac = 1.0 / integralResolution;
            for (int i = 0; i < pieceNum; i++)
            {
                const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0);
                step = T(i) * integralFrac;
                for (int j = 0; j <= integralResolution; j++)
                {
                    // ---- ① 多项式求值 ----
                    s1 = j * step;
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
                    beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
                    beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
                    beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
                    beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
                    pos = c.transpose() * beta0;
                    vel = c.transpose() * beta1;
                    acc = c.transpose() * beta2;
                    jer = c.transpose() * beta3;
                    sna = c.transpose() * beta4;

                    // ---- ② forward: (v,a,j) → 控制量 & 角速度 ----
                    flatMap.forward(vel, acc, jer, quat, omg, &n_t, &n_n, &mu);

                    // ---- ③ 约束越界量计算 (violation = 实际值 - 动态边界) ----
                    violaVel = vel.squaredNorm() - velSqrMax;     // 超上限
                    violaOmg = omg(0) * omg(0) - omgXSqrMax;     // |ω_x| 超限
                    violaMu = mu * mu - muMax * muMax;
                    // 最小空速: vMin² - |v|² > 0 表示低于下限
                    double violaVelMin = velSqrMin - vel.squaredNorm();

                    // 动态边界: V-N 图 + SEP 表
                    V = vel.norm();
                    invV = 1.0 / (V + 1e-6);
                    vel_dir = vel * invV;

                    nnMax = getNnMax(V, kCL, nnStructMax, softMinK, dNnMaxdV);
                    nnMin = -getNnMax(V, kCL, -nnStructMin, softMinK, dNnMindV);

                    getNtBounds(V, n_n, ntMargin,
                                ntMax, dNtMaxdV, dNtMaxdNn,
                                ntMin, dNtMindV, dNtMindNn);

                    // 安全 clamp: 防止上下界倒挂
                    if (ntMax < ntMin + 1e-6) ntMax = ntMin + 1e-6;

                    violaNtUpper = n_t - ntMax;
                    violaNtLower = ntMin - n_t;
                    violaNnUpper = n_n - nnMax;
                    violaNnLower = nnMin - n_n;

                    dJ_dnt = 0.0;
                    dJ_dnn = 0.0;
                    dJ_dmu = 0.0;
                    gradPos.setZero(), gradVel.setZero(), gradOmg.setZero();
                    pena = 0.0;

                    // ---- ③a 位置约束: 半空间 a·p + b ≤ 0 ----
                    L = hIdx(i);
                    K = hPolys[L].rows();
                    for (int k = 0; k < K; k++)
                    {
                        outerNormal = hPolys[L].block<1, 3>(k, 0);
                        violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
                        if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))
                        {
                            gradPos += weightPos * violaPosPenaD * outerNormal;
                            pena += weightPos * violaPosPena;
                        }
                    }

                    // ---- ③b 速度约束: vMin² ≤ |v|² ≤ vMax² ----
                    // 最高空速
                    if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD))
                    {
                        gradVel += weightVel * violaVelPenaD * 2.0 * vel;
                        pena += weightVel * violaVelPena;
                    }
                    // 最低空速 (防失速)
                    if (smoothedL1(violaVelMin, smoothFactor, violaVelPena, violaVelPenaD))
                    {
                        gradVel += weightVel * violaVelPenaD * (-2.0) * vel;
                        pena += weightVel * violaVelPena;
                    }

                    // ---- ③c 角速度约束: |ω_x|² ≤ ωXMax² (仅滚转轴) ----
                    if (smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD))
                    {
                        gradOmg(0) += weightOmg * violaOmgPenaD * 2.0 * omg(0);
                        pena += weightOmg * violaOmgPena;
                    }

                    // ---- ③d 切向过载约束: SEP 动态边界 ----
                    // violeNtUpper = n_t - ntMax(V,n_n), ntMax = SEP_full/V - margin
                    // 梯度路由:
                    //   ∂viola/∂n_t = 1          → dJ_dnt
                    //   ∂viola/∂n_n = -∂ntMax/∂n_n → dJ_dnn (耦合进 CSE)
                    //   ∂viola/∂V   = -∂ntMax/∂V   → gradVel (直接)
                    if (smoothedL1(violaNtUpper, smoothFactor, violaNtPena, violaNtPenaD))
                    {
                        dJ_dnt += weightNt * violaNtPenaD;
                        dJ_dnn += weightNt * violaNtPenaD * (-dNtMaxdNn);
                        gradVel += weightNt * violaNtPenaD * (-dNtMaxdV) * vel_dir;
                        pena += weightNt * violaNtPena;
                    }
                    if (smoothedL1(violaNtLower, smoothFactor, violaNtPena, violaNtPenaD))
                    {
                        dJ_dnt += weightNt * violaNtPenaD * (-1.0);
                        dJ_dnn += weightNt * violaNtPenaD * dNtMindNn;
                        gradVel += weightNt * violaNtPenaD * dNtMindV * vel_dir;
                        pena += weightNt * violaNtPena;
                    }

                    // ---- ③e 法向过载约束: V-N 图动态边界 ----
                    // nnMax(V) = softMin(kCL·V², nnStructMax)
                    // 梯度路由:
                    //   ∂viola/∂n_n = 1          → dJ_dnn (CSE 链式)
                    //   ∂viola/∂V   = -∂nnMax/∂V  → gradVel (直接)
                    if (smoothedL1(violaNnUpper, smoothFactor, violaNnPena, violaNnPenaD))
                    {
                        dJ_dnn += weightNn * violaNnPenaD;
                        gradVel += weightNn * violaNnPenaD * (-dNnMaxdV) * vel_dir;
                        pena += weightNn * violaNnPena;
                    }
                    if (smoothedL1(violaNnLower, smoothFactor, violaNnPena, violaNnPenaD))
                    {
                        dJ_dnn += weightNn * violaNnPenaD * (-1.0);
                        gradVel += weightNn * violaNnPenaD * dNnMindV * vel_dir;
                        pena += weightNn * violaNnPena;
                    }

                    // ---- ③f 滚转角约束: μ² ≤ μMax² ----
                    if (smoothedL1(violaMu, smoothFactor, violaMuPena, violaMuPenaD))
                    {
                        dJ_dmu += weightMu * violaMuPenaD * 2.0 * mu;
                        pena += weightMu * violaMuPena;
                    }

                    // ---- ③g 起止点姿态约束: μ→0, ω→0 @ t=0 & t=T ----
                    if ((i == 0 && j == 0) || (i == pieceNum - 1 && j == integralResolution))
                    {
                        // μ→0 惩罚
                        double violaMuBnd = mu * mu;
                        if (smoothedL1(violaMuBnd, smoothFactor, violaMuPena, violaMuPenaD))
                        {
                            dJ_dmu += weightMuBnd * violaMuPenaD * 2.0 * mu;
                            pena += weightMuBnd * violaMuPena;
                        }
                        // ω→0 惩罚 (全三轴)
                        double violaOmgBnd = omg.squaredNorm();
                        if (smoothedL1(violaOmgBnd, smoothFactor, violaOmgPena, violaOmgPenaD))
                        {
                            gradOmg += weightOmgBnd * violaOmgPenaD * 2.0 * omg;
                            pena += weightOmgBnd * violaOmgPena;
                        }
                    }

                    // ---- ④ backward: 控制/状态梯度 → 轨迹梯度 ----
                    // gradPos, gradVel = 直接约束梯度 (透传/累加)
                    // dJ_dnt, dJ_dnn, dJ_dmu, gradOmg = 控制/状态梯度 (链式法则)
                    flatMap.backward(vel, acc, jer,
                                     dJ_dnt, dJ_dnn, dJ_dmu, gradOmg,
                                     gradPos, gradVel,
                                     totalGradPos, totalGradVel, totalGradAcc, totalGradJer);

                    // ---- ⑤ 多项式积分: totalGrad → gradC, gradT ----
                    // ∂J/∂c_k = Σ ∂J/∂p · ∂p/∂c_k + ∂J/∂v · ∂v/∂c_k + ...
                    // 其中 ∂p/∂c_k = β0[k], ∂v/∂c_k = β1[k], ...
                    node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
                    alpha = j * integralFrac;
                    gradC.block<6, 3>(i * 6, 0) += (beta0 * totalGradPos.transpose() +
                                                    beta1 * totalGradVel.transpose() +
                                                    beta2 * totalGradAcc.transpose() +
                                                    beta3 * totalGradJer.transpose()) *
                                                   node * step;
                    gradT(i) += (totalGradPos.dot(vel) +
                                 totalGradVel.dot(acc) +
                                 totalGradAcc.dot(jer) +
                                 totalGradJer.dot(sna)) *
                                    alpha * node * step +
                                node * integralFrac * pena;
                    cost += node * step * pena;
                }
            }

            return;
        }

        // ================================================================
        //  costFunctional: L-BFGS 回调函数
        //
        //  目标: 给定无约束变量 x = [τ, ξ], 返回代价 J 和梯度 ∂J/∂x。
        //
        //  完整流水线:
        //    ① bijection: τ → T,  ξ → q
        //    ② MINCO: (q,T) → C, 计算平滑能量代价 + ∂J/∂C
        //    ③ attachPenaltyFunctional: 约束惩罚 + ∂J/∂C, ∂J/∂T
        //    ④ MINCO propogateGrad: (∂J/∂C, ∂J/∂T) → (∂J/∂q, ∂J/∂T)
        //    ⑤ 加时间权重: J += ρ·ΣT
        //    ⑥ bijection 梯度: ∂J/∂T → ∂J/∂τ,  ∂J/∂q → ∂J/∂ξ
        //    ⑦ normRetrictionLayer: ξ 范数惩罚
        //
        //  J = 平滑能量 (Min Jerk) + 约束惩罚 + ρ·总时间
        // ================================================================
        static inline double costFunctional(void *ptr,
                                            const Eigen::VectorXd &x,
                                            Eigen::VectorXd &g)
        {
            GCOPTER_PolytopeSFC &obj = *(GCOPTER_PolytopeSFC *)ptr;
            const int dimTau = obj.temporalDim;
            const int dimXi = obj.spatialDim;
            const double weightT = obj.rho;
            Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
            Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
            Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
            Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);

            // ---- ① bijection: 无约束变量 → 物理变量 ----
            forwardT(tau, obj.times);
            forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);

            // ---- ② MINCO 正向 + 平滑能量 ----
            double cost;
            obj.minco.setParameters(obj.points, obj.times);
            obj.minco.getEnergy(cost);
            obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
            obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

            // ---- ③ 约束惩罚 + 反向传播 ----
            attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(),
                                    obj.hPolyIdx, obj.hPolytopes,
                                    obj.smoothEps, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt, obj.physicalPm, obj.flatmap,
                                    cost, obj.partialGradByTimes, obj.partialGradByCoeffs);

            // ---- ④ MINCO 反向: (∂J/∂C, ∂J/∂T) → (∂J/∂q, ∂J/∂T) ----
            obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                    obj.gradByPoints, obj.gradByTimes);

            // ---- ⑤ 时间最小化代价 ----
            cost += weightT * obj.times.sum();
            obj.gradByTimes.array() += weightT;

            // ---- ⑥ bijection 梯度: 物理空间 → 无约束空间 ----
            backwardGradT(tau, obj.gradByTimes, gradTau);
            backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);

            // ---- ⑦ 范数约束层 ----
            normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);

            return cost;
        }

        static inline double costDistance(void *ptr,
                                          const Eigen::VectorXd &xi,
                                          Eigen::VectorXd &gradXi)
        {
            void **dataPtrs = (void **)ptr;
            const double &dEps = *((const double *)(dataPtrs[0]));
            const Eigen::Vector3d &ini = *((const Eigen::Vector3d *)(dataPtrs[1]));
            const Eigen::Vector3d &fin = *((const Eigen::Vector3d *)(dataPtrs[2]));
            const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));

            double cost = 0.0;
            const int overlaps = vPolys.size() / 2;

            Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);
            Eigen::Vector3d a, b, d;
            Eigen::VectorXd r;
            double smoothedDistance;
            for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k)
            {
                a = i == 0 ? ini : b;
                if (i < overlaps)
                {
                    k = vPolys[2 * i + 1].cols();
                    Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                    r = q.normalized().head(k - 1);
                    b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                        vPolys[2 * i + 1].col(0);
                }
                else
                {
                    b = fin;
                }

                d = b - a;
                smoothedDistance = sqrt(d.squaredNorm() + dEps);
                cost += smoothedDistance;

                if (i < overlaps)
                {
                    gradP.col(i) += d / smoothedDistance;
                }
                if (i > 0)
                {
                    gradP.col(i - 1) -= d / smoothedDistance;
                }
            }

            Eigen::VectorXd unitQ;
            double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                Eigen::Map<Eigen::VectorXd> gradQ(gradXi.data() + j, k);
                sqrNormQ = q.squaredNorm();
                invNormQ = 1.0 / sqrt(sqrNormQ);
                unitQ = q * invNormQ;
                gradQ.head(k - 1) = (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradQ += dc * 2.0 * q;
                }
            }

            return cost;
        }

        static inline void getShortestPath(const Eigen::Vector3d &ini,
                                           const Eigen::Vector3d &fin,
                                           const PolyhedraV &vPolys,
                                           const double &smoothD,
                                           Eigen::Matrix3Xd &path)
        {
            const int overlaps = vPolys.size() / 2;
            Eigen::VectorXi vSizes(overlaps);
            for (int i = 0; i < overlaps; i++)
            {
                vSizes(i) = vPolys[2 * i + 1].cols();
            }
            Eigen::VectorXd xi(vSizes.sum());
            for (int i = 0, j = 0; i < overlaps; i++)
            {
                xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));
                j += vSizes(i);
            }

            double minDistance;
            void *dataPtrs[4];
            dataPtrs[0] = (void *)(&smoothD);
            dataPtrs[1] = (void *)(&ini);
            dataPtrs[2] = (void *)(&fin);
            dataPtrs[3] = (void *)(&vPolys);
            lbfgs::lbfgs_parameter_t shortest_path_params;
            shortest_path_params.past = 3;
            shortest_path_params.delta = 1.0e-3;
            shortest_path_params.g_epsilon = 1.0e-5;

            lbfgs::lbfgs_optimize(xi,
                                  minDistance,
                                  &GCOPTER_PolytopeSFC::costDistance,
                                  nullptr,
                                  nullptr,
                                  dataPtrs,
                                  shortest_path_params);

            path.resize(3, overlaps + 2);
            path.leftCols<1>() = ini;
            path.rightCols<1>() = fin;
            Eigen::VectorXd r;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                r = q.normalized().head(k - 1);
                path.col(i + 1) = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                                  vPolys[2 * i + 1].col(0);
            }

            return;
        }

        static inline bool processCorridor(const PolyhedraH &hPs,
                                           PolyhedraV &vPs)
        {
            const int sizeCorridor = hPs.size() - 1;

            vPs.clear();
            vPs.reserve(2 * sizeCorridor + 1);

            int nv;
            PolyhedronH curIH;
            PolyhedronV curIV, curIOB;
            for (int i = 0; i < sizeCorridor; i++)
            {
                if (!geo_utils::enumerateVs(hPs[i], curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);

                curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 4);
                curIH.topRows(hPs[i].rows()) = hPs[i];
                curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];
                if (!geo_utils::enumerateVs(curIH, curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);
            }

            if (!geo_utils::enumerateVs(hPs.back(), curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB.col(0) = curIV.col(0);
            curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);

            return true;
        }

        static inline void setInitial(const Eigen::Matrix3Xd &path,
                                      const double &speed,
                                      const Eigen::VectorXi &intervalNs,
                                      Eigen::Matrix3Xd &innerPoints,
                                      Eigen::VectorXd &timeAlloc)
        {
            const int sizeM = intervalNs.size();
            const int sizeN = intervalNs.sum();
            innerPoints.resize(3, sizeN - 1);
            timeAlloc.resize(sizeN);

            Eigen::Vector3d a, b, c;
            for (int i = 0, j = 0, k = 0, l; i < sizeM; i++)
            {
                l = intervalNs(i);
                a = path.col(i);
                b = path.col(i + 1);
                c = (b - a) / l;
                timeAlloc.segment(j, l).setConstant(c.norm() / speed);
                j += l;
                for (int m = 0; m < l; m++)
                {
                    if (i > 0 || m > 0)
                    {
                        innerPoints.col(k++) = a + c * m;
                    }
                }
            }
        }

    public:
        // ================================================================
        //  setup: 初始化优化器
        //
        //  ① 归一化 H-rep 多面体 (左 3 列归一化)
        //  ② H-rep → V-rep 转换 + 相邻多面体求交 (processCorridor)
        //  ③ RRT* 最短路径 (getShortestPath)
        //  ④ 按长度分配轨迹段数
        //  ⑤ 构建 vPolyIdx / hPolyIdx / spatialDim / temporalDim
        //  ⑥ 初始化 MINCO (边界条件 + 段数)
        //  ⑦ 初始化 FlatnessMap (物理参数)
        //
        //  @return true 若初始化成功
        // ================================================================
        inline bool setup(const double &timeWeight,
                          const Eigen::Matrix3d &initialPVA,
                          const Eigen::Matrix3d &terminalPVA,
                          const PolyhedraH &safeCorridor,
                          const double &lengthPerPiece,
                          const double &smoothingFactor,
                          const int &integralResolution,
                          const Eigen::VectorXd &magnitudeBounds,
                          const Eigen::VectorXd &penaltyWeights,
                          const Eigen::VectorXd &physicalParams)
        {
            rho = timeWeight;
            headPVA = initialPVA;
            tailPVA = terminalPVA;

            hPolytopes = safeCorridor;
            for (size_t i = 0; i < hPolytopes.size(); i++)
            {
                const Eigen::ArrayXd norms =
                    hPolytopes[i].leftCols<3>().rowwise().norm();
                hPolytopes[i].array().colwise() /= norms;
            }
            if (!processCorridor(hPolytopes, vPolytopes))
            {
                return false;
            }

            polyN = hPolytopes.size();
            smoothEps = smoothingFactor;
            integralRes = integralResolution;
            magnitudeBd = magnitudeBounds;
            penaltyWt = penaltyWeights;
            physicalPm = physicalParams;
            allocSpeed = magnitudeBd(0) * 3.0;

            getShortestPath(headPVA.col(0), tailPVA.col(0),
                            vPolytopes, smoothEps, shortPath);
            const Eigen::Matrix3Xd deltas = shortPath.rightCols(polyN) - shortPath.leftCols(polyN);
            pieceIdx = (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();
            pieceIdx.array() += 1;
            pieceN = pieceIdx.sum();

            temporalDim = pieceN;
            spatialDim = 0;
            vPolyIdx.resize(pieceN - 1);
            hPolyIdx.resize(pieceN);
            for (int i = 0, j = 0, k; i < polyN; i++)
            {
                k = pieceIdx(i);
                for (int l = 0; l < k; l++, j++)
                {
                    if (l < k - 1)
                    {
                        vPolyIdx(j) = 2 * i;
                        spatialDim += vPolytopes[2 * i].cols();
                    }
                    else if (i < polyN - 1)
                    {
                        vPolyIdx(j) = 2 * i + 1;
                        spatialDim += vPolytopes[2 * i + 1].cols();
                    }
                    hPolyIdx(j) = i;
                }
            }

            // Setup for MINCO_S3NU, FlatnessMap, and L-BFGS solver
            minco.setConditions(headPVA, tailPVA, pieceN);
            flatmap.reset(physicalPm(0), physicalPm(1), physicalPm(2),
                          physicalPm(3), physicalPm(4), physicalPm(5));

            // Allocate temp variables
            points.resize(3, pieceN - 1);
            times.resize(pieceN);
            gradByPoints.resize(3, pieceN - 1);
            gradByTimes.resize(pieceN);
            partialGradByCoeffs.resize(6 * pieceN, 3);
            partialGradByTimes.resize(pieceN);

            return true;
        }

        // ================================================================
        //  optimize: 执行 L-BFGS 轨迹优化
        //
        //  ① 从最短路径初始化中间航点 + 时间分配
        //  ② backwardT/P: 物理变量 → 无约束优化变量 (τ, ξ)
        //  ③ L-BFGS 循环: 每次迭代调用 costFunctional
        //  ④ 收敛后: 恢复物理变量, MINCO 生成最终轨迹
        //
        //  @param traj       [输出] 5 阶多项式轨迹
        //  @param relCostTol 收敛判据 (相对代价下降容限)
        //  @return  最优代价值 (失败返回 INFINITY)
        // ================================================================
        inline double optimize(Trajectory<5> &traj,
                               const double &relCostTol)
        {
            Eigen::VectorXd x(temporalDim + spatialDim);
            Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
            Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

            setInitial(shortPath, allocSpeed, pieceIdx, points, times);
            backwardT(times, tau);
            backwardP(points, vPolyIdx, vPolytopes, xi);

            double minCostFunctional;
            lbfgs_params.mem_size = 256;
            lbfgs_params.past = 3;
            lbfgs_params.min_step = 1.0e-32;
            lbfgs_params.g_epsilon = 0.0;
            lbfgs_params.delta = relCostTol;

            int ret = lbfgs::lbfgs_optimize(x,
                                            minCostFunctional,
                                            &GCOPTER_PolytopeSFC::costFunctional,
                                            nullptr,
                                            nullptr,
                                            this,
                                            lbfgs_params);

            if (ret >= 0)
            {
                forwardT(tau, times);
                forwardP(xi, vPolyIdx, vPolytopes, points);
                minco.setParameters(points, times);
                minco.getTrajectory(traj);
            }
            else
            {
                traj.clear();
                minCostFunctional = INFINITY;
                std::cout << "Optimization Failed: "
                          << lbfgs::lbfgs_strerror(ret)
                          << std::endl;
            }

            return minCostFunctional;
        }
    };

}

#endif
