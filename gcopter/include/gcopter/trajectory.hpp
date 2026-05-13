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

#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include "gcopter/root_finder.hpp"

#include <Eigen/Eigen>

#include <iostream>
#include <cmath>
#include <cfloat>
#include <vector>

/**
 * @brief 单段多项式轨迹类 (Polynomial Trajectory Piece)
 * 
 * 该类实现了一段 D 阶多项式空间曲线轨迹，存储了相对持续时间与多项式系数矩阵。
 * 主要提供了正向的状态插值（位置、速度、加速度、加加速度）以及导函数求根验证等辅助功能。
 */
template <int D>
class Piece
{
public:
    typedef Eigen::Matrix<double, 3, D + 1> CoefficientMat;
    typedef Eigen::Matrix<double, 3, D> VelCoefficientMat;
    typedef Eigen::Matrix<double, 3, D - 1> AccCoefficientMat;

private:
    double duration;
    CoefficientMat coeffMat;

public:
    Piece() = default;

    /**
     * @brief 初始化轨迹片段
     *
     * @param dur 轨迹段持续时间 (Duration)
     * @param cMat 轨迹段多项式系数矩阵 (3 x (D+1) 的大小)
     */
    Piece(double dur, const CoefficientMat &cMat)
        : duration(dur), coeffMat(cMat) {}

    inline int getDim() const
    {
        return 3;
    }

    inline int getDegree() const
    {
        return D;
    }

    inline double getDuration() const
    {
        return duration;
    }

    inline const CoefficientMat &getCoeffMat() const
    {
        return coeffMat;
    }

    /**
     * @brief 获取对应相对时刻的位置 (Position)
     *
     * @param t 该段内的相对时间 (0 <= t <= duration)
     * @return Eigen::Vector3d 返回该时刻对应的大地系三维位置向量
     */
    inline Eigen::Vector3d getPos(const double &t) const
    {
        Eigen::Vector3d pos(0.0, 0.0, 0.0);
        double tn = 1.0;
        for (int i = D; i >= 0; i--)
        {
            pos += tn * coeffMat.col(i);
            tn *= t;
        }
        return pos;
    }

    /**
     * @brief 获取对应相对时刻的速度 (Velocity)
     *
     * @param t 该段内的相对时间 (0 <= t <= duration)
     * @return Eigen::Vector3d 返回该时刻对应的大地系三维速度向量
     */
    inline Eigen::Vector3d getVel(const double &t) const
    {
        Eigen::Vector3d vel(0.0, 0.0, 0.0);
        double tn = 1.0;
        int n = 1;
        for (int i = D - 1; i >= 0; i--)
        {
            vel += n * tn * coeffMat.col(i);
            tn *= t;
            n++;
        }
        return vel;
    }

    /**
     * @brief 获取对应相对时刻的加速度 (Acceleration)
     *
     * @param t 该段内的相对时间 (0 <= t <= duration)
     * @return Eigen::Vector3d 返回该时刻对应的大地系三维加速度向量
     */
    inline Eigen::Vector3d getAcc(const double &t) const
    {
        Eigen::Vector3d acc(0.0, 0.0, 0.0);
        double tn = 1.0;
        int m = 1;
        int n = 2;
        for (int i = D - 2; i >= 0; i--)
        {
            acc += m * n * tn * coeffMat.col(i);
            tn *= t;
            m++;
            n++;
        }
        return acc;
    }

    /**
     * @brief 获取对应相对时刻的加加速度 (Jerk, 加速度的导数)
     *
     * @param t 该段内的相对时间 (0 <= t <= duration)
     * @return Eigen::Vector3d 返回该时刻对应的大地系三维加加速度向量
     */
    inline Eigen::Vector3d getJer(const double &t) const
    {
        Eigen::Vector3d jer(0.0, 0.0, 0.0);
        double tn = 1.0;
        int l = 1;
        int m = 2;
        int n = 3;
        for (int i = D - 3; i >= 0; i--)
        {
            jer += l * m * n * tn * coeffMat.col(i);
            tn *= t;
            l++;
            m++;
            n++;
        }
        return jer;
    }

    inline CoefficientMat normalizePosCoeffMat() const
    {
        CoefficientMat nPosCoeffsMat;
        double t = 1.0;
        for (int i = D; i >= 0; i--)
        {
            nPosCoeffsMat.col(i) = coeffMat.col(i) * t;
            t *= duration;
        }
        return nPosCoeffsMat;
    }

    inline VelCoefficientMat normalizeVelCoeffMat() const
    {
        VelCoefficientMat nVelCoeffMat;
        int n = 1;
        double t = duration;
        for (int i = D - 1; i >= 0; i--)
        {
            nVelCoeffMat.col(i) = n * coeffMat.col(i) * t;
            t *= duration;
            n++;
        }
        return nVelCoeffMat;
    }

    inline AccCoefficientMat normalizeAccCoeffMat() const
    {
        AccCoefficientMat nAccCoeffMat;
        int n = 2;
        int m = 1;
        double t = duration * duration;
        for (int i = D - 2; i >= 0; i--)
        {
            nAccCoeffMat.col(i) = n * m * coeffMat.col(i) * t;
            n++;
            m++;
            t *= duration;
        }
        return nAccCoeffMat;
    }

    /**
     * @brief 计算该段轨迹上的最大速度模长 (Maximum Velocity Magnitude)
     *
     * ======= 极值求根计算思路 =======
     * 先求得归一化常系数，构造出三维分量平方和构成的一个多项式（表示模长的平方），
     * 利用 RootFinder 进行一阶求导并求解多项式根（驻点），比对各根处的速度获得整段轨迹的绝对极值。
     *
     * @return double 全局最大速度速率
     */
    inline double getMaxVelRate() const
    {
        VelCoefficientMat nVelCoeffMat = normalizeVelCoeffMat();
        Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                RootFinder::polySqr(nVelCoeffMat.row(2));
        int N = coeff.size();
        int n = N - 1;
        for (int i = 0; i < N; i++)
        {
            coeff(i) *= n;
            n--;
        }
        if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
        {
            return getVel(0.0).norm();
        }
        else
        {
            double l = -0.0625;
            double r = 1.0625;
            while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
            {
                l = 0.5 * l;
            }
            while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
            {
                r = 0.5 * (r + 1.0);
            }
            std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                      FLT_EPSILON / duration);
            candidates.insert(0.0);
            candidates.insert(1.0);
            double maxVelRateSqr = -INFINITY;
            double tempNormSqr;
            for (std::set<double>::const_iterator it = candidates.begin();
                 it != candidates.end();
                 it++)
            {
                if (0.0 <= *it && 1.0 >= *it)
                {
                    tempNormSqr = getVel((*it) * duration).squaredNorm();
                    maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
                }
            }
            return sqrt(maxVelRateSqr);
        }
    }

    /**
     * @brief 计算该段轨迹上的最大加速度模长 (Maximum Acceleration Magnitude)
     *
     * @return double 最大加速度的模长大小
     */
    inline double getMaxAccRate() const
    {
        AccCoefficientMat nAccCoeffMat = normalizeAccCoeffMat();
        Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                RootFinder::polySqr(nAccCoeffMat.row(2));
        int N = coeff.size();
        int n = N - 1;
        for (int i = 0; i < N; i++)
        {
            coeff(i) *= n;
            n--;
        }
        if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
        {
            return getAcc(0.0).norm();
        }
        else
        {
            double l = -0.0625;
            double r = 1.0625;
            while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
            {
                l = 0.5 * l;
            }
            while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
            {
                r = 0.5 * (r + 1.0);
            }
            std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                      FLT_EPSILON / duration);
            candidates.insert(0.0);
            candidates.insert(1.0);
            double maxAccRateSqr = -INFINITY;
            double tempNormSqr;
            for (std::set<double>::const_iterator it = candidates.begin();
                 it != candidates.end();
                 it++)
            {
                if (0.0 <= *it && 1.0 >= *it)
                {
                    tempNormSqr = getAcc((*it) * duration).squaredNorm();
                    maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
                }
            }
            return sqrt(maxAccRateSqr);
        }
    }

    /**
     * @brief 动力学校验 - 检验整段轨迹速度是否无越界
     *
     * ======= 极值校验与多项式求根算法相结合 =======
     * 对模长平方多项式偏移阈值（减去极大值的平方），检验该衍生多项式在0~1空间内是否有根。
     * 此机制严格替代了有漏洞的密集布点采样检查法，能够确证“最大速度永不超限”。
     *
     * @param maxVelRate 给定的最大速度模长界限约束
     * @return true 轨迹始终安全，满足要求
     * @return false 存在超速不可行的区间
     */
    inline bool checkMaxVelRate(const double &maxVelRate) const
    {
        double sqrMaxVelRate = maxVelRate * maxVelRate;
        if (getVel(0.0).squaredNorm() >= sqrMaxVelRate ||
            getVel(duration).squaredNorm() >= sqrMaxVelRate)
        {
            return false;
        }
        else
        {
            VelCoefficientMat nVelCoeffMat = normalizeVelCoeffMat();
            Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(2));
            double t2 = duration * duration;
            coeff.tail<1>()(0) -= sqrMaxVelRate * t2;
            return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
        }
    }

    /**
     * @brief 动力学校验 - 检验整段轨迹加速度是否无越界
     *
     * @param maxAccRate 给定的最大加速度模长边界
     * @return true 轨迹始终安全，满足计算得到的推力等需求
     * @return false 存在加速度不可行的区间
     */
    inline bool checkMaxAccRate(const double &maxAccRate) const
    {
        double sqrMaxAccRate = maxAccRate * maxAccRate;
        if (getAcc(0.0).squaredNorm() >= sqrMaxAccRate ||
            getAcc(duration).squaredNorm() >= sqrMaxAccRate)
        {
            return false;
        }
        else
        {
            AccCoefficientMat nAccCoeffMat = normalizeAccCoeffMat();
            Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(2));
            double t2 = duration * duration;
            double t4 = t2 * t2;
            coeff.tail<1>()(0) -= sqrMaxAccRate * t4;
            return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
        }
    }
};

/**
 * @brief 完整多项式分段轨迹类 (Piecewise Polynomial Trajectory)
 * 
 * 该类将多个 Piece<D> 片段沿时间轴首尾相连成平滑连续的一整段全局轨迹。
 * 对外包裹了对内的状态访问调用，提供整体性边界极值审查和按照绝对时间查找、插值的便利接口。
 */
template <int D>
class Trajectory
{
private:
    typedef std::vector<Piece<D>> Pieces;
    Pieces pieces;

public:
    Trajectory() = default;

    /**
     * @brief 基于分段组合信息构造完整轨迹
     *
     * @param durs 各分段片段对应的持续时间序列
     * @param cMats 各分段对应的一组多项式三维系数矩阵集合
     */
    Trajectory(const std::vector<double> &durs,
               const std::vector<typename Piece<D>::CoefficientMat> &cMats)
    {
        int N = std::min(durs.size(), cMats.size());
        pieces.reserve(N);
        for (int i = 0; i < N; i++)
        {
            pieces.emplace_back(durs[i], cMats[i]);
        }
    }

    inline int getPieceNum() const
    {
        return pieces.size();
    }

    inline Eigen::VectorXd getDurations() const
    {
        int N = getPieceNum();
        Eigen::VectorXd durations(N);
        for (int i = 0; i < N; i++)
        {
            durations(i) = pieces[i].getDuration();
        }
        return durations;
    }

    inline double getTotalDuration() const
    {
        int N = getPieceNum();
        double totalDuration = 0.0;
        for (int i = 0; i < N; i++)
        {
            totalDuration += pieces[i].getDuration();
        }
        return totalDuration;
    }

    inline Eigen::Matrix3Xd getPositions() const
    {
        int N = getPieceNum();
        Eigen::Matrix3Xd positions(3, N + 1);
        for (int i = 0; i < N; i++)
        {
            positions.col(i) = pieces[i].getCoeffMat().col(D);
        }
        positions.col(N) = pieces[N - 1].getPos(pieces[N - 1].getDuration());
        return positions;
    }

    inline const Piece<D> &operator[](int i) const
    {
        return pieces[i];
    }

    inline Piece<D> &operator[](int i)
    {
        return pieces[i];
    }

    inline void clear(void)
    {
        pieces.clear();
        return;
    }

    inline typename Pieces::const_iterator begin() const
    {
        return pieces.begin();
    }

    inline typename Pieces::const_iterator end() const
    {
        return pieces.end();
    }

    inline typename Pieces::iterator begin()
    {
        return pieces.begin();
    }

    inline typename Pieces::iterator end()
    {
        return pieces.end();
    }

    inline void reserve(const int &n)
    {
        pieces.reserve(n);
        return;
    }

    inline void emplace_back(const Piece<D> &piece)
    {
        pieces.emplace_back(piece);
        return;
    }

    inline void emplace_back(const double &dur,
                             const typename Piece<D>::CoefficientMat &cMat)
    {
        pieces.emplace_back(dur, cMat);
        return;
    }

    inline void append(const Trajectory<D> &traj)
    {
        pieces.insert(pieces.end(), traj.begin(), traj.end());
        return;
    }

    /**
     * @brief 全局时间到时间段号的快速映射 (Find active piece idx and map local time)
     *
     * 逐段减去耗时来映射：将用户请求的一个“大一统”全局时间 t
     * 直接重定位，算出它落在哪一个单段（Piece）中。
     *
     * @param t [输入/输出] 初始为全局运行时刻(0 ~ totalDuration)；调用完该指针被修改为当前所在分段内的局部运行时间
     * @return int 该时刻当前所在的单段编号 (Range: 0 至 getPieceNum()-1)
     */
    inline int locatePieceIdx(double &t) const
    {
        int N = getPieceNum();
        int idx;
        double dur;
        for (idx = 0;
             idx < N &&
             t > (dur = pieces[idx].getDuration());
             idx++)
        {
            t -= dur;
        }
        if (idx == N)
        {
            idx--;
            t += pieces[idx].getDuration();
        }
        return idx;
    }

    /**
     * @brief 便利函数：获取全局时刻下的位置向量 (Position)
     *
     * @param t 全局时间
     * @return Eigen::Vector3d 组合计算后的大地系空间位置坐标
     */
    inline Eigen::Vector3d getPos(double t) const
    {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getPos(t);
    }

    inline Eigen::Vector3d getVel(double t) const
    {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getVel(t);
    }

    inline Eigen::Vector3d getAcc(double t) const
    {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getAcc(t);
    }

    inline Eigen::Vector3d getJer(double t) const
    {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getJer(t);
    }

    inline Eigen::Vector3d getJuncPos(int juncIdx) const
    {
        if (juncIdx != getPieceNum())
        {
            return pieces[juncIdx].getCoeffMat().col(D);
        }
        else
        {
            return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());
        }
    }

    inline Eigen::Vector3d getJuncVel(int juncIdx) const
    {
        if (juncIdx != getPieceNum())
        {
            return pieces[juncIdx].getCoeffMat().col(D - 1);
        }
        else
        {
            return pieces[juncIdx - 1].getVel(pieces[juncIdx - 1].getDuration());
        }
    }

    inline Eigen::Vector3d getJuncAcc(int juncIdx) const
    {
        if (juncIdx != getPieceNum())
        {
            return pieces[juncIdx].getCoeffMat().col(D - 2) * 2.0;
        }
        else
        {
            return pieces[juncIdx - 1].getAcc(pieces[juncIdx - 1].getDuration());
        }
    }

    /**
     * @brief 计算该段轨迹上的最大速度模长 (Maximum Velocity Magnitude)
     *
     * ======= 极值求根计算思路 =======
     * 先求得归一化常系数，构造出三维分量平方和构成的一个多项式（表示模长的平方），
     * 利用 RootFinder 进行一阶求导并求解多项式根（驻点），比对各根处的速度获得整段轨迹的绝对极值。
     *
     * @return double 全局最大速度速率
     */
    inline double getMaxVelRate() const
    {
        int N = getPieceNum();
        double maxVelRate = -INFINITY;
        double tempNorm;
        for (int i = 0; i < N; i++)
        {
            tempNorm = pieces[i].getMaxVelRate();
            maxVelRate = maxVelRate < tempNorm ? tempNorm : maxVelRate;
        }
        return maxVelRate;
    }

    /**
     * @brief 计算该段轨迹上的最大加速度模长 (Maximum Acceleration Magnitude)
     *
     * @return double 最大加速度的模长大小
     */
    inline double getMaxAccRate() const
    {
        int N = getPieceNum();
        double maxAccRate = -INFINITY;
        double tempNorm;
        for (int i = 0; i < N; i++)
        {
            tempNorm = pieces[i].getMaxAccRate();
            maxAccRate = maxAccRate < tempNorm ? tempNorm : maxAccRate;
        }
        return maxAccRate;
    }

    /**
     * @brief 动力学校验 - 检验整段轨迹速度是否无越界
     *
     * ======= 极值校验与多项式求根算法相结合 =======
     * 对模长平方多项式偏移阈值（减去极大值的平方），检验该衍生多项式在0~1空间内是否有根。
     * 此机制严格替代了有漏洞的密集布点采样检查法，能够确证“最大速度永不超限”。
     *
     * @param maxVelRate 给定的最大速度模长界限约束
     * @return true 轨迹始终安全，满足要求
     * @return false 存在超速不可行的区间
     */
    inline bool checkMaxVelRate(const double &maxVelRate) const
    {
        int N = getPieceNum();
        bool feasible = true;
        for (int i = 0; i < N && feasible; i++)
        {
            feasible = feasible && pieces[i].checkMaxVelRate(maxVelRate);
        }
        return feasible;
    }

    /**
     * @brief 动力学校验 - 检验整段轨迹加速度是否无越界
     *
     * @param maxAccRate 给定的最大加速度模长边界
     * @return true 轨迹始终安全，满足计算得到的推力等需求
     * @return false 存在加速度不可行的区间
     */
    inline bool checkMaxAccRate(const double &maxAccRate) const
    {
        int N = getPieceNum();
        bool feasible = true;
        for (int i = 0; i < N && feasible; i++)
        {
            feasible = feasible && pieces[i].checkMaxAccRate(maxAccRate);
        }
        return feasible;
    }
};

#endif