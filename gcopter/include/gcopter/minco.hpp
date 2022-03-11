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

#ifndef MINCO_HPP
#define MINCO_HPP

#include "gcopter/trajectory.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <vector>

namespace minco
{

    // The banded system class is used for solving
    // banded linear system Ax=b efficiently.
    // A is an N*N band matrix with lower band width lowerBw
    // and upper band width upperBw.
    // Banded LU factorization has O(N) time complexity.
    class BandedSystem
    {
    public:
        // The size of A, as well as the lower/upper
        // banded width p/q are needed
        inline void create(const int &n, const int &p, const int &q)
        {
            // In case of re-creating before destroying
            destroy();
            N = n;
            lowerBw = p;
            upperBw = q;
            int actualSize = N * (lowerBw + upperBw + 1);
            ptrData = new double[actualSize];
            std::fill_n(ptrData, actualSize, 0.0);
            return;
        }

        inline void destroy()
        {
            if (ptrData != nullptr)
            {
                delete[] ptrData;
                ptrData = nullptr;
            }
            return;
        }

    private:
        int N;
        int lowerBw;
        int upperBw;
        // Compulsory nullptr initialization here
        double *ptrData = nullptr;

    public:
        // Reset the matrix to zero
        inline void reset(void)
        {
            std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
            return;
        }

        // The band matrix is stored as suggested in "Matrix Computation"
        inline const double &operator()(const int &i, const int &j) const
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        inline double &operator()(const int &i, const int &j)
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        // This function conducts banded LU factorization in place
        // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
        inline void factorizeLU()
        {
            int iM, jM;
            double cVl;
            for (int k = 0; k <= N - 2; k++)
            {
                iM = std::min(k + lowerBw, N - 1);
                cVl = operator()(k, k);
                for (int i = k + 1; i <= iM; i++)
                {
                    if (operator()(i, k) != 0.0)
                    {
                        operator()(i, k) /= cVl;
                    }
                }
                jM = std::min(k + upperBw, N - 1);
                for (int j = k + 1; j <= jM; j++)
                {
                    cVl = operator()(k, j);
                    if (cVl != 0.0)
                    {
                        for (int i = k + 1; i <= iM; i++)
                        {
                            if (operator()(i, k) != 0.0)
                            {
                                operator()(i, j) -= operator()(i, k) * cVl;
                            }
                        }
                    }
                }
            }
            return;
        }

        // This function solves Ax=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        template <typename EIGENMAT>
        inline void solve(EIGENMAT &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                iM = std::min(j + lowerBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                b.row(j) /= operator()(j, j);
                iM = std::max(0, j - upperBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            return;
        }

        // This function solves ATx=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        template <typename EIGENMAT>
        inline void solveAdj(EIGENMAT &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                b.row(j) /= operator()(j, j);
                iM = std::min(j + upperBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                iM = std::max(0, j - lowerBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            return;
        }
    };

    // MINCO for s=3 and non-uniform time
    class MINCO_S3NU
    {
    public:
        MINCO_S3NU() = default;
        ~MINCO_S3NU() { A.destroy(); }

    private:
        int N;
        Eigen::Matrix3d headPVA;
        Eigen::Matrix3d tailPVA;
        BandedSystem A;
        Eigen::MatrixX3d b;
        Eigen::VectorXd T1;
        Eigen::VectorXd T2;
        Eigen::VectorXd T3;
        Eigen::VectorXd T4;
        Eigen::VectorXd T5;

    public:
        inline void setConditions(const Eigen::Matrix3d &headState,
                                  const Eigen::Matrix3d &tailState,
                                  const int &pieceNum)
        {
            N = pieceNum;
            headPVA = headState;
            tailPVA = tailState;
            A.create(6 * N, 6, 6);
            b.resize(6 * N, 3);
            T1.resize(N);
            T2.resize(N);
            T3.resize(N);
            T4.resize(N);
            T5.resize(N);
            return;
        }

        inline void setParameters(const Eigen::Matrix3Xd &inPs,
                                  const Eigen::VectorXd &ts)
        {
            T1 = ts;
            T2 = T1.cwiseProduct(T1);
            T3 = T2.cwiseProduct(T1);
            T4 = T2.cwiseProduct(T2);
            T5 = T4.cwiseProduct(T1);

            A.reset();
            b.setZero();

            A(0, 0) = 1.0;
            A(1, 1) = 1.0;
            A(2, 2) = 2.0;
            b.row(0) = headPVA.col(0).transpose();
            b.row(1) = headPVA.col(1).transpose();
            b.row(2) = headPVA.col(2).transpose();

            for (int i = 0; i < N - 1; i++)
            {
                A(6 * i + 3, 6 * i + 3) = 6.0;
                A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
                A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
                A(6 * i + 3, 6 * i + 9) = -6.0;
                A(6 * i + 4, 6 * i + 4) = 24.0;
                A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
                A(6 * i + 4, 6 * i + 10) = -24.0;
                A(6 * i + 5, 6 * i) = 1.0;
                A(6 * i + 5, 6 * i + 1) = T1(i);
                A(6 * i + 5, 6 * i + 2) = T2(i);
                A(6 * i + 5, 6 * i + 3) = T3(i);
                A(6 * i + 5, 6 * i + 4) = T4(i);
                A(6 * i + 5, 6 * i + 5) = T5(i);
                A(6 * i + 6, 6 * i) = 1.0;
                A(6 * i + 6, 6 * i + 1) = T1(i);
                A(6 * i + 6, 6 * i + 2) = T2(i);
                A(6 * i + 6, 6 * i + 3) = T3(i);
                A(6 * i + 6, 6 * i + 4) = T4(i);
                A(6 * i + 6, 6 * i + 5) = T5(i);
                A(6 * i + 6, 6 * i + 6) = -1.0;
                A(6 * i + 7, 6 * i + 1) = 1.0;
                A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
                A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
                A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
                A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
                A(6 * i + 7, 6 * i + 7) = -1.0;
                A(6 * i + 8, 6 * i + 2) = 2.0;
                A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
                A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
                A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
                A(6 * i + 8, 6 * i + 8) = -2.0;

                b.row(6 * i + 5) = inPs.col(i).transpose();
            }

            A(6 * N - 3, 6 * N - 6) = 1.0;
            A(6 * N - 3, 6 * N - 5) = T1(N - 1);
            A(6 * N - 3, 6 * N - 4) = T2(N - 1);
            A(6 * N - 3, 6 * N - 3) = T3(N - 1);
            A(6 * N - 3, 6 * N - 2) = T4(N - 1);
            A(6 * N - 3, 6 * N - 1) = T5(N - 1);
            A(6 * N - 2, 6 * N - 5) = 1.0;
            A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
            A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
            A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
            A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
            A(6 * N - 1, 6 * N - 4) = 2;
            A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
            A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
            A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

            b.row(6 * N - 3) = tailPVA.col(0).transpose();
            b.row(6 * N - 2) = tailPVA.col(1).transpose();
            b.row(6 * N - 1) = tailPVA.col(2).transpose();

            A.factorizeLU();
            A.solve(b);

            return;
        }

        inline void getTrajectory(Trajectory &traj) const
        {
            traj.clear();
            traj.reserve(N);
            for (int i = 0; i < N; i++)
            {
                traj.emplace_back(T1(i),
                                  b.block<6, 3>(6 * i, 0)
                                      .transpose()
                                      .rowwise()
                                      .reverse());
            }
            return;
        }

        inline void getEnergy(double &energy) const
        {
            energy = 0.0;
            for (int i = 0; i < N; i++)
            {
                energy += 36.0 * b.row(6 * i + 3).squaredNorm() * T1(i) +
                          144.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T2(i) +
                          192.0 * b.row(6 * i + 4).squaredNorm() * T3(i) +
                          240.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T3(i) +
                          720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T4(i) +
                          720.0 * b.row(6 * i + 5).squaredNorm() * T5(i);
            }
            return;
        }

        inline const Eigen::MatrixX3d &getCoeffs(void) const
        {
            return b;
        }

        inline void getEnergyPartialGradByCoeffs(Eigen::MatrixX3d &gdC) const
        {
            gdC.resize(6 * N, 3);
            for (int i = 0; i < N; i++)
            {
                gdC.row(6 * i + 5) = 240.0 * b.row(6 * i + 3) * T3(i) +
                                     720.0 * b.row(6 * i + 4) * T4(i) +
                                     1440.0 * b.row(6 * i + 5) * T5(i);
                gdC.row(6 * i + 4) = 144.0 * b.row(6 * i + 3) * T2(i) +
                                     384.0 * b.row(6 * i + 4) * T3(i) +
                                     720.0 * b.row(6 * i + 5) * T4(i);
                gdC.row(6 * i + 3) = 72.0 * b.row(6 * i + 3) * T1(i) +
                                     144.0 * b.row(6 * i + 4) * T2(i) +
                                     240.0 * b.row(6 * i + 5) * T3(i);
                gdC.block<3, 3>(6 * i, 0).setZero();
            }
            return;
        }

        inline Eigen::VectorXd getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const
        {
            gdT.resize(N);
            for (int i = 0; i < N; i++)
            {
                gdT(i) = 36.0 * b.row(6 * i + 3).squaredNorm() +
                         288.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T1(i) +
                         576.0 * b.row(6 * i + 4).squaredNorm() * T2(i) +
                         720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T2(i) +
                         2880.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T3(i) +
                         3600.0 * b.row(6 * i + 5).squaredNorm() * T4(i);
            }
            return gdT;
        }

        inline void propogateGrad(const Eigen::MatrixX3d &partialGradByCoeffs,
                                  const Eigen::VectorXd &partialGradByTimes,
                                  Eigen::Matrix3Xd &gradByPoints,
                                  Eigen::VectorXd &gradByTimes)

        {
            gradByPoints.resize(3, N - 1);
            gradByTimes.resize(N);
            Eigen::MatrixX3d adjGrad = partialGradByCoeffs;
            A.solveAdj(adjGrad);

            for (int i = 0; i < N - 1; i++)
            {
                gradByPoints.col(i) = adjGrad.row(6 * i + 5).transpose();
            }

            Eigen::Matrix<double, 6, 3> B1;
            Eigen::Matrix3d B2;
            for (int i = 0; i < N - 1; i++)
            {
                // negative velocity
                B1.row(2) = -(b.row(i * 6 + 1) +
                              2.0 * T1(i) * b.row(i * 6 + 2) +
                              3.0 * T2(i) * b.row(i * 6 + 3) +
                              4.0 * T3(i) * b.row(i * 6 + 4) +
                              5.0 * T4(i) * b.row(i * 6 + 5));
                B1.row(3) = B1.row(2);

                // negative acceleration
                B1.row(4) = -(2.0 * b.row(i * 6 + 2) +
                              6.0 * T1(i) * b.row(i * 6 + 3) +
                              12.0 * T2(i) * b.row(i * 6 + 4) +
                              20.0 * T3(i) * b.row(i * 6 + 5));

                // negative jerk
                B1.row(5) = -(6.0 * b.row(i * 6 + 3) +
                              24.0 * T1(i) * b.row(i * 6 + 4) +
                              60.0 * T2(i) * b.row(i * 6 + 5));

                // negative snap
                B1.row(0) = -(24.0 * b.row(i * 6 + 4) +
                              120.0 * T1(i) * b.row(i * 6 + 5));

                // negative crackle
                B1.row(1) = -120.0 * b.row(i * 6 + 5);

                gradByTimes(i) = B1.cwiseProduct(adjGrad.block<6, 3>(6 * i + 3, 0)).sum();
            }

            // negative velocity
            B2.row(0) = -(b.row(6 * N - 5) +
                          2.0 * T1(N - 1) * b.row(6 * N - 4) +
                          3.0 * T2(N - 1) * b.row(6 * N - 3) +
                          4.0 * T3(N - 1) * b.row(6 * N - 2) +
                          5.0 * T4(N - 1) * b.row(6 * N - 1));

            // negative acceleration
            B2.row(1) = -(2.0 * b.row(6 * N - 4) +
                          6.0 * T1(N - 1) * b.row(6 * N - 3) +
                          12.0 * T2(N - 1) * b.row(6 * N - 2) +
                          20.0 * T3(N - 1) * b.row(6 * N - 1));

            // negative jerk
            B2.row(2) = -(6.0 * b.row(6 * N - 3) +
                          24.0 * T1(N - 1) * b.row(6 * N - 2) +
                          60.0 * T2(N - 1) * b.row(6 * N - 1));

            gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block<3, 3>(6 * N - 3, 0)).sum();

            gradByTimes += partialGradByTimes;
        }
    };

}

#endif
