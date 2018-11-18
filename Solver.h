//
// Created by lightol on 18-11-18.
//

#ifndef ORB_TEMP_SOLVER_H
#define ORB_TEMP_SOLVER_H

#include <eigen3/Eigen/Eigen>
#include <iostream>

namespace LINE_PNP
{
struct costFunctor {
    costFunctor(double _x, double _y): x(_x), y(_y) {}

    template <typename T>
    bool operator()(const T* const a, const T* const b, T* residual) const {
        residual[0] = T(y) - (a[0]*x + b[0]);

        return true;
    }

    const double x;
    const double y;
};

struct ReprojectError {
    explicit ReprojectError(const std::pair<Eigen::Vector2d, Eigen::Vector2d> &_keyline,
                            const std::pair<Eigen::Vector3d, Eigen::Vector3d> &_mapline,
                            const Eigen::Matrix3d &_K,
                            double _weight):
            keyline(_keyline), mapline(_mapline), K(_K), weight(_weight){}

    template <typename T>  // 这个T是double或者Jet
    bool operator()(const T* const pKeyFrame, T* residual) const {
        // Step0: 数据转换，把所有的数组转换为Eigen::Matrix
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> rotateVector(pKeyFrame);
        Eigen::AngleAxis<T> angleAxis(rotateVector.norm(), rotateVector.normalized());
        Eigen::Matrix<T, 3, 3> R(angleAxis);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(pKeyFrame+3);

        for (int i = 0; i < 2; ++i) {
            Eigen::Matrix<T, 3, 1> mapPoint;
            if (i == 0) {  // mapline的两个端点
                mapPoint = mapline.first.template cast<T>();
            } else {
                mapPoint = mapline.second.template cast<T>();
            }

            // step1: 将这个mapPoint变换到相机坐标系下
            Eigen::Matrix<T, 3, 1> Pc = R*mapPoint + t;

            // step2: 转换到归一化平面
            T depth = Pc[2];
            Pc /= Pc[2];

            // step3: 归一化平面转换到成像平面上
            const Eigen::Matrix<T, 3, 1> uv = K.template cast<T>() * Pc;

            // step4: 最终计算残差，重投影点P(u,v)到直线p1p2的距离
            // step4.1: 对于投影到成像平面后面的重投影点，给它极大的残差
            Eigen::Matrix<T, 3, 1> p1 = Eigen::Matrix<T, 3, 1>::Ones();
            p1.topRows(2) = keyline.first.template cast<T>();
            Eigen::Matrix<T, 3, 1> p2 = Eigen::Matrix<T, 3, 1>::Ones();
            p2.topRows(2) = keyline.second.template cast<T>();
            const Eigen::Matrix<T, 3, 1> p1P = uv - p1;
            const Eigen::Matrix<T, 3, 1> p1p2 = (p2 - p1).normalized();

            residual[i] = T(weight) * (p1P.cross(p1p2)).norm();
//            residual[i] = (p1P.cross(p1p2)).norm();
        }

        return true;
    }

    const std::pair<Eigen::Vector2d, Eigen::Vector2d> keyline;
    const std::pair<Eigen::Vector3d, Eigen::Vector3d> mapline;
    const Eigen::Matrix3d K;
    const double weight;
};
}

#endif  // ORB_TEMP_SOLVER_H
