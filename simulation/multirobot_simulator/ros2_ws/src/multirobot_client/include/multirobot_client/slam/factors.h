#ifndef FACTORS_H
#define FACTORS_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/base/numericalDerivative.h>

using namespace gtsam;

namespace localization
{

    class OrientationPriorFactor : public NoiseModelFactor1<Pose3>
    {
    public:
        Rot3 R0_;

        OrientationPriorFactor(Key key, const Rot3 &R0,
                               const SharedNoiseModel &model)
            : NoiseModelFactor1<Pose3>(model, key), R0_(R0) {}

        Vector evaluateError(
            const Pose3 &x,
            boost::optional<Matrix &> H = boost::none) const override
        {
            if (H)
            {
                Matrix Hfull = Matrix::Zero(3, 6);
                Hfull.block<3, 3>(0, 0) = Matrix3::Identity();
                *H = Hfull;
            }

            return R0_.localCoordinates(x.rotation());
        }
    };
    
}

#endif // FACTORS_H
