#include <iostream>
#include <fstream>
#include <chrono>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

using namespace gtsam;

void saveValues(std::ostream &stream, const Values& values, const Marginals& marginals,
                std::vector<Symbol>& poseSymbols, std::vector<Symbol>& landmarkSymbols,
                const NonlinearFactorGraph &graph)
{
    // POSE3 xn Pose3(x, y, z, roll, pitch, yaw) Covariance(xx, xy, xz, xroll, xpitch, xyaw, yx, yy, yz, yroll, ypitch, yyaw, zx, zy, zz, zroll, zpitch, zyaw, rollx, rolly, rollz, rollroll, rollpitch, rollyaw, pitchx, pitchy, pitchz, pitchroll, pitchpitch, pitchyaw, yawx, yawy, yawz, yawroll, yawpitch, yawyaw)
    for(Symbol x : poseSymbols)
    {
        Pose3 poseEstimate = values.at<Pose3>(x);
        auto poseCovariance = marginals.marginalCovariance(x);
        stream << "POSE3 " << x << " " << poseEstimate.x() << " " << poseEstimate.y() << " " << poseEstimate.z() 
               << " " << poseEstimate.rotation().roll() << " " << poseEstimate.rotation().pitch() << " " << poseEstimate.rotation().yaw();
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                stream << " " << poseCovariance(i, j);
        stream << "\n";
    }

    // POINT3 ln Point3(x, y, z) Covariance(xx, xy, xz, yx, yy, yz, zx, zy, zz)
    for(Symbol l : landmarkSymbols)
    {
        Point3 landmarkEstimate = values.at<Point3>(l);
        auto landmarkCovariance = marginals.marginalCovariance(l);
        stream << "POINT3 " << l << " " << landmarkEstimate.x() << " " << landmarkEstimate.y() << " " << landmarkEstimate.z();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                stream << " " << landmarkCovariance(i, j);
        stream << "\n";
    }


    // BETWEENFACTOR x1 x2 x y z roll pitch yaw Covariance(xx, xy, xz, xroll, xpitch, xyaw, yx, yy, yz, yroll, ypitch, yyaw, zx, zy, zz, zroll, zpitch, zyaw, rollx, rolly, rollz, rollroll, rollpitch, rollyaw, pitchx, pitchy, pitchz, pitchroll, pitchpitch, pitchyaw, yawx, yawy, yawz, yawroll, yawpitch, yawyaw)
    // BEARINGRANGEFACTOR x l x, y, z Covariance(xx, xy, xz, yx, yy, yz, zx, zy, zz)
    for (const auto &factor : graph)
    {
        if (auto betweenFactor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor))
        {
            Symbol poseKey1 = betweenFactor->keys().at(0);
            Symbol poseKey2 = betweenFactor->keys().at(1);
            
            Pose3 measurement = betweenFactor->measured();
            Vector3 rpy = measurement.rotation().rpy();
            Matrix6 covariance = boost::dynamic_pointer_cast<noiseModel::Diagonal>(betweenFactor->noiseModel())->covariance();

            stream << "BETWEENFACTOR " << poseKey1 << " " << poseKey2 << " "
                << measurement.x() << " " << measurement.y() << " " << measurement.z() << " "
                << rpy(0) << " " << rpy(1) << " " << rpy(2);

            for (int i = 0; i < 6; i++)
                for (int j = 0; j < 6; j++)
                    stream << " " << covariance(i, j);

            stream << "\n";
        }

        if (auto brFactor = boost::dynamic_pointer_cast<BearingRangeFactor<Pose3, Point3>>(factor))
        {
            Symbol poseKey = brFactor->keys().at(0);
            Symbol landmarkKey = brFactor->keys().at(1);

            Unit3 bearing = brFactor->measured().bearing();
            double range = brFactor->measured().range();

            Vector3 bearingVec = bearing.unitVector();
            double bx = bearingVec(0);
            double by = bearingVec(1);
            double bz = bearingVec(2);

            double x = bx * range;
            double y = by * range;
            double z = bz * range;

            Matrix3 covariance = boost::dynamic_pointer_cast<noiseModel::Diagonal>(brFactor->noiseModel())->covariance();
            
            stream << "BEARINGRANGEFACTOR " << poseKey << " " << landmarkKey << " "
                << x << " " << y << " " << z;

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    stream << " " << covariance(i, j);
            stream << "\n";
        }
    }
}



int main() {
    NonlinearFactorGraph graph;
    static Symbol x0('x', 0), x1('x', 1), x2('x', 2);
    static Symbol l0('l', 0), l1('l', 1);

    Pose3 prior(Rot3(), Point3(0, 0, 0));

    auto priorNoise = noiseModel::Diagonal::Sigmas((Vector6() << 0.3, 0.3, 0.3, 0.1, 0.1, 0.1).finished());
    graph.addPrior(x0, prior, priorNoise);

    Pose3 odometry(Rot3::RzRyRx(0, 0, 0), Point3(2, 0, 1));
    auto odometryNoise = noiseModel::Diagonal::Sigmas((Vector6() << 0.2, 0.2, 0.2, 0.1, 0.1, 0.1).finished());

    graph.emplace_shared<BetweenFactor<Pose3>>(x0, x1, odometry, odometryNoise);
    graph.emplace_shared<BetweenFactor<Pose3>>(x1, x2, odometry, odometryNoise);

    auto measurementNoise = noiseModel::Diagonal::Sigmas((Vector3() << 0.1, 0.1, 0.2).finished());

    // Measurement vectors
    Point3 vec00(2, 2, 2);
    Point3 vec10(0, 2, 1);
    Point3 vec21(0, 2, 1);

    // Convert bearings and ranges
    Unit3 bearing00 = Unit3(vec00);
    Unit3 bearing10 = Unit3(vec10);
    Unit3 bearing21 = Unit3(vec21);

    double range00 = vec00.norm();
    double range10 = vec10.norm();
    double range21 = vec21.norm();

    graph.emplace_shared<BearingRangeFactor<Pose3, Point3>>(x0, l0, bearing00, range00, measurementNoise);
    graph.emplace_shared<BearingRangeFactor<Pose3, Point3>>(x1, l0, bearing10, range10, measurementNoise);
    graph.emplace_shared<BearingRangeFactor<Pose3, Point3>>(x2, l1, bearing21, range21, measurementNoise);

    Values initialEstimate;
    initialEstimate.insert(x0, Pose3(Rot3(), Point3(0.5, 0, 0.5)));
    initialEstimate.insert(x1, Pose3(Rot3(), Point3(2.3, 0.1, 1.2)));
    initialEstimate.insert(x2, Pose3(Rot3(), Point3(4.1, 0.1, 2.3)));
    initialEstimate.insert(l0, Point3(1.8, 2.1, 2.0));
    initialEstimate.insert(l1, Point3(4.1, 1.8, 3.1));

    Marginals initialMarginals(graph, initialEstimate);
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);

    auto start = std::chrono::high_resolution_clock::now();

    Values results = optimizer.optimize();
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Computation time: " << duration.count() * 1000 << " ms" << std::endl;

    Marginals marginals(graph, results);

    std::vector<Symbol> poseSymbols = {x0, x1, x2};
    std::vector<Symbol> landmarkSymbols = {l0, l1};

    {
        std::ofstream outputFile("./res/slam_3d/before.txt");
        if (outputFile.is_open()) {
            saveValues(outputFile, initialEstimate, initialMarginals, poseSymbols, landmarkSymbols, graph);
            outputFile.close();
            std::cout << "Initial configuration saved at before.txt." << std::endl;
        } else {
            std::cout << "Error opening the file." << std::endl;
        }
    }

    {
        std::ofstream outputFile("./res/slam_3d/after.txt");
        if (outputFile.is_open()) {
            saveValues(outputFile, results, marginals, poseSymbols, landmarkSymbols, graph);
            outputFile.close();
            std::cout << "Results saved as after.txt." << std::endl;
        } else {
            std::cout << "Error opening the file." << std::endl;
        }
    }

    // graph.print("Factor Graph:\n");

    return 0;
}