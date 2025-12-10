#include <iostream>
#include <fstream>
#include <chrono>
// As this is a planar SLAM example, we will use Pose2 variables (x, y, theta) to represent
// the robot positions and Point2 variables (x, y) to represent the landmark coordinates.
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use a RangeBearing factor for the range-bearing measurements to identified
// landmarks, and Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// common Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

using namespace gtsam;


void saveValues(std::ostream &stream, const Values &values, const Marginals &marginals,
                std::vector<Symbol> &poseSymbols, std::vector<Symbol> &landmarkSymbols,
                const NonlinearFactorGraph &graph)
{
    // POSE2 xn Pose2(x, y, theta) Covariance(xx, xy, xtheta, yx, yy, ytheta, thetax, thetay, thetatheta)
    for (Symbol x : poseSymbols)
    {
        Pose2 poseEstimate = values.at<Pose2>(x);
        auto poseCovariance = marginals.marginalCovariance(x);

        stream << "POSE2 " << x << " " << poseEstimate.x() << " " << poseEstimate.y() << " " << poseEstimate.theta();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                stream << " " << poseCovariance(i, j);
        stream << "\n";
    }

    // POINT2 ln Point2(x, y) Covariance(xx, xy, yx, yy)
    for (Symbol l : landmarkSymbols)
    {
        Point2 landmarkEstimate = values.at<Point2>(l);
        auto landmarkCovariance = marginals.marginalCovariance(l);

        stream << "POINT2 " << l << " " << landmarkEstimate.x() << " " << landmarkEstimate.y();
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                stream << " " << landmarkCovariance(i, j);
        stream << "\n";
    }

    // BETWEENFACTOR x1 x2 x y theta Covariance(xx, xy, xtheta, yx, yy, ytheta, thetax, thetay, thetatheta)
    // BEARINGRANGEFACTOR x l bearing range Covariance(bearing, range)
    for (const auto &factor : graph)
    {
        if (auto betweenFactor = boost::dynamic_pointer_cast<BetweenFactor<Pose2>>(factor))
        {
            Symbol poseKey1 = betweenFactor->keys().at(0);
            Symbol poseKey2 = betweenFactor->keys().at(1);
            
            Pose2 measurement = betweenFactor->measured();
            Matrix3 covariance = boost::dynamic_pointer_cast<noiseModel::Diagonal>(betweenFactor->noiseModel())->covariance();

            stream << "BETWEENFACTOR " << poseKey1 << " " << poseKey2 << " " << measurement.x() << " " << measurement.y() << " " << measurement.theta();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    stream << " " << covariance(i, j);
            stream << "\n";
        }

        if (auto brFactor = boost::dynamic_pointer_cast<BearingRangeFactor<Pose2, Point2>>(factor))
        {
            Symbol poseKey = brFactor->keys().at(0);
            Symbol landmarkKey = brFactor->keys().at(1);
            double bearing = brFactor->measured().bearing().theta();
            double range = brFactor->measured().range();
            Matrix2 covariance = boost::dynamic_pointer_cast<noiseModel::Diagonal>(brFactor->noiseModel())->covariance();

            stream << "BEARINGRANGEFACTOR " << poseKey << " " << landmarkKey << " " << bearing << " " << range;
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                    stream << " " << covariance(i, j);
            stream << "\n";
        }
    }
}

int main(int argc, char **argv)
{
    // Create a factor graph
    NonlinearFactorGraph graph;

    // Create the keys we need for this simple example
    static Symbol x0('x', 0), x1('x', 1), x2('x', 2);
    static Symbol l0('l', 0), l1('l', 1);

    // Add a prior on pose x1 at the origin. A prior factor consists of a mean and
    // a noise model (covariance matrix)
    Pose2 prior(0.0, 0.0, 0.0);                                             // prior mean is at origin
    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1)); // 30cm std on x,y, 0.1 rad on theta
    graph.addPrior(x0, prior, priorNoise);                                  // add directly to graph

    // Add two odometry factors
    Pose2 odometry(2.0, 0.0, 0.0);
    // create a measurement for both factors (the same in this case)
    auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1)); // 20cm std on x,y, 0.1 rad on theta
    graph.emplace_shared<BetweenFactor<Pose2>>(x0, x1, odometry, odometryNoise);
    graph.emplace_shared<BetweenFactor<Pose2>>(x1, x2, odometry, odometryNoise);

    // Add Range-Bearing measurements to two different landmarks
    // create a noise model for the landmark measurements
    auto measurementNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.2)); // 0.1 rad std on bearing, 20cm on range
    // create the measurement values - indices are (pose id, landmark id)
    Rot2 bearing00 = Rot2::fromDegrees(45);
    Rot2 bearing10 = Rot2::fromDegrees(90);
    Rot2 bearing21 = Rot2::fromDegrees(90);
    double range00 = std::sqrt(4.0 + 4.0), range10 = 2.0, range21 = 2.0;

    // Add Bearing-Range factors
    graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(x0, l0, bearing00, range00, measurementNoise);
    graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(x1, l0, bearing10, range10, measurementNoise);
    graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(x2, l1, bearing21, range21, measurementNoise);

    // Print
    // graph.print("Factor Graph:\n");

    // Create (deliberately inaccurate) initial estimate
    Values initialEstimate;
    initialEstimate.insert(x0, Pose2(0.5, 0.0, 0.2));
    initialEstimate.insert(x1, Pose2(2.3, 0.1, -0.2));
    initialEstimate.insert(x2, Pose2(4.1, 0.1, 0.1));
    initialEstimate.insert(l0, Point2(1.8, 2.1));
    initialEstimate.insert(l1, Point2(4.1, 1.8));

    // Print
    // initialEstimate.print("Initial Estimate:\n");

    Marginals initialMarginals(graph, initialEstimate);

    // Optimize using Levenberg-Marquardt optimization. The optimizer
    // accepts an optional set of configuration parameters, controlling
    // things like convergence criteria, the type of linear system solver
    // to use, and the amount of information displayed during optimization.
    // Here we will use the default set of parameters.  See the
    // documentation for the full set of parameters.
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);

    auto start = std::chrono::high_resolution_clock::now();

    Values results = optimizer.optimize();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Computation time: " << duration.count() * 1000 << " ms" << std::endl;

    // result.print("Final Result:\n");

    Marginals marginals(graph, results);

    std::vector<Symbol> poseSymbols = {x0, x1, x2};
    std::vector<Symbol> landmarkSymbols = {l0, l1};

    {
        std::ofstream outputFile("./res/slam_2d/before.txt");
        if (outputFile.is_open())
        {
            saveValues(outputFile, initialEstimate, initialMarginals, poseSymbols, landmarkSymbols, graph);
            outputFile.close();
            std::cout << "Initial configuration saved at before.txt." << std::endl;
        }
        else
        {
            std::cout << "Error opening the file." << std::endl;
        }
    }

    {
        std::ofstream outputFile("./res/slam_2d/after.txt");
        if (outputFile.is_open())
        {
            saveValues(outputFile, results, marginals, poseSymbols, landmarkSymbols, graph);
            outputFile.close();
            std::cout << "Result saved at after.txt." << std::endl;
        }
        else
        {
            std::cout << "Error opening the file." << std::endl;
        }
    }

    // graph.print("Factor Graph:\n");

    return 0;
}
