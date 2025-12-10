#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "arg_parser.h"

using namespace gtsam;

namespace fs = std::filesystem;

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

double mod(double x, double y)
{
    return std::sqrt(x*x + y*y);
}

std::pair<double, double> evaluate_error(Values &values,
    std::vector<std::pair<Symbol, Pose2>> &poses_ground_truth, std::vector<std::pair<Symbol, Point2>> &landmarks_ground_truth,
    std::vector<Symbol> &poseSymbols, std::vector<Symbol> &landmarkSymbols)
{

    double cumulativeAbsError = 0.0;
    double cumulativeSquaredError = 0.0;

    for (const std::pair<Symbol, Pose2> &pose : poses_ground_truth) {
        Symbol x = pose.first;
        Pose2 poseGroundTruth = pose.second;
        Pose2 poseEstimate = values.at<Pose2>(x);

        double absError = mod((poseEstimate.x() - poseGroundTruth.x()), (poseEstimate.y() - poseGroundTruth.y()));
        cumulativeAbsError += absError;
        cumulativeSquaredError += absError*absError;
    }


    for (const std::pair<Symbol, Point2> &landmark : landmarks_ground_truth) {
        Symbol l = landmark.first;
        Point2 landmarkGroundTruth = landmark.second;
        Point2 landmarkEstimate = values.at<Point2>(l);

        double absError = mod((landmarkEstimate.x() - landmarkGroundTruth.x()), (landmarkEstimate.y() - landmarkGroundTruth.y()));
        cumulativeAbsError += absError;
        cumulativeSquaredError += absError*absError;
    }

    // double meanAbsError = cumulativeAbsError / poses_ground_truth.size();
    // double meanAbsError = cumulativeAbsError / landmarks_ground_truth.size();
    double meanAbsError = cumulativeAbsError / (poses_ground_truth.size() + landmarks_ground_truth.size());
    double rootMeanSquaredError = std::sqrt(cumulativeSquaredError / (poses_ground_truth.size() + landmarks_ground_truth.size()));

    return std::make_pair(meanAbsError, rootMeanSquaredError);
}


std::vector<std::pair<Symbol, Point2>> get_landmarks(const std::string &filename)
{
    std::vector<std::pair<Symbol, Point2>> landmarks;
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + filename);
    }

    std::string line;

    std::getline(file, line);
    while (std::getline(file, line)) {
        std::vector<std::string> row;
        std::stringstream lineStream(line);
        std::string cell;

        while (std::getline(lineStream, cell, ',')) {
            row.push_back(cell);
        }

        int l_id = std::stoi(row[0]);
        double x = std::stod(row[1]);
        double y = std::stod(row[2]);

        landmarks.push_back(
            std::pair<Symbol, Point2>(
                Symbol('l', l_id),
                Point2(x, y)
            )
        );
    }

    file.close();
    return landmarks;
}




std::vector<std::pair<Symbol, Pose2>> get_poses(const std::string &filename)
{
    std::vector<std::pair<Symbol, Pose2>> poses;
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + filename);
    }

    std::string line;

    std::getline(file, line);
    while (std::getline(file, line)) {
        std::vector<std::string> row;
        std::stringstream lineStream(line);
        std::string cell;

        while (std::getline(lineStream, cell, ',')) {
            row.push_back(cell);
        }

        int p_id = std::stoi(row[0]);
        double x = std::stod(row[1]);
        double y = std::stod(row[2]);
        double theta = std::stod(row[3]);

        poses.push_back(
            std::pair<Symbol, Pose2>(
                Symbol('x', p_id),
                Pose2(x, y, theta)
            )
        );
    }

    file.close();
    return poses;
}




std::vector<std::pair<std::pair<Symbol, Symbol>, std::pair<Rot2, double>>> get_observations(const std::string &filename)
{
    std::vector<std::pair<std::pair<Symbol, Symbol>, std::pair<Rot2, double>>> observations;
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + filename);
    }

    std::string line;

    std::getline(file, line);
    while (std::getline(file, line)) {
        std::vector<std::string> row;
        std::stringstream lineStream(line);
        std::string cell;

        while (std::getline(lineStream, cell, ',')) {
            row.push_back(cell);
        }

        int p_id = std::stoi(row[0]);
        int l_id = std::stoi(row[1]);
        double bearing = std::stod(row[2]);
        double range = std::stod(row[3]);

        observations.push_back(
            std::pair<std::pair<Symbol, Symbol>, std::pair<Rot2, double>>(
                std::pair<Symbol, Symbol>(Symbol('x', p_id), Symbol('l', l_id)),
                std::pair<Rot2, double>(Rot2::fromAngle(bearing), range)
            )
        );
    }

    file.close();
    return observations;
}



std::vector<std::pair<std::pair<Symbol, Symbol>, Pose2>> get_odometries(const std::string &filename)
{
    std::vector<std::pair<std::pair<Symbol, Symbol>, Pose2>> odometries;
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + filename);
    }

    std::string line;

    std::getline(file, line);
    while (std::getline(file, line)) {
        std::vector<std::string> row;
        std::stringstream lineStream(line);
        std::string cell;

        while (std::getline(lineStream, cell, ',')) {
            row.push_back(cell);
        }

        int p_id0 = std::stoi(row[0]);
        int p_id1 = std::stoi(row[1]);
        double x = std::stod(row[2]);
        double y = std::stod(row[3]);
        double theta = std::stod(row[4]);

        odometries.push_back(
            std::pair<std::pair<Symbol, Symbol>, Pose2>(
                std::pair<Symbol, Symbol>(Symbol('x', p_id0), Symbol('x', p_id1)),
                Pose2(x, y, theta)
            )
        );
    }

    file.close();
    return odometries;
}




int main(int argc, char **argv)
{
    ArgumentParser parser;
    parser.parse(argc, argv);

    int benchmark = 0;
    int exp = 0;
    std::string optimizerName = "";

    std::cout << "----------------------------------------" << std::endl;
    try {
        benchmark = parser.get<int>("benchmark", 0);
        exp = parser.get<int>("exp", 0);
        optimizerName = parser.get<std::string>("optimizer", "LevenbergMarquardtOptimizer");

        std::cout << "Running benchmark: " << benchmark << std::endl;
        std::cout << "Running experiment: " << exp << std::endl;
        std::cout << "Optimizer: " << optimizerName << std::endl;
        std::cout << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
    }



    std::vector<Symbol> poseSymbols;
    std::vector<Symbol> landmarkSymbols;

    std::vector<std::pair<Symbol, Point2>> landmarks_ground_truth;
    std::vector<std::pair<Symbol, Pose2>> poses_ground_truth;

    std::vector<std::pair<Symbol, Point2>> landmarks_estimates;
    std::vector<std::pair<Symbol, Pose2>> poses_estimates;
    
    std::vector<std::pair<std::pair<Symbol, Symbol>, std::pair<Rot2, double>>> observations;
    std::vector<std::pair<std::pair<Symbol, Symbol>, Pose2>> odometries;

  
    landmarks_ground_truth = get_landmarks("./data/ground_truth/landmarks.csv");
    poses_ground_truth = get_poses("./data/ground_truth/poses.csv");
    
    landmarks_estimates = get_landmarks("./data/benchmark_" + std::to_string(benchmark) + "/exp_" + std::to_string(exp) + "/landmarks_initial_estimates.csv");
    poses_estimates = get_poses("./data/benchmark_" + std::to_string(benchmark) + "/exp_" + std::to_string(exp) + "/poses_initial_estimates.csv");
    
    observations = get_observations("./data/benchmark_" + std::to_string(benchmark) + "/exp_" + std::to_string(exp) + "/observations.csv");
    odometries = get_odometries("./data/benchmark_" + std::to_string(benchmark) + "/exp_" + std::to_string(exp) + "/odometries.csv");


    std::cout << "Building factor graph..." << std::endl;

    NonlinearFactorGraph graph;

    // Add a prior on pose x0 at the origin
    {
        Symbol x0('x', 0);
        Pose2 prior(0.0, 0.0, -0.5);
        auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.0, 0.0, 0.0));          // Noise 5cm std on x,y and 0.05 rad std on theta
        graph.addPrior(x0, prior, priorNoise);
    }

    // Add odometries
    for (const std::pair<std::pair<Symbol, Symbol>, Pose2> &odo : odometries) {
        Symbol x0 = odo.first.first;
        Symbol x1 = odo.first.second;

        Pose2 odometry = odo.second;

        auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.005, 0.005, 0.005));      // Noise 5cm std on x,y and 0.05 rad std on theta
        graph.emplace_shared<BetweenFactor<Pose2>>(x0, x1, odometry, odometryNoise);
    }

    // Add observations
    for (const std::pair<std::pair<Symbol, Symbol>, std::pair<Rot2, double>> &obs : observations) {
        Symbol x0 = obs.first.first;
        Symbol l0 = obs.first.second;

        Rot2 bearing = obs.second.first;
        double range = obs.second.second;

        auto observationNoise = noiseModel::Diagonal::Sigmas(Vector2(0.005, 0.005));          // Noise 0.05 rad std on bearing and 5cm std on range
        graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(x0, l0, bearing, range, observationNoise);
    }

    // Print graph
    // graph.print("Factor Graph:\n");

    std::cout << "Factor graph built." << std::endl;
    std::cout << std::endl;


    // Define poses and landmarks initial estimate

    Values initialEstimate;

    for (const std::pair<Symbol, Pose2> &pose : poses_estimates) {
        Symbol x = pose.first;
        Pose2 pose_estimate = pose.second;

        initialEstimate.insert(x, pose_estimate);
        poseSymbols.push_back(x);
    }

    for (const std::pair<Symbol, Point2> &landmark : landmarks_estimates) {
        Symbol l = landmark.first;
        Point2 landmark_estimate = landmark.second;

        initialEstimate.insert(l, landmark_estimate);
        landmarkSymbols.push_back(l);
    }

    // Print inital estimate
    // initialEstimate.print("Initial Estimate:\n");


    Marginals initialMarginals(graph, initialEstimate);

    
    // Optimization

    Values finalEstimates;
    std::cout << "Optimizing factor graph..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    
    if(optimizerName == "LevenbergMarquardtOptimizer")
    {
        LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
        finalEstimates = optimizer.optimize();
    }
    else if (optimizerName == "GaussNewtonOptimizer")
    {
        GaussNewtonOptimizer optimizer(graph, initialEstimate);
        finalEstimates = optimizer.optimize();
    }
    else if (optimizerName == "NonlinearConjugateGradientOptimizer")
    {
        NonlinearConjugateGradientOptimizer optimizer(graph, initialEstimate);
        finalEstimates = optimizer.optimize();
    }
    else if (optimizerName == "ISAM2")
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        parameters.cacheLinearizedFactors = false;
        parameters.enableDetailedResults = true;
        // parameters.print();

        ISAM2 optimizer(parameters);

        ISAM2Result result = optimizer.update(graph, initialEstimate);
        // result.print();

        finalEstimates = optimizer.calculateEstimate();
    }
    else
    {
        throw std::runtime_error("Invalid optimizer: " + optimizerName);
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    double computation_time = duration.count() * 1000;
    std::cout << "Computation time: " << computation_time << " ms" << std::endl;
    std::cout << "Factor graph optimized." << std::endl;
    std::cout << std::endl;

    // Print final estimates
    // finalEstimates.print("Final Estimates:\n");

    Marginals marginals(graph, finalEstimates);


    auto errorBefore = evaluate_error(initialEstimate, poses_ground_truth, landmarks_ground_truth, poseSymbols, landmarkSymbols);
    std::cout << "MAE before optimization: " << errorBefore.first << " m" << std::endl;
    std::cout << "RMSE before optimization: " << errorBefore.second << " m" << std::endl;

    auto errorAfter = evaluate_error(finalEstimates, poses_ground_truth, landmarks_ground_truth, poseSymbols, landmarkSymbols);
    std::cout << "MAE after optimization: " << errorAfter.first << " m" << std::endl;
    std::cout << "RMSE after optimization: " << errorAfter.second << " m" << std::endl;
    std::cout << std::endl;


    std::string res_path = "./res/batch";
    fs::create_directories(res_path);

    // std::string res_exp_path = res_path + "/benchmark_" + std::to_string(benchmark) + "exp_" + std::to_string(exp);
    // fs::create_directories(res_exp_path);

    // {
    //     std::ofstream outputFile(res_exp_path + "/before.txt");
    //     if (outputFile.is_open())
    //     {
    //         saveValues(outputFile, initialEstimate, initialMarginals, poseSymbols, landmarkSymbols, graph);
    //         outputFile.close();
    //         std::cout << "Result saved in " + res_exp_path + "/before.txt" << std::endl;
    //     }
    //     else
    //     {
    //         throw std::runtime_error("Error opening file: " + res_exp_path + "/before.txt");
    //     }
    // }

    // {
    //     std::ofstream outputFile(res_exp_path + "/after.txt");
    //     if (outputFile.is_open())
    //     {
    //         saveValues(outputFile, finalEstimates, marginals, poseSymbols, landmarkSymbols, graph);
    //         outputFile.close();
    //         std::cout << "Result saved in " + res_exp_path + "/after.txt" << std::endl;
    //     }
    //     else
    //     {
    //         throw std::runtime_error("Error opening file: " + res_exp_path + "/after.txt");
    //     }
    // }

    {
        YAML::Node params = YAML::LoadFile("./data/benchmark_" + std::to_string(benchmark) + "/exp_" + std::to_string(exp) + "/params.yaml");

        std::string landmark_err = params["landmark_err"].as<std::string>();
        std::string pose_pos_err = params["pose_pos_err"].as<std::string>();
        std::string pose_theta_err = params["pose_theta_err"].as<std::string>();
        std::string obs_range_err_perc_min = params["obs_range_err_perc_min"].as<std::string>();
        std::string obs_range_err_perc_max = params["obs_range_err_perc_max"].as<std::string>();
        std::string obs_bearing_err = params["obs_bearing_err"].as<std::string>();
        std::string odo_pos_err = params["odo_pos_err"].as<std::string>();
        std::string odo_theta_err = params["odo_theta_err"].as<std::string>();
        std::string obs_outlier_rate = params["obs_outlier_rate"].as<std::string>();
        std::string obs_outlier_range_err_perc_min = params["obs_outlier_range_err_perc_min"].as<std::string>();
        std::string obs_outlier_range_err_perc_max = params["obs_outlier_range_err_perc_max"].as<std::string>();
        std::string obs_outlier_bearing_err = params["obs_outlier_bearing_err"].as<std::string>();
        
        std::string filename = "./res/batch/benchmark_" + std::to_string(benchmark) + ".csv";

        bool writeHeader = (!fs::exists(filename) || !(fs::file_size(filename) > 0));
        
        std::ofstream resFile(filename, std::ios::app);
        if (!resFile) {
            throw std::runtime_error("Error opening file: " + filename);
        }

        if (writeHeader) 
            resFile << "exp,optimizer,landmark_err,pose_pos_err,pose_theta_err,obs_range_err_perc_min,obs_range_err_perc_max,obs_bearing_err,odo_pos_err,odo_theta_err,obs_outlier_rate,obs_outlier_range_err_perc_min,obs_outlier_range_err_perc_max,obs_outlier_bearing_err,computation_time,mae_before,mae_after,rmse_before,rmse_after\n";
        

        resFile << exp << "," \
            << optimizerName << "," \
            << landmark_err << "," \
            << pose_pos_err << "," \
            << pose_theta_err << "," \
            << obs_range_err_perc_min << "," \
            << obs_range_err_perc_max << "," \
            << obs_bearing_err << "," \
            << odo_pos_err << "," \
            << odo_theta_err << "," \
            << obs_outlier_rate << "," \
            << obs_outlier_range_err_perc_min << "," \
            << obs_outlier_range_err_perc_max << "," \
            << obs_outlier_bearing_err << "," \
            << computation_time << "," \
            << errorBefore.first << "," \
            << errorAfter.first << "," \
            << errorBefore.second << "," \
            << errorAfter.second << "\n";

        resFile.close();

        std::cout << "Benchmark results updated in " + filename << std::endl;
    }

    // graph.print("Factor Graph:\n");

    std::cout << "----------------------------------------" << std::endl;
    return 0;
}
