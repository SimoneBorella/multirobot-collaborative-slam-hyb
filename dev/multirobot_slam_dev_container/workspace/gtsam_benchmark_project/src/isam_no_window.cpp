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


namespace std {
    template <>
    struct hash<gtsam::Symbol> {
        size_t operator()(const gtsam::Symbol& symbol) const {
            // Combine the character and integer part of the Symbol to generate a hash
            size_t h1 = std::hash<char>{}(symbol.chr());  // Hash for the character
            size_t h2 = std::hash<int>{}(symbol.index()); // Hash for the integer index

            // Combine both hashes using bitwise XOR and bit shifts
            return h1 ^ (h2 << 1); 
        }
    };
}







double mod(double x, double y)
{
    return std::sqrt(x*x + y*y);
}

std::pair<double, double> evaluate_error(Values &values,
    std::map<Symbol, Pose2> &poses_ground_truth, std::map<Symbol, Point2> &landmarks_ground_truth,
    std::vector<Symbol> &poseSymbols, std::vector<Symbol> &landmarkSymbols)
{

    double cumulativeAbsError = 0.0;
    double cumulativeSquaredError = 0.0;

    for (const Symbol &x : poseSymbols) {
        Pose2 poseGroundTruth = poses_ground_truth[x];
        Pose2 poseEstimate = values.at<Pose2>(x);

        double absError = mod((poseEstimate.x() - poseGroundTruth.x()), (poseEstimate.y() - poseGroundTruth.y()));
        cumulativeAbsError += absError;
        cumulativeSquaredError += absError*absError;
    }


    for (const Symbol &l : landmarkSymbols) {
        Point2 landmarkGroundTruth = landmarks_ground_truth[l];
        Point2 landmarkEstimate = values.at<Point2>(l);

        double absError = mod((landmarkEstimate.x() - landmarkGroundTruth.x()), (landmarkEstimate.y() - landmarkGroundTruth.y()));
        cumulativeAbsError += absError;
        cumulativeSquaredError += absError*absError;
    }


    double meanAbsError = cumulativeAbsError / (poseSymbols.size() + landmarkSymbols.size());
    double rootMeanSquaredError = std::sqrt(cumulativeSquaredError / (poseSymbols.size() + landmarkSymbols.size()));

    return std::make_pair(meanAbsError, rootMeanSquaredError);
}



std::map<Symbol, Point2> get_landmarks(const std::string &filename) {
    std::map<Symbol, Point2> landmarks;
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

        landmarks[Symbol('l', l_id)] = Point2(x, y);
    }

    file.close();
    return landmarks;
}





std::map<Symbol, Pose2> get_poses(const std::string &filename) {
    std::map<Symbol, Pose2> poses;
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

        poses[Symbol('x', p_id)] = Pose2(x, y, theta);
    }

    file.close();
    return poses;
}






std::unordered_map<Symbol, std::vector<std::pair<Symbol, std::pair<Rot2, double>>>> get_observations(const std::string &filename) {
    std::unordered_map<Symbol, std::vector<std::pair<Symbol, std::pair<Rot2, double>>>> observations;
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

        std::pair<Symbol, std::pair<Rot2, double>> observation(
            Symbol('l', l_id), 
            std::pair<Rot2, double>(Rot2::fromAngle(bearing), range)
        );

        observations[Symbol('x', p_id)].push_back(observation);
    }

    file.close();
    return observations;
}




std::unordered_map<Symbol, std::pair<Symbol, Pose2>> get_odometries(const std::string &filename) {
    std::unordered_map<Symbol, std::pair<Symbol, Pose2>> odometries;
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

        std::pair<Symbol, Pose2> odometry(
            Symbol('x', p_id1),
            Pose2(x, y, theta)
        );

        odometries[Symbol('x', p_id0)] = odometry;
    }

    file.close();
    return odometries;
}



bool contains(const std::vector<Symbol>& vec, Symbol value) {
    return std::find(vec.begin(), vec.end(), value) != vec.end();
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
        optimizerName = parser.get<std::string>("optimizer", "ISAM2");

        std::cout << "Running benchmark: " << benchmark << std::endl;
        std::cout << "Running experiment: " << exp << std::endl;
        std::cout << "Optimizer: " << optimizerName << std::endl;
        std::cout << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
    }


    if (optimizerName != "ISAM2")
    {
        throw std::runtime_error("Invalid optimizer: " + optimizerName);
    }


    std::string res_path = "./res/incremental_no_window/benchmark_" + std::to_string(benchmark) + "/exp_" + std::to_string(exp);
    fs::create_directories(res_path);

    std::string params_path = "./data/benchmark_" + std::to_string(benchmark) + "/exp_" + std::to_string(exp) + "/params.yaml";
    fs::copy(params_path, res_path, fs::copy_options::overwrite_existing);

    std::string filename = res_path + "/results.csv";


    std::ofstream resFile(filename, std::ios::out);
    if (!resFile) {
        throw std::runtime_error("Error opening file: " + filename);
    }

    resFile << "iteration,update_time,optimization_time,mae,rmse\n";
        




    std::vector<Symbol> poseSymbols;
    std::vector<Symbol> landmarkSymbols;

    std::map<Symbol, Point2> landmarks_ground_truth;
    std::map<Symbol, Pose2> poses_ground_truth;

    std::map<Symbol, Point2> landmarks_estimates;
    std::map<Symbol, Pose2> poses_estimates;
    
    std::unordered_map<Symbol, std::vector<std::pair<Symbol, std::pair<Rot2, double>>>> observations;
    std::unordered_map<Symbol, std::pair<Symbol, Pose2>> odometries;

  
    landmarks_ground_truth = get_landmarks("./data/ground_truth/landmarks.csv");
    poses_ground_truth = get_poses("./data/ground_truth/poses.csv");
    
    landmarks_estimates = get_landmarks("./data/benchmark_" + std::to_string(benchmark) + "/exp_" + std::to_string(exp) + "/landmarks_initial_estimates.csv");
    poses_estimates = get_poses("./data/benchmark_" + std::to_string(benchmark) + "/exp_" + std::to_string(exp) + "/poses_initial_estimates.csv");
    
    observations = get_observations("./data/benchmark_" + std::to_string(benchmark) + "/exp_" + std::to_string(exp) + "/observations.csv");
    odometries = get_odometries("./data/benchmark_" + std::to_string(benchmark) + "/exp_" + std::to_string(exp) + "/odometries.csv");



    // ISAM2 initialization

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    parameters.cacheLinearizedFactors = false;
    parameters.enableDetailedResults = true;
    // parameters.print();

    ISAM2 optimizer(parameters);
    ISAM2Result result;
    


    Values initialEstimate;
    Values finalEstimates;
    NonlinearFactorGraph graph;

    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.005, 0.005, 0.005));
    auto observationNoise = noiseModel::Diagonal::Sigmas(Vector2(0.005, 0.005));          // Noise 0.05 rad std on bearing and 5cm std on range
    auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.005, 0.005, 0.005));      // Noise 5cm std on x,y and 0.05 rad std on theta



    

    std::cout << "Simulation started..." << std::endl;
    auto start_simulation = std::chrono::high_resolution_clock::now();

    Symbol x0('x', 0);
    Pose2 prior(0.0, 0.0, -0.5);

    graph.addPrior(x0, prior, priorNoise);
    
    // Iterate over poses
    for (const auto& [x, pose_estimate] : poses_estimates)
    {
        std::cout << "Current pose: " << x << std::endl;

        // Insert odometry from last pose 
        if(poseSymbols.size()!=0)
        {
            auto [x_last, odometry] = odometries[poseSymbols[poseSymbols.size()-1]];
            graph.emplace_shared<BetweenFactor<Pose2>>(x_last, x, odometry, odometryNoise);
        }
        
        
        // Insert pose initial estimate
        initialEstimate.insert(x, pose_estimate);
        poseSymbols.push_back(x);

        // Retrieve observations from current pose
        std::vector<std::pair<Symbol, std::pair<Rot2, double>>> observations_x = observations[x];
        
        for (const auto& [l, observation] : observations_x)
        {
            Rot2 bearing = observation.first;
            double range = observation.second;

            // If the observation has never been seen so far add landmark initial estimate
            if(!contains(landmarkSymbols, l))
            {
                auto landmark_estimate = landmarks_estimates[l];
                std::cout << "Adding landmark " << l << std::endl;
                initialEstimate.insert(l, landmark_estimate);
                landmarkSymbols.push_back(l);
            }

            std::cout << "Adding observation " << x << " " << l << std::endl;
            graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(x, l, bearing, range, observationNoise);
        }
        std::cout << std::endl;
        
        // Optimization

        std::cout << "Updating optimizer..." << std::endl;
        auto start_update = std::chrono::high_resolution_clock::now();

        result = optimizer.update(graph, initialEstimate);
        
        auto end_update = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration_update = end_update - start_update;
        double computation_time_update = duration_update.count() * 1000;
        std::cout << "Optimizer updated." << std::endl;
        std::cout << "Computation time: " << computation_time_update << " ms" << std::endl;
        std::cout << std::endl;



        std::cout << "Optimizing factor graph..." << std::endl;
        auto start_optimization = std::chrono::high_resolution_clock::now();
        
        finalEstimates = optimizer.calculateEstimate();

        auto end_optimization = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration_optimization = end_optimization - start_optimization;
        double computation_time_optimization = duration_optimization.count() * 1000;
        std::cout << "Factor graph optimized." << std::endl;
        std::cout << "Computation time: " << computation_time_optimization << " ms" << std::endl;
        std::cout << std::endl;


        auto error = evaluate_error(finalEstimates, poses_ground_truth, landmarks_ground_truth, poseSymbols, landmarkSymbols);
        std::cout << "MAE: " << error.first << " m" << std::endl;
        std::cout << "RMSE: " << error.second << " m" << std::endl;
        std::cout << std::endl;

        resFile << x.index() << ',' << computation_time_update << ',' << computation_time_optimization << ',' << error.first << ',' << error.second << '\n';

        initialEstimate.clear();
        graph.resize(0);
    }


    auto end_simulation = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_simulation = end_simulation - start_simulation;
    double computation_time_simulation = duration_simulation.count() * 1000;
    std::cout << "Simulation finished." << std::endl;
    std::cout << "Computation time: " << computation_time_simulation << " ms" << std::endl;
    std::cout << std::endl;

    resFile.close();
    std::cout << "Benchmark results updated in " + filename << std::endl;

    std::cout << "----------------------------------------" << std::endl;
    return 0;
}
