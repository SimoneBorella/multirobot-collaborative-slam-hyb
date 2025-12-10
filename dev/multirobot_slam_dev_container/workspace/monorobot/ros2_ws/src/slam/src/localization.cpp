#include "localization.h"

Localization::Localization(rclcpp::Node::SharedPtr node)
{
    this->node = node;

    map_frame = node->get_parameter("map_frame").as_string();
    odom_frame = node->get_parameter("odom_frame").as_string();
    base_frame = node->get_parameter("base_frame").as_string();
    
    localization_rate = node->get_parameter("localization_rate").as_double();
    fine_localization_rate = node->get_parameter("fine_localization_rate").as_double();
    map_to_odom_rate = node->get_parameter("map_to_odom_rate").as_double();

    map_to_odom_correction = node->get_parameter("map_to_odom_correction").as_bool();

    isam_relinearize_threshold = node->get_parameter("isam_relinearize_threshold").as_double();
    isam_relinearize_skip = node->get_parameter("isam_relinearize_skip").as_int();
    
    std::vector<double> init_prior_noise_vec = node->get_parameter("init_prior_noise").as_double_array();
    std::vector<double> marginal_prior_noise_vec = node->get_parameter("marginal_prior_noise").as_double_array();
    std::vector<double> observation_noise_vec = node->get_parameter("observation_noise").as_double_array();
    std::vector<double> odometry_noise_vec = node->get_parameter("odometry_noise").as_double_array();

    init_prior_noise = noiseModel::Diagonal::Sigmas(Vector3(init_prior_noise_vec[0], init_prior_noise_vec[1], init_prior_noise_vec[2]));
    marginal_prior_noise = noiseModel::Diagonal::Sigmas(Vector3(marginal_prior_noise_vec[0], marginal_prior_noise_vec[1], marginal_prior_noise_vec[2]));
    observation_noise = noiseModel::Diagonal::Sigmas(Vector2(observation_noise_vec[0], observation_noise_vec[1]));
    odometry_noise = noiseModel::Diagonal::Sigmas(Vector3(odometry_noise_vec[0], odometry_noise_vec[1], odometry_noise_vec[2]));
    
    window_size =  static_cast<size_t>(node->get_parameter("window_size").as_int());
    lag =  static_cast<size_t>(node->get_parameter("lag").as_int());

    use_fixed_marginal = node->get_parameter("use_fixed_marginal").as_bool();

    min_pose_displacement = node->get_parameter("min_pose_displacement").as_double();
    min_pose_theta_displacement = node->get_parameter("min_pose_theta_displacement").as_double() * M_PI / 180.0;
    
    data_association_mode = node->get_parameter("data_association_mode").as_string();
    use_all_landmarks = node->get_parameter("use_all_landmarks").as_bool();
    data_association_distance = node->get_parameter("data_association_distance").as_double();
    mahalanobis_dist_threshold = node->get_parameter("mahalanobis_dist_threshold").as_double();

    // Create subscriptions 
    landmarks_subscription = node->create_subscription<interfaces::msg::PointArray>(
        "landmarks", 10, std::bind(&Localization::landmarks_callback, this, std::placeholders::_1));
    
    scan_subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Localization::scan_callback, this, std::placeholders::_1));

    // Create tf2 listener
    tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Create tf2 broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    

    // Initialize ISAM2
    ISAM2Params parameters;
    parameters.relinearizeThreshold = isam_relinearize_threshold;
    parameters.relinearizeSkip = isam_relinearize_skip;
    parameters.cacheLinearizedFactors = false;

    optimizer = std::make_unique<ISAM2>(parameters);
    
    // Initialize graph params
    pose_id = 0;
    landmark_id = 0;

    factor_id_ref = 0;

    map_to_odom = Pose2(0.0, 0.0, 0.0);

    // Initializa posed scans
    posed_scans = std::make_shared<std::vector<PosedScan>>();
    posed_scans_mutex = std::make_shared<std::shared_mutex>();

    // Initialize threads
    localization_running = false;
    fine_localization_running = false;
    map_to_odom_running = false;
}

Localization::~Localization()
{
    localization_running = false;
    fine_localization_running = false;
    map_to_odom_running = false;

    if (localization_thread && localization_thread->joinable()) {
        localization_thread->join();
    }
    if (fine_localization_thread && fine_localization_thread->joinable()) {
        fine_localization_thread->join();
    }
    if (map_to_odom_thread && map_to_odom_thread->joinable()) {
        map_to_odom_thread->join();
    }
}


void Localization::startLocalization()
{
    if(localization_rate != 0.0)
    {
        localization_running = true;
        localization_thread = std::make_unique<std::thread>([this]() {
            rclcpp::Rate rate(localization_rate);
            while (rclcpp::ok() && localization_running.load()) {
                localization();
                rate.sleep();
            }
        });
        RCLCPP_INFO_STREAM(node->get_logger(), "Localization thread started.");
    }

    if(fine_localization_rate != 0.0)
    {
        fine_localization_running = true;
        fine_localization_thread = std::make_unique<std::thread>([this]() {
            rclcpp::Rate rate(fine_localization_rate);
            while (rclcpp::ok() && fine_localization_running.load()) {
                fine_localization();
                rate.sleep();
            }
        });
        RCLCPP_INFO_STREAM(node->get_logger(), "Fine Localization thread started.");
    }
    
    if(map_to_odom_rate != 0.0)
    {
        map_to_odom_running = true;
        map_to_odom_thread = std::make_unique<std::thread>([this]() {
            rclcpp::Rate rate(map_to_odom_rate);
            while (rclcpp::ok() && map_to_odom_running.load()) {
                publish_map_to_odom();
                rate.sleep();
            }
        });
        RCLCPP_INFO_STREAM(node->get_logger(), "Map to odom thread started.");
    }
}


void Localization::landmarks_callback(const interfaces::msg::PointArray::SharedPtr landmarks_msg)
{
    this->landmarks = landmarks_msg;
}

void Localization::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    this->scan = scan_msg;
}

void Localization::publish_map_to_odom()
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = map_frame;
    transform.child_frame_id = odom_frame;

    // Translation
    transform.transform.translation.x = map_to_odom.x();
    transform.transform.translation.y = map_to_odom.y();
    transform.transform.translation.z = 0.0;

    // Rotation
    tf2::Quaternion q;
    q.setRPY(0, 0, map_to_odom.theta());
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    transform.header.stamp = node->get_clock()->now();
    tf_broadcaster->sendTransform(transform);
}



std::optional<Symbol> Localization::nearest_neighbor_data_association(const Point2& observed_point)
{
    double min_dist = std::numeric_limits<double>::max();
    std::optional<Symbol> associated_l;

    Values estimates;
    if (use_all_landmarks)
        estimates = graph_estimates;
    else
        estimates = opt_estimates;

    for (const Symbol& l : landmark_symbols)
    {
        if (estimates.exists(l))
        {
            Point2 landmark_estimate = estimates.at<Point2>(l);
            double dist = (observed_point - landmark_estimate).norm();

            if (dist < data_association_distance && dist < min_dist)
            {
                min_dist = dist;
                associated_l = l;
            }
        }
    }

    return associated_l;
}





std::optional<Symbol> Localization::flann_data_association(const Point2& observed_point)
{
    if (landmark_symbols.empty())
        return std::nullopt;

    // Prepare the query point as float
    std::vector<float> query = { 
        static_cast<float>(observed_point.x()), 
        static_cast<float>(observed_point.y()) 
    };

    // Flatten the landmark dataset
    std::vector<float> dataset;
    std::vector<Symbol> valid_symbols;

    Values estimates;
    if (use_all_landmarks)
        estimates = graph_estimates;
    else
        estimates = opt_estimates;

    for (const Symbol& l : landmark_symbols)
    {
        if (estimates.exists(l))
        {
            Point2 p = estimates.at<Point2>(l);
            dataset.push_back(static_cast<float>(p.x()));
            dataset.push_back(static_cast<float>(p.y()));
            valid_symbols.push_back(l);
        }
    }

    if (dataset.empty())
        return std::nullopt;

    flann::Matrix<float> dataset_mat(dataset.data(), valid_symbols.size(), 2);
    flann::Matrix<float> query_mat(query.data(), 1, 2);

    std::vector<int> indices(1);
    std::vector<float> dists(1);
    flann::Matrix<int> indices_mat(indices.data(), 1, 1);
    flann::Matrix<float> dists_mat(dists.data(), 1, 1);

    flann::Index<flann::L2<float>> index(dataset_mat, flann::KDTreeIndexParams(1));
    index.buildIndex();
    index.knnSearch(query_mat, indices_mat, dists_mat, 1, flann::SearchParams(128));

    float min_dist = dists[0];
    if (min_dist < data_association_distance * data_association_distance)
        return valid_symbols[indices[0]];

    return std::nullopt;
}


std::vector<std::pair<Symbol, double>> Localization::probabilistic_data_association(const Point2& observed_point)
{
    std::vector<std::pair<Symbol, double>> results;

    if (landmark_symbols.empty())
        return results;

    double total_likelihood = 0.0;
    std::vector<std::tuple<Symbol, double, double>> temp_results;

    Values estimates;
    if (use_all_landmarks)
        estimates = graph_estimates;
    else
        estimates = opt_estimates;

    for (const Symbol& l : landmark_symbols)
    {
        if (estimates.exists(l))
        {
            Point2 landmark_estimate = estimates.at<Point2>(l);
            Vector2 dist = observed_point - landmark_estimate;

            if (dist.norm() > data_association_distance)
                continue;

            Matrix2 cov = marginals.marginalCovariance(l);

            double cov_determinant = cov.determinant();
            if (cov_determinant < 1e-6) {
                cov_determinant = 1e-6;
            }

            Matrix2 cov_inv = cov.inverse();
            double mahalanobis_dist = std::sqrt(dist.transpose() * cov_inv * dist);

            if (mahalanobis_dist > mahalanobis_dist_threshold)
                continue;

            // Compute likelihood using Mahalanobis distance
            double likelihood = (1.0 / (2.0 * M_PI * std::sqrt(cov_determinant))) * std::exp(-0.5 * mahalanobis_dist);

            // Apply threshold to likelihood to ignore weak associations
            temp_results.push_back({l, likelihood, mahalanobis_dist});
            total_likelihood += likelihood;
            
        }
    }

    // Normalize likelihoods to get probabilities
    for (const auto& [l, likelihood, _] : temp_results)
    {
        if (total_likelihood > 0.0)
        {
            double probability = likelihood / total_likelihood;
            results.emplace_back(l, probability);
        }
    }

    return results;
}





void Localization::localization()
{
#ifdef LOCALIZATION_DEBUG
    auto start = std::chrono::high_resolution_clock::now();
#endif

    // Check if landmarks are available
    bool landmarks_available = true;
    if (!landmarks || landmarks->points.size() <= 0)
    {
        landmarks_available = false;
    }

    // Check if scan is available
    bool scan_available = true;
    if (!scan)
    {
        scan_available = false;
    }

    if (!landmarks_available && !scan_available)
        return;


    if (scan_available)
    {
        // Get pose
        geometry_msgs::msg::TransformStamped odom_transform_stamped;
        try
        {
            odom_transform_stamped = tf_buffer->lookupTransform(
                map_frame,
                base_frame,
                scan->header.stamp);
        }
        catch (const tf2::TransformException &ex)
        {
            // RCLCPP_ERROR_STREAM(node->get_logger(), "Transform not available: " << ex.what());
            return;
        }

        double pose_x = odom_transform_stamped.transform.translation.x;
        double pose_y = odom_transform_stamped.transform.translation.y;
        // double pose_z = odom_transform_stamped.transform.translation.z;

        double pose_roll, pose_pitch, pose_yaw;
        tf2::Quaternion quat_tf;
        tf2::fromMsg(odom_transform_stamped.transform.rotation, quat_tf);
        tf2::Matrix3x3(quat_tf).getRPY(pose_roll, pose_pitch, pose_yaw);

        Pose2 pose(pose_x, pose_y, pose_yaw);
        

        geometry_msgs::msg::TransformStamped scan_transform_stamped;
        try
        {
            scan_transform_stamped = tf_buffer->lookupTransform(
                base_frame,
                scan->header.frame_id,
                scan->header.stamp);
            }
        catch (const tf2::TransformException &ex)
        {
            return;
        }
        
        
        sensor_msgs::msg::LaserScan transformed_scan;

        {
            std::unique_lock lock(*posed_scans_mutex);

            transformed_scan.header.stamp = scan->header.stamp;
            transformed_scan.header.frame_id = base_frame;

            transformed_scan.angle_min = scan->angle_min;
            transformed_scan.angle_max = scan->angle_max;
            transformed_scan.angle_increment = scan->angle_increment;
            transformed_scan.range_min = scan->range_min;
            transformed_scan.range_max = scan->range_max;
            transformed_scan.time_increment = scan->time_increment;
            transformed_scan.scan_time = scan->scan_time;

            transformed_scan.ranges.resize(scan->ranges.size(), std::numeric_limits<float>::quiet_NaN());


            tf2::Transform scan_tf2_transform;
            tf2::fromMsg(scan_transform_stamped.transform, scan_tf2_transform);

            double angle = scan->angle_min;
            for (size_t i = 0; i < scan->ranges.size(); i++, angle += scan->angle_increment)
            {
                float range = scan->ranges[i];

                if (std::isnan(range) || std::isinf(range))
                {
                    transformed_scan.ranges[i] = range;
                    continue;
                }

                tf2::Vector3 point_laser(range * std::cos(angle), range * std::sin(angle), 0.0);
                tf2::Vector3 point_base = scan_tf2_transform * point_laser;

                transformed_scan.ranges[i] = static_cast<float>((point_base).length());
            }
        }

        PosedScan posed_scan(pose, transformed_scan, true);
        (*posed_scans).push_back(posed_scan);
        scan.reset();
    }


    if(!landmarks_available)
        return;

    // Get pose
    geometry_msgs::msg::TransformStamped odom_transform_stamped;
    try
    {
        odom_transform_stamped = tf_buffer->lookupTransform(
            map_frame,
            base_frame,
            landmarks->header.stamp);
    }
    catch (const tf2::TransformException &ex)
    {
        // RCLCPP_ERROR_STREAM(node->get_logger(), "Transform not available: " << ex.what());
        return;
    }

    double pose_x = odom_transform_stamped.transform.translation.x;
    double pose_y = odom_transform_stamped.transform.translation.y;
    // double pose_z = odom_transform_stamped.transform.translation.z;

    double pose_roll, pose_pitch, pose_yaw;
    tf2::Quaternion quat_tf;
    tf2::fromMsg(odom_transform_stamped.transform.rotation, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(pose_roll, pose_pitch, pose_yaw);

    Pose2 pose(pose_x, pose_y, pose_yaw);



    // Initialize optimizer variables
    NonlinearFactorGraph new_factors;
    NonlinearFactorGraph new_temp_prior_factors;
    NonlinearFactorGraph new_odom_factors;
    NonlinearFactorGraph new_obs_factors;
    Values initial_estimate;
    FactorIndices remove_factor_indicies;


    // Add pose
    Symbol x('x', pose_id);

    bool displacement_over_min_threshold = true;

    if (pose_id == 0)
    {
        // Add prior to first pose
        new_odom_factors.addPrior(x, pose, init_prior_noise);
    }
    else
    {
        Pose2 odometry = last_pose.between(pose);

        // Check minimum displacement from last pose
        double displacement = odometry.translation().norm();
        double theta_displacement = std::abs(odometry.rotation().theta());

        if (displacement < min_pose_displacement && theta_displacement < min_pose_theta_displacement)
            displacement_over_min_threshold = false;

        new_odom_factors.emplace_shared<BetweenFactor<Pose2>>(last_x, x, odometry, odometry_noise);
    }

    

    if(!displacement_over_min_threshold)
        return;


    initial_estimate.insert(x, pose);
    pose_symbols.push_back(x);

    // Add landmarks

    std::vector<geometry_msgs::msg::Point> landmarks_points = landmarks->points;

    // Transform landmarks in map frame
    std::vector<geometry_msgs::msg::Point> map_landmarks_points;
    geometry_msgs::msg::TransformStamped landmark_transform_stamped;

    try
    {
        landmark_transform_stamped = tf_buffer->lookupTransform(
            map_frame,
            landmarks->header.frame_id,
            landmarks->header.stamp);
    }
    catch (const tf2::TransformException &ex)
    {
        // RCLCPP_ERROR_STREAM(node->get_logger(), "Transform not available: " << ex.what());
        return;
    }

    
    
    
    
    
    geometry_msgs::msg::PointStamped point_in, point_out;
    point_in.header = landmarks->header;
    
    for (const geometry_msgs::msg::Point &point : landmarks->points)
    {
        point_in.point = point;
        
        try
        {
            tf2::doTransform(point_in, point_out, landmark_transform_stamped);
            map_landmarks_points.push_back(point_out.point);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_STREAM(node->get_logger(), "Transform failed for a landmark point: " << ex.what());
            continue;
        }
    }
    

    
    // Add landmarks to graph

    auto huber_noise = noiseModel::Robust::Create(
        noiseModel::mEstimator::Huber::Create(1.345),
        observation_noise
    );

    for (const geometry_msgs::msg::Point &point : map_landmarks_points)
    {
        // Compute measure from pose to landmark
        Point2 observed_map_landmark(point.x, point.y);

        Rot2 bearing = pose.bearing(observed_map_landmark);
        double range = pose.range(observed_map_landmark);

        // Data association
        if(data_association_mode == "nn" || data_association_mode == "flann")
        {
            std::optional<Symbol> associated_l_opt;
            if(data_association_mode == "nn")
                associated_l_opt = nearest_neighbor_data_association(observed_map_landmark);
            else if (data_association_mode == "flann")
                associated_l_opt = flann_data_association(observed_map_landmark);
    
            // Add landmark if not associated and add landmark measure to graph
            if (associated_l_opt.has_value())
            {
                Symbol associated_l = associated_l_opt.value();
                new_obs_factors.emplace_shared<BearingRangeFactor<Pose2, Point2>>(x, associated_l, bearing, range, huber_noise);
                
                if (!opt_estimates.exists(associated_l))
                    initial_estimate.insert_or_assign(associated_l, graph_estimates.at<Point2>(associated_l));
            }
            else
            {
                Symbol l('l', landmark_id++);
                initial_estimate.insert(l, observed_map_landmark);
                landmark_symbols.push_back(l);
                new_obs_factors.emplace_shared<BearingRangeFactor<Pose2, Point2>>(x, l, bearing, range, huber_noise);
            }
        }
        else if (data_association_mode == "pda")
        {
            auto associations = probabilistic_data_association(observed_map_landmark);
            
            if (!associations.empty())
            {
                for (const auto& [associated_l, probability] : associations)
                {
                    // To scale information
                    // I * p  =>  S / p => sigmas / sqrt(p)
                    auto scaled_noise = noiseModel::Robust::Create(
                        noiseModel::mEstimator::Huber::Create(1.345),
                        noiseModel::Diagonal::Sigmas(
                            observation_noise->sigmas() / std::sqrt(probability)
                        )
                    );

                    new_obs_factors.emplace_shared<BearingRangeFactor<Pose2, Point2>>(x, associated_l, bearing, range, scaled_noise);
                    if (!opt_estimates.exists(associated_l))
                        initial_estimate.insert_or_assign(associated_l, graph_estimates.at<Point2>(associated_l));
                }
            }
            else
            {
                Symbol l('l', landmark_id++);
                initial_estimate.insert(l, observed_map_landmark);
                landmark_symbols.push_back(l);
                new_obs_factors.emplace_shared<BearingRangeFactor<Pose2, Point2>>(x, l, bearing, range, huber_noise);
            }
        }
    }

    landmarks.reset();

    

    // Compute factors to remove and update temporary prior factors

    if (window_size != static_cast<size_t>(-1))
    {
        bool insert_temp_prior = false;

        // Remove temporary prior factors
        if(temp_factor_indices.size() > 0)
        {
            for(size_t factor_id : temp_factor_indices)
                remove_factor_indicies.push_back(factor_id);
            
            temp_factor_indices.clear();
        }
        
        // Remove factors for poses out of the window
        if (odom_factor_indices_per_pose.size() > window_size)
        {
            for (size_t factor_id = odom_factor_indices_per_pose[0].first; factor_id < odom_factor_indices_per_pose[0].second; factor_id++)
                remove_factor_indicies.push_back(factor_id);
    
            odom_factor_indices_per_pose.erase(odom_factor_indices_per_pose.begin());
        }

        if (obs_factor_indices_per_pose.size() > window_size)
        {
            if (odom_factor_indices_per_pose.size() > window_size-1)
            {
                for (size_t factor_id = odom_factor_indices_per_pose[0].first; factor_id < odom_factor_indices_per_pose[0].second; factor_id++)
                    remove_factor_indicies.push_back(factor_id);
        
                odom_factor_indices_per_pose.erase(odom_factor_indices_per_pose.begin());
            }

            for (size_t factor_id = obs_factor_indices_per_pose[0].first; factor_id < obs_factor_indices_per_pose[0].second; factor_id++)
                remove_factor_indicies.push_back(factor_id);
    
            obs_factor_indices_per_pose.erase(obs_factor_indices_per_pose.begin());
            insert_temp_prior = true;
        }

        // Update per pose odom factor indices
        odom_factor_indices_per_pose.push_back(std::make_pair(factor_id_ref, factor_id_ref + new_odom_factors.size()));
        factor_id_ref += new_odom_factors.size();

        // Update per pose obs factor indices
        obs_factor_indices_per_pose.push_back(std::make_pair(factor_id_ref, factor_id_ref + new_obs_factors.size()));
        factor_id_ref += new_obs_factors.size();
        

        // Add new initial temporary prior factors
        if (insert_temp_prior)
        {
            Symbol window_x = Symbol('x', x.index() - window_size);
            if (use_fixed_marginal)
                new_temp_prior_factors.addPrior(window_x, graph_estimates.at<Pose2>(window_x), marginal_prior_noise);
            else
                new_temp_prior_factors.addPrior(window_x, graph_estimates.at<Pose2>(window_x), marginals.marginalCovariance(window_x));

            temp_factor_indices.push_back(factor_id_ref);
            factor_id_ref++;
        }
        
    }
    
    graph.push_back(new_odom_factors);
    graph.push_back(new_obs_factors);
    

    new_factors.push_back(new_odom_factors);
    new_factors.push_back(new_obs_factors);
    new_factors.push_back(new_temp_prior_factors);



    // Update odometry pose pool

    window_odom_poses.push_back(pose);
    if (window_odom_poses.size() > window_size)
        window_odom_poses.erase(window_odom_poses.begin());



    // Update inserting new factors and removing old factors
#ifdef LOCALIZATION_DEBUG
    auto start_update = std::chrono::high_resolution_clock::now();
#endif

    optimizer->update(new_factors, initial_estimate, remove_factor_indicies);

#ifdef LOCALIZATION_DEBUG
    auto end_update = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_update = end_update - start_update;
#endif


    // Optimize graph
#ifdef LOCALIZATION_DEBUG
    auto start_opt = std::chrono::high_resolution_clock::now();
#endif

    opt_estimates = optimizer->calculateEstimate();

#ifdef LOCALIZATION_DEBUG
    auto end_opt = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_opt = end_opt - start_opt;
#endif

    graph_estimates.insert_or_assign(opt_estimates);

    pose_id++;
    last_x = x;
    last_pose = pose;

    if(map_to_odom_correction && window_odom_poses.size() >= lag)
    {
        Pose2 correction_odom_pose = window_odom_poses[window_odom_poses.size() - lag];
        Pose2 correction_map_pose = opt_estimates.at<Pose2>(Symbol('x', x.index() - lag + 1));

        map_to_odom = correction_map_pose * correction_odom_pose.inverse();
    }

    
    if (data_association_mode == "pda")
    {
        Values estimates;

        if(use_all_landmarks)
            marginals = Marginals(graph, graph_estimates, Marginals::CHOLESKY);
        else
            marginals = Marginals(optimizer->getFactorsUnsafe(), opt_estimates, Marginals::CHOLESKY);
    }

#ifdef LOCALIZATION_DEBUG
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
#endif

#ifdef LOCALIZATION_DEBUG
    std::cout << "Pose " << x << ":           " << pose.x() << ", " << pose.y() << ", " << pose.theta() << std::endl;
    std::cout << "Optimized Pose " << x << ": " << optimized_pose.x() << ", " << optimized_pose.y() << ", " << optimized_pose.theta() << std::endl;
    std::cout << "Update time:           " << (duration_update.count() * 1000) << " ms" << std::endl;
    std::cout << "Optimization time:     " << (duration_opt.count() * 1000) << " ms" << std::endl;
    std::cout << "Localization time:     " << (duration.count() * 1000) << " ms" << std::endl;
    std::cout << "Graph size:       " << graph.size() << std::endl;
    std::cout << "ISAM2 Graph size: " << optimizer->getFactorsUnsafe().size() << std::endl;
    std::cout << std::endl;
#endif

#ifdef LOCALIZATION_SAVE_GRAPH_DEBUG
    if (x.index() != 0 && x.index() % 20 == 0)
    {
        // std::cout << "Saving graph" << std::endl;
        // Marginals graph_marginals(graph, graph_estimates);
        // save_graph(graph, graph_estimates, graph_marginals, "./slam/graphs/graph_" + std::string(1, x.chr()) + std::to_string(x.index()) + ".txt");
        // std::cout << "Graph saved" << std::endl;

        std::cout << "Saving ISAM2 graph" << std::endl;
        save_graph(optimizer->getFactorsUnsafe(), opt_estimates, marginals, "./slam/graphs/isam_graph_" + std::string(1, x.chr()) + std::to_string(x.index()) + ".txt");
        std::cout << "ISAM2 graph saved" << std::endl;
    }
#endif
}











void Localization::fine_localization()
{
    // Optimize whole graph
#ifdef FINE_LOCALIZATION_DEBUG
    auto start_opt = std::chrono::high_resolution_clock::now();
    LevenbergMarquardtOptimizer optimizer(graph, graph_estimates);
    opt_estimates = optimizer.optimize();
    auto end_opt = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_opt = end_opt - start_opt;
#else
    LevenbergMarquardtOptimizer optimizer(graph, graph_estimates);
    opt_estimates = optimizer.optimize();
#endif

    // graph_estimates.insert_or_assign(opt_estimates);


#ifdef FINE_LOCALIZATION_DEBUG
    std::cout << "Fine Optimization time:     " << (duration_opt.count() * 1000) << " ms" << std::endl;
    std::cout << std::endl;
#endif
}










void Localization::save_graph(NonlinearFactorGraph graph, Values estimates, Marginals marginals, const std::string &filename)
{
    std::ofstream graph_file(filename);

    if (!graph_file.is_open())
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Could not open file: " << filename);
        return;
    }

    // POSE2 xn Pose2(x, y, theta) Covariance(xx, xy, xtheta, yx, yy, ytheta, thetax, thetay, thetatheta)
    for (Symbol x : pose_symbols)
    {
        if (estimates.exists(x))
        {
            Pose2 pose_estimate = estimates.at<Pose2>(x);
            auto pose_covariance = marginals.marginalCovariance(x);

            graph_file << "POSE2 " << x << " " << pose_estimate.x() << " " << pose_estimate.y() << " " << pose_estimate.theta();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    graph_file << " " << pose_covariance(i, j);
            graph_file << "\n";
        }
    }

    // POINT2 ln Point2(x, y) Covariance(xx, xy, yx, yy)
    for (Symbol l : landmark_symbols)
    {
        if (estimates.exists(l))
        {
            Point2 landmark_estimate = estimates.at<Point2>(l);
            auto landmark_covariance = marginals.marginalCovariance(l);

            graph_file << "POINT2 " << l << " " << landmark_estimate.x() << " " << landmark_estimate.y();
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                    graph_file << " " << landmark_covariance(i, j);
            graph_file << "\n";
        }
    }

    for (const auto &factor : graph)
    {
        // PRIORPOSE2 x0 x y theta Covariance(xx, xy, xtheta, yx, yy, ytheta, thetax, thetay, thetatheta)
        if (auto priorPose = boost::dynamic_pointer_cast<PriorFactor<Pose2>>(factor))
        {
            Symbol key = priorPose->keys().at(0);
            Pose2 prior = priorPose->prior();
            Matrix3 covariance = boost::dynamic_pointer_cast<noiseModel::Gaussian>(priorPose->noiseModel())->covariance();

            graph_file << "PRIORPOSE2 " << key << " " << prior.x() << " " << prior.y() << " " << prior.theta();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    graph_file << " " << covariance(i, j);
            graph_file << "\n";
        }

        // PRIORPOINT2 l0 x y Covariance(xx, xy, yx, yy)
        if (auto priorPoint = boost::dynamic_pointer_cast<PriorFactor<Point2>>(factor))
        {
            Symbol key = priorPoint->keys().at(0);
            Point2 prior = priorPoint->prior();
            Matrix2 covariance = boost::dynamic_pointer_cast<noiseModel::Gaussian>(priorPoint->noiseModel())->covariance();

            graph_file << "PRIORPOINT2 " << key << " " << prior.x() << " " << prior.y();
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                    graph_file << " " << covariance(i, j);
            graph_file << "\n";
        }

        // BETWEENFACTOR2 x1 x2 x y theta Covariance(xx, xy, xtheta, yx, yy, ytheta, thetax, thetay, thetatheta)
        if (auto betweenFactor = boost::dynamic_pointer_cast<BetweenFactor<Pose2>>(factor))
        {
            Symbol pose_x1 = betweenFactor->keys().at(0);
            Symbol pose_x2 = betweenFactor->keys().at(1);

            Pose2 measurement = betweenFactor->measured();

            noiseModel::Base::shared_ptr noise_model;

            if (auto robust_noise = boost::dynamic_pointer_cast<noiseModel::Robust>(betweenFactor->noiseModel()))
            {
                noise_model = robust_noise->noise();
            }
            else
            {
                noise_model = betweenFactor->noiseModel();
            }

            Matrix3 covariance = boost::dynamic_pointer_cast<noiseModel::Diagonal>(noise_model)->covariance();

            graph_file << "BETWEENFACTOR2 " << pose_x1 << " " << pose_x2 << " " << measurement.x() << " " << measurement.y() << " " << measurement.theta();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    graph_file << " " << covariance(i, j);
            graph_file << "\n";
        }

        // BEARINGRANGEFACTOR2 x l bearing range Covariance(bearing, range)
        if (auto brFactor = boost::dynamic_pointer_cast<BearingRangeFactor<Pose2, Point2>>(factor))
        {
            Symbol pose_x = brFactor->keys().at(0);
            Symbol landmark_l = brFactor->keys().at(1);
            double bearing = brFactor->measured().bearing().theta();
            double range = brFactor->measured().range();

            noiseModel::Base::shared_ptr noise_model;

            if (auto robust_noise = boost::dynamic_pointer_cast<noiseModel::Robust>(brFactor->noiseModel()))
            {
                noise_model = robust_noise->noise();
            }
            else
            {
                noise_model = brFactor->noiseModel();
            }

            Matrix2 covariance = boost::dynamic_pointer_cast<noiseModel::Diagonal>(noise_model)->covariance();

            graph_file << "BEARINGRANGEFACTOR2 " << pose_x << " " << landmark_l << " " << bearing << " " << range;
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                    graph_file << " " << covariance(i, j);
            graph_file << "\n";
        }
    }
}

