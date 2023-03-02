#include <msckf_mono/ros_interface.h>

namespace msckf_mono
{
  RosInterface::RosInterface(ros::NodeHandle nh) :
    nh_(nh),
    it_(nh_),
    imu_calibrated_(false),
    prev_imu_time_(0.0)
  {
    load_parameters();
    setup_track_handler();

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 100);
    track_image_pub_ = it_.advertise("track_overlay_image", 1);
    gt_track_pub_ = nh.advertise<nav_msgs::Odometry>("ground_truth_path", 100);
    imu_sub_ = nh_.subscribe("imu", 200, &RosInterface::imuCallback, this);
    image_sub_ = it_.subscribe("image_mono", 20,
                               &RosInterface::imageCallback, this);
    gt_sub_ = nh.subscribe("/vicon/firefly_sbx/firefly_sbx", 10, &RosInterface::gtCallback,this);
  }

  void RosInterface::gtCallback(const geometry_msgs::TransformStampedConstPtr& vicon_gt)
  {
    gt_pose.header.stamp = vicon_gt->header.stamp;
    gt_pose.header.frame_id = "map";
    
    Quaternion <float> vicon_q(vicon_gt->transform.rotation.w,vicon_gt->transform.rotation.x,
      vicon_gt->transform.rotation.y,vicon_gt->transform.rotation.z);
    
    T_vicon_world = Isometry3f::Identity();
    T_vicon_world.linear() = vicon_q.toRotationMatrix();

    Vector3<float> vicon_t;
    vicon_t << vicon_gt->transform.translation.x,vicon_gt->transform.translation.y,vicon_gt->transform.translation.z;
    T_vicon_world.translation() = vicon_t;

    Isometry3f T_IG;
    T_IG.linear() = init_imu_state_.q_IG.toRotationMatrix();
    T_IG.translation() << 0.0,0.0,0.0;

    // Isometry3f gt = T_IG * T_vicon_imu * T_world_vicon1 * T_vicon_world;
    Isometry3f gt = T_vicon_world;
    Quaternion<float> gt_q(gt.linear());

    gt_pose.pose.pose.position.x = gt.translation().x();
    gt_pose.pose.pose.position.y = gt.translation().y();
    gt_pose.pose.pose.position.z = gt.translation().z();
    gt_pose.pose.pose.orientation.x = gt_q.inverse().x();
    gt_pose.pose.pose.orientation.y = gt_q.inverse().y();
    gt_pose.pose.pose.orientation.z = gt_q.inverse().z();
    gt_pose.pose.pose.orientation.w = gt_q.inverse().w();

    return;
  }

  void RosInterface::imuCallback(const sensor_msgs::ImuConstPtr& imu)
  {
    double cur_imu_time = imu->header.stamp.toSec();
    if(prev_imu_time_ == 0.0){
      prev_imu_time_ = cur_imu_time;
      done_stand_still_time_ = cur_imu_time + stand_still_time_;
      return;
    }

    imuReading<float> current_imu;

    current_imu.a[0] = imu->linear_acceleration.x;
    current_imu.a[1] = imu->linear_acceleration.y;
    current_imu.a[2] = imu->linear_acceleration.z;

    current_imu.omega[0] = imu->angular_velocity.x;
    current_imu.omega[1] = imu->angular_velocity.y;
    current_imu.omega[2] = imu->angular_velocity.z;

    current_imu.dT = cur_imu_time - prev_imu_time_;

    imu_queue_.emplace_back(cur_imu_time, current_imu);

    prev_imu_time_ = cur_imu_time;
  }

  void RosInterface::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    double cur_image_time = msg->header.stamp.toSec();
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if(!imu_calibrated_){
      if(imu_queue_.size() % 100 == 0){
        ROS_INFO_STREAM("Has " << imu_queue_.size() << " readings");
      }

      if(can_initialize_imu()){
        initialize_imu();

        imu_calibrated_ = true;
        imu_queue_.clear();

        setup_msckf();
      }

      return;
    }

    std::vector<imuReading<float>> imu_since_prev_img;
    imu_since_prev_img.reserve(10);

    // get the first imu reading that belongs to the next image
    auto frame_end = std::find_if(imu_queue_.begin(), imu_queue_.end(),
        [&](const auto& x){return std::get<0>(x) > cur_image_time;});

    std::transform(imu_queue_.begin(), frame_end,
        std::back_inserter(imu_since_prev_img),
        [](auto& x){return std::get<1>(x);});

    imu_queue_.erase(imu_queue_.begin(), frame_end);

    for(auto& reading : imu_since_prev_img){
      msckf_.propagate(reading);

      Vector3<float> gyro_measurement = R_imu_cam_ * (reading.omega - init_imu_state_.b_g);
      track_handler_->add_gyro_reading(gyro_measurement);
    }

    track_handler_->set_current_image( cv_ptr->image, cur_image_time );

    std::vector<Vector2<float>,
      Eigen::aligned_allocator<Vector2<float>>> cur_features;
    corner_detector::IdVector cur_ids;
    track_handler_->tracked_features(cur_features, cur_ids);

    std::vector<Vector2<float>,
      Eigen::aligned_allocator<Vector2<float>>> new_features;
    corner_detector::IdVector new_ids;
    track_handler_->new_features(new_features, new_ids);

    msckf_.augmentState(state_k_, (float)cur_image_time);
    msckf_.update(cur_features, cur_ids);
    msckf_.addFeatures(new_features, new_ids);
    msckf_.marginalize();
    // msckf_.pruneRedundantStates();
    msckf_.pruneEmptyStates();

    publish_core(msg->header.stamp);
    publish_extra(msg->header.stamp);
  }

  void RosInterface::publish_core(const ros::Time& publish_time)
  {
    auto imu_state = msckf_.getImuState();

    nav_msgs::Odometry odom;
    odom.header.stamp = publish_time;
    odom.header.frame_id = "map";
    odom.twist.twist.linear.x = imu_state.v_I_G[0];
    odom.twist.twist.linear.y = imu_state.v_I_G[1];
    odom.twist.twist.linear.z = imu_state.v_I_G[2];

    odom.pose.pose.position.x = imu_state.p_I_G[0];
    odom.pose.pose.position.y = imu_state.p_I_G[1];
    odom.pose.pose.position.z = imu_state.p_I_G[2];
    Quaternion<float> q_out = imu_state.q_IG.inverse();
    odom.pose.pose.orientation.w = q_out.w();
    odom.pose.pose.orientation.x = q_out.x();
    odom.pose.pose.orientation.y = q_out.y();
    odom.pose.pose.orientation.z = q_out.z();

    odom_pub_.publish(odom);

    gt_track_pub_.publish(gt_pose);

  }

  void RosInterface::publish_extra(const ros::Time& publish_time)
  {
    if(track_image_pub_.getNumSubscribers() > 0){
      cv_bridge::CvImage out_img;
      out_img.header.frame_id = "cam0";
      out_img.header.stamp = publish_time;
      out_img.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
      out_img.image = track_handler_->get_track_image();
      track_image_pub_.publish(out_img.toImageMsg());
    }
  }

  bool RosInterface::can_initialize_imu()
  {
    if(imu_calibration_method_ == TimedStandStill){
      return prev_imu_time_ > done_stand_still_time_;
    }

    return false;
  }

  void RosInterface::initialize_imu()
  {
    Eigen::Vector3f accel_accum;
    Eigen::Vector3f gyro_accum;
    int num_readings = 0;

    accel_accum.setZero();
    gyro_accum.setZero();

    for(const auto& entry : imu_queue_){
      auto imu_time = std::get<0>(entry);
      auto imu_reading = std::get<1>(entry);

      accel_accum += imu_reading.a;
      gyro_accum += imu_reading.omega;
      num_readings++;
    }

    Eigen::Vector3f accel_mean = accel_accum / num_readings;
    Eigen::Vector3f gyro_mean = gyro_accum / num_readings;

    init_imu_state_.b_g = gyro_mean;
    init_imu_state_.g << 0.0, 0.0, -9.81;
    // init_imu_state_.q_IG = Quaternion<float>::FromTwoVectors(
    //     -init_imu_state_.g, accel_mean);
    Quaternion<float> q_vicon(T_vicon_world.linear());
    init_imu_state_.q_IG = q_vicon;

    init_imu_state_.b_a = init_imu_state_.q_IG*init_imu_state_.g + accel_mean;

    // init_imu_state_.p_I_G.setZero();
    init_imu_state_.p_I_G = T_vicon_world.translation();
    init_imu_state_.v_I_G.setZero();
    const auto q = init_imu_state_.q_IG;

    ROS_INFO_STREAM("\nInitial IMU State" <<
      "\n--p_I_G " << init_imu_state_.p_I_G.transpose() <<
      "\n--q_IG " << q.w() << "," << q.x() << "," << q.y() << "," << q.x() <<
      "\n--v_I_G " << init_imu_state_.v_I_G.transpose() <<
      "\n--b_a " << init_imu_state_.b_a.transpose() <<
      "\n--b_g " << init_imu_state_.b_g.transpose() <<
      "\n--g " << init_imu_state_.g.transpose());
    
    T_world_vicon1 = T_vicon_world.inverse();
  }

  void RosInterface::setup_track_handler()
  {
    track_handler_.reset( new corner_detector::TrackHandler(K_, dist_coeffs_, distortion_model_) );
    track_handler_->set_grid_size(n_grid_rows_, n_grid_cols_);
    track_handler_->set_ransac_threshold(ransac_threshold_);
  }

  void RosInterface::setup_msckf()
  {
    state_k_ = 0;
    msckf_.initialize(camera_, noise_params_, msckf_params_, init_imu_state_);
  }

  void RosInterface::load_parameters()
  {

    std::string kalibr_camera;
    nh_.getParam("kalibr_camera_name", kalibr_camera);

    nh_.getParam(kalibr_camera+"/camera_model", camera_model_);

    K_ = cv::Mat::eye(3,3,CV_32F);
    std::vector<float> intrinsics(4);
    nh_.getParam(kalibr_camera+"/intrinsics", intrinsics);
    K_.at<float>(0,0) = intrinsics[0];
    K_.at<float>(1,1) = intrinsics[1];
    K_.at<float>(0,2) = intrinsics[2];
    K_.at<float>(1,2) = intrinsics[3];

    nh_.getParam(kalibr_camera+"/distortion_model", distortion_model_);

    std::vector<float> distortion_coeffs(4);
    nh_.getParam(kalibr_camera+"/distortion_coeffs", distortion_coeffs);
    dist_coeffs_ = cv::Mat::zeros(distortion_coeffs.size(),1,CV_32F);
    dist_coeffs_.at<float>(0) = distortion_coeffs[0];
    dist_coeffs_.at<float>(1) = distortion_coeffs[1];
    dist_coeffs_.at<float>(2) = distortion_coeffs[2];
    dist_coeffs_.at<float>(3) = distortion_coeffs[3];

    XmlRpc::XmlRpcValue ros_param_list;
    nh_.getParam(kalibr_camera+"/T_cam_imu", ros_param_list);
    ROS_ASSERT(ros_param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    
    Matrix4<float> T_cam_imu;
    T_cam_imu << 0.0148655429818, -0.999880929698,   0.00414029679422, -0.021640145497, 0.999557249008,   0.0149672133247,  0.025715529948,   -0.064676986768
      ,-0.0257744366974,  0.00375618835797, 0.999660727178,    0.009810730590
      ,0.0,  0.0, 0.0,1.000000000000;
    
    // ROS_INFO_STREAM("Has " << ros_param_list.size() << " readings");
    // for (int32_t i = 0; i < ros_param_list.size(); ++i) 
    // {
    //   // ROS_ASSERT(ros_param_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    //   for(int32_t j=0; j<ros_param_list[i].size(); ++j){
    //     ROS_ASSERT(ros_param_list[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    //     T_cam_imu(i,j) = static_cast<double>(ros_param_list[i][j]);
    //   }
    // }

    R_cam_imu_ =  T_cam_imu.block<3,3>(0,0);//T_bs
    p_cam_imu_ =  T_cam_imu.block<3,1>(0,3);

    R_imu_cam_ = R_cam_imu_.transpose();
    p_imu_cam_ = R_imu_cam_ * (-1. * p_cam_imu_);

    T_world_vicon1 = Isometry3f::Identity();

    // const Eigen::AngleAxisf roll_angle(0, Eigen::Vector3f::UnitX());
    // const Eigen::AngleAxisf pitch_angle(-1.5707, Eigen::Vector3f::UnitY());// circle:-
    // const Eigen::AngleAxisf yaw_angle(3.14159, Eigen::Vector3f::UnitZ());
    // Eigen::Quaternionf q = yaw_angle * pitch_angle * roll_angle;
    // T_vicon_imu.linear() = q.toRotationMatrix();
    // T_vicon_imu.translation() << 0.0,0.0,0.0;


    T_vicon_imu.linear() << 0.33638, -0.01749,  0.94156,  
         -0.02078, -0.99972, -0.01114, 
          0.94150, -0.01582, -0.33665;

    T_vicon_imu.translation() << 0.06901,-0.02781,-0.12395;
    
    // T_vicon_imu = T_vicon_imu.inverse();
    // setup camera parameters
    camera_.f_u = intrinsics[0];
    camera_.f_v = intrinsics[1];
    camera_.c_u = intrinsics[2];
    camera_.c_v = intrinsics[3];

    camera_.q_CI = Quaternion<float>(R_cam_imu_).inverse(); // TODO please check it 
    camera_.p_C_I = p_cam_imu_;

    // Feature tracking parameteres
    nh_.param<int>("n_grid_rows", n_grid_rows_, 8);
    nh_.param<int>("n_grid_cols", n_grid_cols_, 8);

    float ransac_threshold_;
    nh_.param<float>("ransac_threshold_", ransac_threshold_, 0.000002);

    // MSCKF Parameters
    float feature_cov;
    nh_.param<float>("feature_covariance", feature_cov, 7);

    Eigen::Matrix<float,12,1> Q_imu_vars;
    float w_var, dbg_var, a_var, dba_var;
    nh_.param<float>("imu_vars/w_var", w_var, 1e-5);
    nh_.param<float>("imu_vars/dbg_var", dbg_var, 3.6733e-5);
    nh_.param<float>("imu_vars/a_var", a_var, 1e-3);
    nh_.param<float>("imu_vars/dba_var", dba_var, 7e-4);
    Q_imu_vars << w_var, 	w_var, 	w_var,
                  dbg_var,dbg_var,dbg_var,
                  a_var,	a_var,	a_var,
                  dba_var,dba_var,dba_var;

    Eigen::Matrix<float,15,1> IMUCovar_vars;
    float q_var_init, bg_var_init, v_var_init, ba_var_init, p_var_init;
    nh_.param<float>("imu_covars/q_var_init", q_var_init, 1e-5);
    nh_.param<float>("imu_covars/bg_var_init", bg_var_init, 1e-2);
    nh_.param<float>("imu_covars/v_var_init", v_var_init, 1e-2);
    nh_.param<float>("imu_covars/ba_var_init", ba_var_init, 1e-2);
    nh_.param<float>("imu_covars/p_var_init", p_var_init, 1e-12);
    IMUCovar_vars << q_var_init, q_var_init, q_var_init,
                     bg_var_init,bg_var_init,bg_var_init,
                     v_var_init, v_var_init, v_var_init,
                     ba_var_init,ba_var_init,ba_var_init,
                     p_var_init, p_var_init, p_var_init;

    // Setup noise parameters
    noise_params_.initial_imu_covar = IMUCovar_vars.asDiagonal();
    noise_params_.Q_imu = Q_imu_vars.asDiagonal();
    noise_params_.u_var_prime = pow(feature_cov/camera_.f_u,2);
    noise_params_.v_var_prime = pow(feature_cov/camera_.f_v,2);

    nh_.param<float>("max_gn_cost_norm", msckf_params_.max_gn_cost_norm, 11);
    msckf_params_.max_gn_cost_norm = pow(msckf_params_.max_gn_cost_norm/camera_.f_u, 2);
    nh_.param<float>("translation_threshold", msckf_params_.translation_threshold, 0.05);
    nh_.param<float>("min_rcond", msckf_params_.min_rcond, 3e-12);
    nh_.param<float>("keyframe_transl_dist", msckf_params_.redundancy_angle_thresh, 0.005);
    nh_.param<float>("keyframe_rot_dist", msckf_params_.redundancy_distance_thresh, 0.05);
    nh_.param<int>("max_track_length", msckf_params_.max_track_length, 1000);
    nh_.param<int>("min_track_length", msckf_params_.min_track_length, 3);
    nh_.param<int>("max_cam_states", msckf_params_.max_cam_states, 20);

    // Load calibration time
    int method;
    nh_.param<int>("imu_initialization_method", method, 0);
    if(method == 0){
      imu_calibration_method_ = TimedStandStill;
    }
    nh_.param<double>("stand_still_time", stand_still_time_, 8.0);

    ROS_INFO_STREAM("Loaded " << kalibr_camera);
    ROS_INFO_STREAM("-Intrinsics " << intrinsics[0] << ", "
                                   << intrinsics[1] << ", "
                                   << intrinsics[2] << ", "
                                   << intrinsics[3] );
    ROS_INFO_STREAM("-Distortion " << distortion_coeffs[0] << ", "
                                   << distortion_coeffs[1] << ", "
                                   << distortion_coeffs[2] << ", "
                                   << distortion_coeffs[3] );
    const auto q_CI = camera_.q_CI;
    ROS_INFO_STREAM("-q_CI \n" << q_CI.x() << "," << q_CI.y() << "," << q_CI.z() << "," << q_CI.w());
    ROS_INFO_STREAM("-p_C_I \n" << camera_.p_C_I.transpose());


//test:
    // std::string kalibr_camera;
    // kalibr_camera = "cam0";
    // camera_model_ = "pinhole";
    
    // std::vector<float> intrinsics;
    // // intrinsics.reserve(4);
    // intrinsics.push_back(458.654);
    // intrinsics.push_back(457.296);
    // intrinsics.push_back(367.215);
    // intrinsics.push_back(248.375);
    
    // K_ = cv::Mat::eye(3,3,CV_32F);
    // K_.at<float>(0,0) = intrinsics[0];
    // K_.at<float>(1,1) = intrinsics[1];
    // K_.at<float>(0,2) = intrinsics[2];
    // K_.at<float>(1,2) = intrinsics[3];

    // distortion_model_ = "radtan";

    // std::vector<float> distortion_coeffs;
    // // distortion_coeffs.reserve(4);
    // distortion_coeffs.push_back(-0.28340811);
    // distortion_coeffs.push_back(0.07395907);
    // distortion_coeffs.push_back(0.00019359);
    // distortion_coeffs.push_back(1.76187114e-05);

    // dist_coeffs_ = cv::Mat::zeros(distortion_coeffs.size(),1,CV_32F);
    // dist_coeffs_.at<float>(0) = distortion_coeffs[0];
    // dist_coeffs_.at<float>(1) = distortion_coeffs[1];
    // dist_coeffs_.at<float>(2) = distortion_coeffs[2];
    // dist_coeffs_.at<float>(3) = distortion_coeffs[3];

    // Matrix4<float> T_cam_imu;
    // T_cam_imu << 0.0148655429818, -0.999880929698,   0.00414029679422, -0.021640145497, 0.999557249008,   0.0149672133247,  0.025715529948,   -0.064676986768
    //   ,-0.0257744366974,  0.00375618835797, 0.999660727178,    0.009810730590
    //   ,0.0,  0.0, 0.0,1.000000000000;

    // R_cam_imu_ =  T_cam_imu.block<3,3>(0,0);
    // p_cam_imu_ =  T_cam_imu.block<3,1>(0,3);

    // R_imu_cam_ = R_cam_imu_.transpose();
    // p_imu_cam_ = R_imu_cam_ * (-1. * p_cam_imu_);

    // // setup camera parameters
    // camera_.f_u = intrinsics[0];
    // camera_.f_v = intrinsics[1];
    // camera_.c_u = intrinsics[2];
    // camera_.c_v = intrinsics[3];

    // camera_.q_CI = Quaternion<float>(R_cam_imu_).inverse(); // TODO please check it 
    // camera_.p_C_I = p_cam_imu_;

    // // Feature tracking parameteres
    // int n_grid_rows_ = 8;
    // int n_grid_cols_ = 8;

    // float ransac_threshold_ = 0.000002;

    // // MSCKF Parameters
    // float feature_cov = 7;

    // Eigen::Matrix<float,12,1> Q_imu_vars;
    // float w_var, dbg_var, a_var, dba_var;
    // w_var =  1e-5;
    // dbg_var = 3.6733e-5;
    // a_var = 1e-3;
    // dba_var =  7e-4;
    // Q_imu_vars << w_var, 	w_var, 	w_var,
    //               dbg_var,dbg_var,dbg_var,
    //               a_var,	a_var,	a_var,
    //               dba_var,dba_var,dba_var;

    // Eigen::Matrix<float,15,1> IMUCovar_vars;
    // float q_var_init, bg_var_init, v_var_init, ba_var_init, p_var_init;
    // q_var_init =  1e-5;
    // bg_var_init =  1e-2;
    // v_var_init =  1e-2;
    // ba_var_init =  1e-2;
    // p_var_init =  1e-12;
    // IMUCovar_vars << q_var_init, q_var_init, q_var_init,
    //                  bg_var_init,bg_var_init,bg_var_init,
    //                  v_var_init, v_var_init, v_var_init,
    //                  ba_var_init,ba_var_init,ba_var_init,
    //                  p_var_init, p_var_init, p_var_init;

    // // Setup noise parameters
    // noise_params_.initial_imu_covar = IMUCovar_vars.asDiagonal();
    // noise_params_.Q_imu = Q_imu_vars.asDiagonal();
    // noise_params_.u_var_prime = pow(feature_cov/camera_.f_u,2);
    // noise_params_.v_var_prime = pow(feature_cov/camera_.f_v,2);

    // msckf_params_.max_gn_cost_norm = 11;
    // // msckf_params_.max_gn_cost_norm = pow(msckf_params_.max_gn_cost_norm/camera_.f_u, 2);
    // msckf_params_.translation_threshold =  0.05;
    // msckf_params_.min_rcond = 3e-12;
    // msckf_params_.redundancy_angle_thresh = 0.005;
    // msckf_params_.redundancy_distance_thresh = 0.05;
    // msckf_params_.max_track_length =  1000;
    // msckf_params_.min_track_length =  3;
    // msckf_params_.max_cam_states = 20;

    // // Load calibration time
    // int method = 0;
    // if(method == 0){
    //   imu_calibration_method_ = TimedStandStill;
    // }
    // stand_still_time_ =  1.0;

    // ROS_INFO_STREAM("Loaded " << kalibr_camera);
    // ROS_INFO_STREAM("-Intrinsics " << intrinsics[0] << ", "
    //                                << intrinsics[1] << ", "
    //                                << intrinsics[2] << ", "
    //                                << intrinsics[3] );

    // ROS_INFO_STREAM("-Distortion " << distortion_coeffs[0] << ", "
    //                                << distortion_coeffs[1] << ", "
    //                                << distortion_coeffs[2] << ", "
    //                                << distortion_coeffs[3] );
    // const auto q_CI = camera_.q_CI;
    // ROS_INFO_STREAM("-q_CI \n" << q_CI.x() << "," << q_CI.y() << "," << q_CI.z() << "," << q_CI.w());
    // ROS_INFO_STREAM("-p_C_I \n" << camera_.p_C_I.transpose());
  }

}
