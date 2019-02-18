
#include "frontier_exploration.h"

namespace frontier{

FrontierExploration::FrontierExploration() : global_planner_actionlib_client_("global_planner_node_action", true),
                                             local_planner_actionlib_client_("local_planner_node_action", true){

  nh_.param<bool>("enable_visualization", enable_visualization_, true);
  nh_.param<std::string>("base_frame", base_frame_, "base_link");
  nh_.param<std::string>("global_frame", map_frame_, "map");

  updated_ = false;
  new_path_ = false;
  planner_state_ = NodeState::IDLE;

  tf_listener_ptr_ = std::make_unique<tf::TransformListener>();

  map_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::OccupancyGrid>>(nh_, "map", 5);
  map_filter_ = std::make_unique<tf::MessageFilter<nav_msgs::OccupancyGrid>>(*map_sub_,
                                                                             *tf_listener_ptr_,
                                                                             map_frame_,
                                                                             100);
  map_filter_->registerCallback(boost::bind(&FrontierExploration::MapCallback, this, _1));

  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 1);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("prior_goal", 1);
  current_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_pose", 1);

  exploration_thread_ = std::thread(std::bind(&FrontierExploration::ExplorationThread, this));

}

void FrontierExploration::ExplorationThread() {
  ros::Rate rate(20);
  local_planner_actionlib_client_.waitForServer();
  ROS_INFO("Local planner module has been connected!");
  global_planner_actionlib_client_.waitForServer();
  ROS_INFO("Global planner module has been connected!");
  ros::Time current_time = ros::Time::now();

  while (ros::ok() && running_){
    if(updated_) {
      if (planner_state_ == NodeState::IDLE || planner_state_ == NodeState::FAILURE) {
        std::lock_guard<std::mutex> guard(mutex_);
        if(exploration_goals_.empty()){
          continue;
        }

        GetPoseFromTf(base_frame_, map_frame_, ros::Time(0), robot_pose_);

        std::sort(exploration_goals_.begin(),
                  exploration_goals_.end(),
                  [this](const FrontierPoint &pose_goal_1, const FrontierPoint &pose_goal_2) {
                    auto distance1 = std::abs(pose_goal_1.x() - this->robot_pose_.pose.position.x)
                        + std::abs(pose_goal_1.y() - this->robot_pose_.pose.position.y);
                    auto distance2 = std::abs(pose_goal_2.x() - this->robot_pose_.pose.position.x)
                        + std::abs(pose_goal_2.y() - this->robot_pose_.pose.position.y);
                    return distance1 < distance2;
                  });

        current_goal_ = exploration_goals_.at(0);
        current_goal_.set_yaw(robot_pose_);

        if(std::find(failed_goals_.begin(), failed_goals_.end(), current_goal_) == failed_goals_.end()) {

          if(enable_visualization_) {
            PublishMarkers();
          }

          global_planner_goal_.goal = current_goal_.ToPoseStamped(ros::Time::now(), map_frame_);
          exploration_goals_.erase(exploration_goals_.begin());
          current_pose_pub_.publish(robot_pose_);

          SendGoalToGlobalPlanner(global_planner_goal_);
          no_goal_time = 0;
          current_time = ros::Time::now();
        } else{
          ROS_WARN("Invalid frontiers! Skip!");
          exploration_goals_.erase(exploration_goals_.begin());
          if(enable_visualization_) {
            PublishMarkers();
          }
          auto spend_time = ros::Time::now() - current_time;
          no_goal_time = spend_time.toSec();
          if(no_goal_time > timeout){
            running_ = false;
          }
        }
      }
      if(new_path_){
        local_planner_actionlib_client_.sendGoal(local_planner_goal_,
                                                 boost::bind(&FrontierExploration::LocalPlannerDoneCallback, this, _1, _2),
                                                 actionlib::SimpleActionClient<roborts_msgs::LocalPlannerAction>::SimpleActiveCallback(),
                                                 boost::bind(&FrontierExploration::LocalPlannerFeedbackCallback, this, _1));
        new_path_ = false;
      }
    }
    rate.sleep();
  }
  ROS_INFO("No more valid frontiers! End frontier search!");
}

void FrontierExploration::SendGoalToGlobalPlanner(const roborts_msgs::GlobalPlannerGoal &goal) {
  goal_pub_.publish(global_planner_goal_.goal);
  global_planner_actionlib_client_.sendGoal(global_planner_goal_,
                                            actionlib::SimpleActionClient<roborts_msgs::GlobalPlannerAction>::SimpleDoneCallback(),
                                            actionlib::SimpleActionClient<roborts_msgs::GlobalPlannerAction>::SimpleActiveCallback(),
                                            boost::bind(&FrontierExploration::GlobalPlannerFeedbackCallback,
                                                        this,
                                                        _1));
  planner_state_ = NodeState::RUNNING;
}

void FrontierExploration::GlobalPlannerFeedbackCallback(const roborts_msgs::GlobalPlannerFeedbackConstPtr& feedback){
  if (feedback->error_code != ErrorCode::OK) {
    ROS_WARN("Global planner: %s", feedback->error_msg.c_str());
    failed_goals_.insert(current_goal_);
    planner_state_ = NodeState::FAILURE;
  }
  if (!feedback->path.poses.empty()) {
    local_planner_goal_.route = feedback->path;
    new_path_=true;
  }
}

void FrontierExploration::LocalPlannerFeedbackCallback(const roborts_msgs::LocalPlannerFeedbackConstPtr &feedback) {
  if(feedback->error_code != ErrorCode::OK){
//    Rotation();
    planner_state_ = NodeState::FAILURE;
  }
}

void FrontierExploration::LocalPlannerDoneCallback(const actionlib::SimpleClientGoalState& state,  const roborts_msgs::LocalPlannerResultConstPtr& result){
//  ROS_INFO("Local planner %s !", state.toString().c_str());
//  Rotation();
  planner_state_ = NodeState::IDLE;
}


void FrontierExploration::MapCallback(nav_msgs::OccupancyGrid::ConstPtr map_msg_ptr) {
  std::lock_guard<std::mutex> lock_guard(mutex_);

  exploration_goals_.clear();
  UpdateFrontiers(map_msg_ptr, exploration_goals_);

  updated_ = !exploration_goals_.empty();


}

void FrontierExploration::UpdateFrontiers(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr, std::vector<FrontierPoint> &goal_vec) {

  auto width = map_msg_ptr->info.width;
  auto height = map_msg_ptr->info.height;
  auto resolution = map_msg_ptr->info.resolution;
  auto origin_x = map_msg_ptr->info.origin.position.x;
  auto origin_y = map_msg_ptr->info.origin.position.y;

  cv::Mat img(height, width, CV_8UC1);
//  cv::Mat out(height, width, CV_8UC1);

  for(int i = 0; i < height; i++){
    for(int j = 0; j < width; j++){
      switch (map_msg_ptr->data.at(i * width + j)) {
        case 100:
          img.at<uchar>(i,j) = 0;
          break;
        case 0:
          img.at<uchar>(i,j) = 255;
          break;
        case -1:
          img.at<uchar>(i,j) = 205;
          break;
        default:
          img.at<uchar>(i,j) = 205;
      }
    }
  }


////  cv::imshow("0", img);
////  cv::waitKey(0);
//  // 滤出所有占据部分（障碍物），存到out里
//  cv::inRange(img, cv::Scalar(0), cv::Scalar(1), out);
////  cv::imshow("1", out);
////  cv::waitKey(0);
//
//  // 原图做边缘检测
//  cv::Mat edges;
//  cv::Canny(img, edges, 0, 255);
////  cv::imshow("2", edges);
////  cv::waitKey(0);
//
//  std::vector<std::vector<cv::Point>> contours ;
//  cv::Mat hierarchy;
//  cv::findContours(out, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
////  cv::Mat img2 = cv::Mat(out);
//
//  cv::drawContours(out, contours, -1, (255,255,255), 5);
////  cv::imshow("3", out);
//  //  cv::waitKey(0);
//
//  cv::bitwise_not(out, out);
//  cv::Mat res;
//  cv::bitwise_and(out, edges, res);
////  cv::imshow("4", out);
////  cv::waitKey(0);
//
//  auto frontier = cv::Mat(res);
//  cv::findContours(frontier, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
//  cv::drawContours(frontier, contours, -1, (255,255,255), 2);
//  cv::findContours(frontier, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
////  cv::imshow("frontier", frontier);
////  cv::waitKey(0);

  std::vector<std::vector<cv::Point>> contours = FrontierDetection(img);

  if(!contours.empty()){
    int i = 0;
    for(const auto &it : contours){
      auto M = cv::moments(it);
      auto cx = static_cast<int>(M.m10/M.m00);
      auto cy = static_cast<int>(M.m01/M.m00);
      auto xr = cx * resolution + origin_x;
      auto yr = cy * resolution + origin_y;

      FrontierPoint goal(xr, yr, 0);
      goal_vec.push_back(goal);
      i++;
    }
  }
}

bool FrontierExploration::GetPoseFromTf(const std::string &target_frame,
                                     const std::string &source_frame,
                                     const ros::Time &timestamp,
                                     geometry_msgs::PoseStamped &pose)
{

  tf::StampedTransform transform;

  try {
    tf_listener_ptr_->lookupTransform(map_frame_, base_frame_, ros::Time(0), transform);
  } catch(tf::TransformException ex) {
    ROS_ERROR("-------> %s", ex.what());

  }

  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = map_frame_;

  pose.pose.position.x = transform.getOrigin().getX();
  pose.pose.position.y = transform.getOrigin().getY();
  pose.pose.position.z = transform.getOrigin().getZ();

  pose.pose.orientation.x = transform.getRotation().getX();
  pose.pose.orientation.y = transform.getRotation().getY();
  pose.pose.orientation.z = transform.getRotation().getZ();
  pose.pose.orientation.w = transform.getRotation().getW();

  return true;
}

void FrontierExploration::PublishMarkers() {
  for (auto &it : markers_.markers) {
    it.action = visualization_msgs::Marker::DELETE;
  }
  marker_pub_.publish(markers_);
  markers_.markers.clear();
  int i = 0;

  for (const auto &it : exploration_goals_) {
    markers_.markers.push_back(it.ToMarker(i));
    i++;
  }
  marker_pub_.publish(markers_);
}

} // namespace frontier