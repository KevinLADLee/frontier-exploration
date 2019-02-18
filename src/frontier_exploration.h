
#ifndef PROJECT_FRONTIER_EXPLORATION_H
#define PROJECT_FRONTIER_EXPLORATION_H

#include <memory>
#include <iostream>
#include <mutex>
#include <thread>
#include <set>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <tf/tf.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <roborts_msgs/GlobalPlannerAction.h>
#include "roborts_msgs/LocalPlannerAction.h"

#include "frontier_point.h"
#include "frontier_detection_opencv.h"

//#include <opencv2/opencv.hpp>

//Time Cost Test
//#include <boost/timer.hpp>
//#define TIMER_START(FUNC) boost::timer t_##FUNC;
//#define TIMER_END(FUNC) std::cout << "[" << #FUNC << "]" << "cost time: " << t_##FUNC.elapsed() << std::endl;

namespace frontier {

class FrontierExploration {
 public:
  FrontierExploration();

  void MapCallback(nav_msgs::OccupancyGrid::ConstPtr map_msg_ptr);

  void GlobalPlannerFeedbackCallback(const roborts_msgs::GlobalPlannerFeedbackConstPtr &feedback);

  void LocalPlannerDoneCallback(const actionlib::SimpleClientGoalState &state,
                                const roborts_msgs::LocalPlannerResultConstPtr &result);

  void LocalPlannerFeedbackCallback(const roborts_msgs::LocalPlannerFeedbackConstPtr &feedback);

  void SendGoalToGlobalPlanner(const roborts_msgs::GlobalPlannerGoal &goal);

 private:

  void UpdateFrontiers(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr,
                       std::vector<FrontierPoint> &goal_vec);

  void PublishMarkers();

  void ExplorationThread();

  bool GetPoseFromTf(const std::string &target_frame,
                     const std::string &source_frame,
                     const ros::Time &timestamp,
                     geometry_msgs::PoseStamped &pose);

 private:

  enum NodeState {
    IDLE,
    RUNNING,
    PAUSE,
    SUCCESS,
    FAILURE
  };
  enum ErrorCode {
    OK = 0,
    Error = 1
  };

 private:

  std::mutex mutex_;
  std::thread exploration_thread_;

  double timeout = 60;
  double no_goal_time = 0;

  ros::NodeHandle nh_;
//  ros::Subscriber map_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher current_pose_pub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::OccupancyGrid>> map_sub_;
  std::unique_ptr<tf::MessageFilter<nav_msgs::OccupancyGrid>> map_filter_;

  actionlib::SimpleActionClient<roborts_msgs::LocalPlannerAction> local_planner_actionlib_client_;
  actionlib::SimpleActionClient<roborts_msgs::GlobalPlannerAction> global_planner_actionlib_client_;
  roborts_msgs::LocalPlannerGoal local_planner_goal_;
  roborts_msgs::GlobalPlannerGoal global_planner_goal_;

  std::unique_ptr<tf::TransformListener> tf_listener_ptr_;

  bool enable_visualization_;
  bool running_ = true;
  std::string base_frame_;
  std::string map_frame_;

  std::vector<FrontierPoint> exploration_goals_;
  std::set<FrontierPoint> failed_goals_;
  FrontierPoint current_goal_;

  visualization_msgs::MarkerArray markers_;
  geometry_msgs::PoseStamped robot_pose_;
  bool updated_;
  bool new_path_;
  NodeState planner_state_;

};

} // namespace frontier

#endif //PROJECT_FRONTIER_EXPLORATION_H
