
#ifndef PROJECT_FRONTIER_POINT_H
#define PROJECT_FRONTIER_POINT_H

namespace frontier {

class FrontierPoint {
 public:
  FrontierPoint() : x_(0),
                    y_(0),
                    yaw_radians_(0),
                    distance_threshhold_(0.5) {};

  FrontierPoint(double x, double y, double yaw_radians) : x_(x),
                                                          y_(y),
                                                          yaw_radians_(yaw_radians),
                                                          distance_threshhold_(0.5) {};

  double x() const {
    return x_;
  }

  double y() const {
    return y_;
  }

  double yaw() const {
    return yaw_radians_;
  }

  void set_yaw(const geometry_msgs::PoseStamped &robot_pose) {
    yaw_radians_ = std::atan2(y_ - robot_pose.pose.position.y, x_ - robot_pose.pose.position.x);
  }

  geometry_msgs::PoseStamped ToPoseStamped(ros::Time time_stamp, const std::string &frame) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame;
    pose_stamped.header.stamp = time_stamp;
    pose_stamped.pose.position.x = x_;
    pose_stamped.pose.position.y = y_;
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_radians_);
    return pose_stamped;
  };

  visualization_msgs::Marker ToMarker(int id) const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 0.8;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(60);
    return marker;
  }

  // For std::set compare function
  bool operator<(const FrontierPoint &point_a) const {
    auto x_diff = std::abs(point_a.x_ - this->x_);
    auto y_diff = std::abs(point_a.y_ - this->y_);
    if (x_diff < distance_threshhold_ && y_diff < distance_threshhold_) {
      return false;
    }
    return true;
  };

  // For std::set find function
  bool operator==(const FrontierPoint &point_a) const {
    auto x_diff = std::abs(point_a.x_ - this->x_);
    auto y_diff = std::abs(point_a.y_ - this->y_);
    if (x_diff < distance_threshhold_ && y_diff < distance_threshhold_) {
      return true;
    }
    return false;
  };

 private:
  double x_ = 0;
  double y_ = 0;
  double yaw_radians_ = 0;
  double distance_threshhold_ = 0.5;

};

}

#endif //PROJECT_FRONTIER_POINT_H
