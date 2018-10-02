#ifndef CAR_POSE_ESTIMATOR_H_
#define CAR_POSE_ESTIMATOR_H_

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <radar_to_rviz/CarData.h>
#include <cmath>

enum RadarType
  {
    KANZA = 0,
    TIPI_79_FL,
    TIPI_79_FR,
    TIPI_79_RL,
    TIPI_79_RR,
    N_RADARS
  };

class CarPoseEstimator
{
public:
  CarPoseEstimator();
  ~CarPoseEstimator(){}

  void updatePose( const radar_to_rviz::CarData &msg );
  void updateMap( const visualization_msgs::Marker &msg );

  static const double wheelbase;
  static const double steering_ratio;
  
private:
  double yaw_;
  geometry_msgs::TransformStamped pose_tf_;
  ros::Time time_prev_;
  double dt_;
  
  ros::NodeHandle node_handle_;
  ros::Subscriber sub_car_speed_;
  std::array<ros::Subscriber, N_RADARS> sub_radar_targets_;
  visualization_msgs::Marker radar_data_;
  int pc_ind_;
  
  tf2_ros::TransformBroadcaster br_pose_tf_;
  ros::Publisher pub_radar_data_;
};

#endif
