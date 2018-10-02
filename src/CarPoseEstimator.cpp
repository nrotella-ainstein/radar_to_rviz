#include "CarPoseEstimator.h"

// Acura ILX 2016 parameters from openpilot interface:
const double CarPoseEstimator::wheelbase = 2.67;
const double CarPoseEstimator::steering_ratio = 18.61;


const char* radar_names[] =
  {
    "KANZA_TARGETS",
    "TIPI_79_FL_TARGETS",
    "TIPI_79_FR_TARGETS",
    "TIPI_79_RL_TARGETS",
    "TIPI_79_RR_TARGETS",
  };

CarPoseEstimator::CarPoseEstimator( void )
{
  pose_tf_.header.frame_id = "base_link";
  pose_tf_.child_frame_id = "chassis";
  
  // Initialize car position:
  pose_tf_.transform.translation.x = 0.0;
  pose_tf_.transform.translation.y = 0.0;
  pose_tf_.transform.translation.z = 0.0;
  
  // Initialize car orientation:
  tf2::Quaternion q;
  q.setRPY( 0.0, 0.0, 0.0 );
  pose_tf_.transform.rotation.x = q.x();
  pose_tf_.transform.rotation.y = q.y();
  pose_tf_.transform.rotation.z = q.z();
  pose_tf_.transform.rotation.w = q.w();

  // Subscribe to the car_speed topic with the updatePose callback:
  sub_car_speed_ = node_handle_.subscribe( "car_data", 10, &CarPoseEstimator::updatePose, this );

  // // Subscribe to the radar target topics with the updateMap callback:
  // for( int i = 0; i < N_RADARS; ++i )
  //   {
  //     sub_radar_targets_[i] = node_handle_.subscribe( radar_names[i], 10, &CarPoseEstimator::updateMap, this );
  //   }

  // // Resize the point cloud and advertise it:
  // radar_data_.points.resize( 1000000 );
  // radar_data_.header.frame_id = "base_link";
  // radar_data_.scale.x = 0.1;
  // radar_data_.scale.y = 0.1;
  // radar_data_.scale.z = 0.0;
  // radar_data_.color.r = 1.0;
  // radar_data_.color.g = 0.0;
  // radar_data_.color.b = 0.0;
  // radar_data_.color.a = 1.0;

  // radar_data_.type = visualization_msgs::Marker::POINTS;
  // radar_data_.action = visualization_msgs::Marker::ADD;

  // pc_ind_ = 0;
  
  // pub_radar_data_ = node_handle_.advertise<visualization_msgs::Marker>( "radar_data", 10 );
  
  time_prev_ = ros::Time::now();
}

void CarPoseEstimator::updatePose( const radar_to_rviz::CarData &msg )
{
  ros::Time time_now = ros::Time::now();
  pose_tf_.header.stamp = time_now;
  dt_ = ( time_now - time_prev_ ).toSec();
  
  // Update orientation using speed and steering angle:
  double yaw_rate = ( msg.speed / CarPoseEstimator::wheelbase ) * tan( ( M_PI / 180.0 ) * ( msg.steer_angle / CarPoseEstimator::steering_ratio ) );
  yaw_ += dt_ * yaw_rate;
  tf2::Quaternion q;
  q.setRPY( 0.0, 0.0, yaw_ + ( 0.5 * M_PI ) );
  pose_tf_.transform.rotation.x = q.x();
  pose_tf_.transform.rotation.y = q.y();
  pose_tf_.transform.rotation.z = q.z();
  pose_tf_.transform.rotation.w = q.w();
  
  // Update position using speed and updated yaw:
  pose_tf_.transform.translation.x += dt_ * ( msg.speed * cos( yaw_ ) );
  pose_tf_.transform.translation.y += dt_ * ( msg.speed * sin( yaw_ ) );

  // Send the updated car pose transform:
  br_pose_tf_.sendTransform( pose_tf_ );

  time_prev_ = time_now;
}

void CarPoseEstimator::updateMap( const visualization_msgs::Marker &msg )
{
  // for( auto it = msg.points.begin(); it != msg.points.end(); ++it)
  //   {
  //     radar_data_.points[pc_ind_].x = it->x;
  //     radar_data_.points[pc_ind_].y = it->y;
  //     radar_data_.points[pc_ind_].z = it->z;
  //     ++pc_ind_;
  //   }

  // radar_data_.header.stamp = ros::Time::now();
  // pub_radar_data_.publish( radar_data_ );
}
