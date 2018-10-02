/*
  Copyright <2018> <Ainstein, Inc.>

  Redistribution and use in source and binary forms, with or without modification, are permitted 
  provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this list of 
  conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this list of 
  conditions and the following disclaimer in the documentation and/or other materials provided 
  with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors may be used to 
  endorse or promote products derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR 
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND 
  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
  OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "CarDemoRadarNode.h"

CarDemoRadarNode::CarDemoRadarNode( void )
{
  // Set up the subscriber to receive CAN frames:
  sub_can_msg_ = node_handle_.subscribe( "received_messages", 10, &CarDemoRadarNode::canMsgCallback, this );

  // Set up the subscriber to receive speed messages from the PANDA:
  sub_speed_msg_ = node_handle_.subscribe( "car_speed", 10, &CarDemoRadarNode::speedMsgCallback, this );

  // Set up the publisher for sending commands to the radar:
  pub_radar_ = node_handle_.advertise<can_msgs::Frame>( "sent_messages", 10 );

  // Set up the publishers for sending visualization messages for all radars:
  pub_target_marker_[KANZA] = node_handle_.advertise<visualization_msgs::Marker>( "KANZA_TARGETS", 10 );
  pub_target_marker_[TIPI_79_FL] = node_handle_.advertise<visualization_msgs::Marker>( "TIPI_79_FL_TARGETS", 10 );
  pub_target_marker_[TIPI_79_FR] = node_handle_.advertise<visualization_msgs::Marker>( "TIPI_79_FR_TARGETS", 10 );
  pub_target_marker_[TIPI_79_RL] = node_handle_.advertise<visualization_msgs::Marker>( "TIPI_79_RL_TARGETS", 10 );
  pub_target_marker_[TIPI_79_RR] = node_handle_.advertise<visualization_msgs::Marker>( "TIPI_79_RR_TARGETS", 10 );
  pub_BSD_marker_[TIPI_79_RL_BSD] = node_handle_.advertise<visualization_msgs::Marker>( "TIPI_79_RL_BSD", 10 );
  pub_BSD_marker_[TIPI_79_RR_BSD] = node_handle_.advertise<visualization_msgs::Marker>( "TIPI_79_RR_BSD", 10 );

  // Initialize tracked marker info common to all radars:
  for( auto it = marker_tracked_.begin(); it != marker_tracked_.end(); ++it )
    {
      it->ns              = "tracked";
      it->type            = visualization_msgs::Marker::POINTS;
      it->action          = visualization_msgs::Marker::ADD;
      node_handle_.getParam( "marker_scale_tracked", it->scale.x );
      node_handle_.getParam( "marker_scale_tracked", it->scale.y );
      it->scale.z         = 0.0;
      it->lifetime        = ros::Duration( 0.1 );
    }

  // Initialize raw marker info common to all radars:
  for( auto it = marker_raw_.begin(); it != marker_raw_.end(); ++it )
    {
      it->ns              = "raw";
      it->type            = visualization_msgs::Marker::POINTS;
      it->action          = visualization_msgs::Marker::ADD;
      node_handle_.getParam( "marker_scale_raw", it->scale.x );
      node_handle_.getParam( "marker_scale_raw", it->scale.y );
      it->scale.z         = 0.0;
      it->lifetime        = ros::Duration( 0.1 );
    }

  // Initialize BSD marker info common to all radars:
  for( auto it = BSD_marker_.begin(); it != BSD_marker_.end(); ++it )
    {
      it->ns              = "bsd";
      it->type            = visualization_msgs::Marker::POINTS;
      it->action          = visualization_msgs::Marker::ADD;
      node_handle_.getParam( "marker_scale_bsd", it->scale.x );
      node_handle_.getParam( "marker_scale_bsd", it->scale.y );
      it->scale.z         = 0.0;
      it->lifetime        = ros::Duration( 0.1 );
    }
    
  // Launch the "keep alive" thread, if specified in configuration YAML file:
  bool use_keep_alive_thread;
  node_handle_.getParam( "use_keep_alive_thread", use_keep_alive_thread );
  if( use_keep_alive_thread )
    {
      keep_alive_thread_ = std::unique_ptr<std::thread>( new std::thread( &CarDemoRadarNode::keepAliveLoop, this ) );
      keep_alive_mutex_.lock();
      keep_alive_thread_is_running_ = true;
      keep_alive_mutex_.unlock();
    }

  // Determine whether or not to publish markers for radars:
  node_handle_.getParam( "use_radar_front_center_tracked", use_radar_front_center_tracked_ );
  node_handle_.getParam( "use_radar_front_center_raw", use_radar_front_center_raw_ );

  node_handle_.getParam( "use_radar_front_left_tracked", use_radar_front_left_tracked_ );
  node_handle_.getParam( "use_radar_front_left_raw", use_radar_front_left_raw_ );

  node_handle_.getParam( "use_radar_front_right_tracked", use_radar_front_right_tracked_ );
  node_handle_.getParam( "use_radar_front_right_raw", use_radar_front_right_raw_ );

  node_handle_.getParam( "use_radar_rear_left_tracked", use_radar_rear_left_tracked_ );
  node_handle_.getParam( "use_radar_rear_left_raw", use_radar_rear_left_raw_ );
  node_handle_.getParam( "use_radar_rear_left_bsd", use_radar_rear_left_bsd_ );

  node_handle_.getParam( "use_radar_rear_right_tracked", use_radar_rear_right_tracked_ );
  node_handle_.getParam( "use_radar_rear_right_raw", use_radar_rear_right_raw_ );
  node_handle_.getParam( "use_radar_rear_right_bsd", use_radar_rear_right_bsd_ );
        
  // Sleep for a little to make sure messages are being advertised before we start sending:
  ros::Duration(1).sleep();
    
}

  
CarDemoRadarNode::~CarDemoRadarNode( void )
{
  // Shut down the "keep alive" thread:    
  keep_alive_mutex_.lock();
  keep_alive_thread_is_running_ = false;
  keep_alive_mutex_.unlock();

  keep_alive_thread_->join();

  // Shut down the radar when ROS node is destroyed (not working?):
  stopRadar();
 
  std::cout << "Destructor finished." << std::endl;
}

  
void CarDemoRadarNode::keepAliveLoop( void )
{
  // Sleep for a bit before launching the thread to make sure publisher is set up:
  ros::Duration( 3.0 ).sleep();
    
  // Enter the main keep alive loop:
  bool running = true;
  while( running )
    {
      // Send a message to start all radars, then sleep for a period:
      startRadar();
      ros::Duration( 1.0 ).sleep();

      // Check whether the loop should still be running:
      keep_alive_mutex_.lock();
      running = keep_alive_thread_is_running_;
      keep_alive_mutex_.unlock();
    }
}
  
  
void CarDemoRadarNode::startRadar( void )
{
  can_msgs::Frame can_frame;
  can_frame.header.frame_id = "0";
  can_frame.header.stamp    = ros::Time::now();
  can_frame.is_rtr          = false;
  can_frame.is_extended     = false;
  can_frame.is_error        = false;
  can_frame.dlc             = 8;
  can_frame.id              = RADAR_START_STOP;
  can_frame.data[0]         = RADAR_START;
  can_frame.data[1]         = 0xff;
  can_frame.data[2]         = 0xff;
  can_frame.data[3]         = 0xff;
  can_frame.data[4]         = 0xff;
  can_frame.data[5]         = 0xff;
  can_frame.data[6]         = 0xff;
  can_frame.data[7]         = 0xff;

  pub_radar_.publish( can_frame );
}

void CarDemoRadarNode::stopRadar( void )
{
  can_msgs::Frame can_frame;
  can_frame.header.frame_id = "0";
  can_frame.header.stamp    = ros::Time::now();
  can_frame.is_rtr          = false;
  can_frame.is_extended     = false;
  can_frame.is_error        = false;
  can_frame.dlc             = 8;
  can_frame.id              = RADAR_START_STOP;
  can_frame.data[0]         = RADAR_STOP;
  can_frame.data[1]         = 0x00;
  can_frame.data[2]         = 0xff;
  can_frame.data[3]         = 0xff;
  can_frame.data[4]         = 0xff;
  can_frame.data[5]         = 0xff;
  can_frame.data[6]         = 0xff;
  can_frame.data[7]         = 0xff;

  pub_radar_.publish( can_frame );
}


void CarDemoRadarNode::sendSpeedToRadar( double car_speed )
{
  // Convert the floating point car speed to 
  uint16_t car_speed_int = (uint16_t)( car_speed / 0.06875 );
  can_msgs::Frame can_frame;
  can_frame.header.frame_id = "0";
  can_frame.header.stamp    = ros::Time::now();
  can_frame.is_rtr          = false;
  can_frame.is_extended     = false;
  can_frame.is_error        = false;
  can_frame.dlc             = 8;
  can_frame.id              = RADAR_SEND_SPEED;
  can_frame.data[0]         = ( car_speed_int & 0x00ff ); // first 8 bits
  can_frame.data[1]         = ( ( ( car_speed_int & 0x0f00 ) >> 8) | ( 1UL << 7 ) ); // bits 9-12, and set effective pos bit to 1
  can_frame.data[2]         = 0xff;
  can_frame.data[3]         = 0xff;
  can_frame.data[4]         = 0xff;
  can_frame.data[5]         = 0xff;
  can_frame.data[6]         = 0xff;
  can_frame.data[7]         = 0xff;
  ROS_INFO( "Sending speed %d encoded as %02x %02x", car_speed_int, can_frame.data[0], can_frame.data[1] );

  pub_radar_.publish( can_frame );
}

void CarDemoRadarNode::speedMsgCallback( const std_msgs::Float64& speed_msg )
{
  // This will only be called if something publishes to "car_speed" topic:
  sendSpeedToRadar( (double)( speed_msg.data ) );
}
  
void CarDemoRadarNode::canMsgCallback( const can_msgs::Frame& can_msg )
{
  // Parse out start radar response messages:
  if( can_msg.id >= RADAR_START_STOP_KANZA_RET && can_msg.id <= RADAR_START_STOP_TIPI_79_RR_RET )
    {
      // Check whether the response is for a start or stop radar message:
      uint8_t start_stop_byte = can_msg.data[0];
      switch( can_msg.id )
	{
	case RADAR_START_STOP_KANZA_RET:
	  switch( start_stop_byte )
	    {
	    case RADAR_START:
	      ROS_INFO( "received radar start from KANZA" );
	      break;

	    case RADAR_STOP:
	      ROS_INFO( "received radar stop from KANZA" );
	      break;

	    default:
	      ROS_ERROR( "received unknown radar start/stop from KANZA" );
	      break;
	    }
	  break;

	case RADAR_START_STOP_TIPI_79_FL_RET:
	  switch( start_stop_byte )
	    {
	    case RADAR_START:
	      ROS_INFO( "received radar start from FRONT LEFT" );
	      break;

	    case RADAR_STOP:
	      ROS_INFO( "received radar stop from FRONT LEFT" );
	      break;

	    default:
	      ROS_ERROR( "received unknown radar start/stop from FRONT LEFT" );
	      break;
	    }
	  break;
	    
	case RADAR_START_STOP_TIPI_79_FR_RET:
	  switch( start_stop_byte )
	    {
	    case RADAR_START:
	      ROS_INFO( "received radar start from FRONT RIGHT" );
	      break;

	    case RADAR_STOP:
	      ROS_INFO( "received radar stop from FRONT RIGHT" );
	      break;

	    default:
	      ROS_ERROR( "received unknown radar start/stop from FRONT RIGHT" );
	      break;
	    }
	  break;
	    
	case RADAR_START_STOP_TIPI_79_RL_RET:
	  switch( start_stop_byte )
	    {
	    case RADAR_START:
	      ROS_INFO( "received radar start from REAR LEFT" );
	      break;

	    case RADAR_STOP:
	      ROS_INFO( "received radar stop from REAR LEFT" );
	      break;

	    default:
	      ROS_ERROR( "received unknown radar start/stop from REAR LEFT" );
	      break;
	    }
	  break;
	    
	case RADAR_START_STOP_TIPI_79_RR_RET:
	  switch( start_stop_byte )
	    {
	    case RADAR_START:
	      ROS_INFO( "received radar start from REAR RIGHT" );
	      break;

	    case RADAR_STOP:
	      ROS_INFO( "received radar stop from REAR RIGHT" );
	      break;

	    default:
	      ROS_ERROR( "received unknown radar start/stop from REAR RIGHT" );
	      break;
	    }
	  break;
	    
	default:
	  ROS_ERROR( "received unknown radar start message: %02x", can_msg.id );
	  break;
	}
    }

  // Parse out start of frame messages (KANZA comes first in BSD firmware):
  else if( can_msg.id >= RADAR_START_FRAME_KANZA && can_msg.id <= RADAR_START_FRAME_TIPI_79_RR )
    {
      switch( can_msg.id )
	{
	case RADAR_START_FRAME_KANZA:
	  if( use_radar_front_center_tracked_ || use_radar_front_center_raw_ )
	    {
	      ROS_INFO( "received start frame from KANZA" );
	      marker_tracked_[KANZA].points.clear();
	      marker_raw_[KANZA].points.clear();
	      text_marker_[KANZA].clear();
	    }
	  break;

	case RADAR_START_FRAME_TIPI_79_FL:
	  if( use_radar_front_left_tracked_ || use_radar_front_left_raw_ )
	    {
	      ROS_INFO( "received start frame from FRONT LEFT" );
	      marker_tracked_[TIPI_79_FL].points.clear();
	      marker_raw_[TIPI_79_FL].points.clear();
	    }
	  break;
	    
	case RADAR_START_FRAME_TIPI_79_FR:
	  if( use_radar_front_right_tracked_ || use_radar_front_right_raw_ )
	    {
	      ROS_INFO( "received start frame from FRONT RIGHT" );
	      marker_tracked_[TIPI_79_FR].points.clear();
	      marker_raw_[TIPI_79_FR].points.clear();
	    }
	  break;
	    
	case RADAR_START_FRAME_TIPI_79_RL:
	  if( use_radar_rear_left_tracked_ || use_radar_rear_left_raw_ || use_radar_rear_left_bsd_ )
	    {
	      ROS_INFO( "received start frame from REAR LEFT" );
	      marker_tracked_[TIPI_79_RL].points.clear();
	      marker_raw_[TIPI_79_RL].points.clear();
	      BSD_marker_[TIPI_79_RL_BSD].points.clear();
	    }
	  break;
	    
	case RADAR_START_FRAME_TIPI_79_RR:
	  if( use_radar_rear_right_tracked_ || use_radar_rear_right_raw_ || use_radar_rear_right_bsd_ )
	    {
	      ROS_INFO( "received start frame from REAR RIGHT" );
	      marker_tracked_[TIPI_79_RR].points.clear();
	      marker_raw_[TIPI_79_RR].points.clear();
	      BSD_marker_[TIPI_79_RR_BSD].points.clear();
	    }
	  break;
	    
	default:
	  ROS_ERROR( "received unknown start message: %02x", can_msg.id );
	  break;
	}
    }

  // Parse out end of frame messages:
  else if( can_msg.id >= RADAR_STOP_FRAME_KANZA && can_msg.id <= RADAR_STOP_FRAME_TIPI_79_RR )
    {
      switch( can_msg.id )
	{
	case RADAR_STOP_FRAME_KANZA:
	  if( use_radar_front_center_tracked_ || use_radar_front_center_raw_ )
	    ROS_INFO( "received stop frame from KANZA" );
	    {
	      pub_target_marker_[KANZA].publish( marker_tracked_[KANZA] );
	      pub_target_marker_[KANZA].publish( marker_raw_[KANZA] );
		    
	      for( auto it = text_marker_[KANZA].begin(); it != text_marker_[KANZA].end(); ++it )
		{
		  pub_target_marker_[KANZA].publish( *it );
		}
	    }
	  break;

	case RADAR_STOP_FRAME_TIPI_79_FL:
	  if( use_radar_front_left_tracked_ || use_radar_front_left_raw_ )
	    {
	      ROS_INFO( "received stop frame from FRONT LEFT" );
	      pub_target_marker_[TIPI_79_FL].publish( marker_tracked_[TIPI_79_FL] );
	      pub_target_marker_[TIPI_79_FL].publish( marker_raw_[TIPI_79_FL] );
	    }
	  break;
	    
	case RADAR_STOP_FRAME_TIPI_79_FR:
	  if( use_radar_front_right_tracked_ || use_radar_front_right_raw_ )
	    {
	      ROS_INFO( "received stop frame from FRONT RIGHT" );
	      pub_target_marker_[TIPI_79_FR].publish( marker_tracked_[TIPI_79_FR] );
	      pub_target_marker_[TIPI_79_FR].publish( marker_raw_[TIPI_79_FR] );
	    }
	  break;
	    
	case RADAR_STOP_FRAME_TIPI_79_RL:
	  if( use_radar_rear_left_tracked_ || use_radar_rear_left_raw_ || use_radar_rear_left_bsd_ )
	    {
	      ROS_INFO( "received stop frame from REAR LEFT" );
	      pub_target_marker_[TIPI_79_RL].publish( marker_tracked_[TIPI_79_RL] );
	      pub_target_marker_[TIPI_79_RL].publish( marker_raw_[TIPI_79_RL] );
	      pub_BSD_marker_[TIPI_79_RL_BSD].publish( BSD_marker_[TIPI_79_RL_BSD] );
	    }
	  break;
	    
	case RADAR_STOP_FRAME_TIPI_79_RR:
	  if( use_radar_rear_right_tracked_ || use_radar_rear_right_raw_ || use_radar_rear_right_bsd_ )
	    {
	      ROS_INFO( "received stop frame from REAR RIGHT" );
	      pub_target_marker_[TIPI_79_RR].publish( marker_tracked_[TIPI_79_RR] );
	      pub_target_marker_[TIPI_79_RR].publish( marker_raw_[TIPI_79_RR] );
	      pub_BSD_marker_[TIPI_79_RR_BSD].publish( BSD_marker_[TIPI_79_RR_BSD] );
	    }
	  break;
	    
	default:
	  ROS_ERROR( "received unknown stop message: %02x", can_msg.id );
	  break;
	}
    }

  // Parse out tracked and raw target data messages (note we don't use KANZA TRACKED because of
  // message ID overlap with TIPI BSD software. KANZA RAW comes last here using old firmware):
  else if( ( can_msg.id >= RADAR_TRACK_KANZA && can_msg.id <= RADAR_TRACK_TIPI_79_RR ) ||
	   ( can_msg.id >= RADAR_RAW_KANZA && can_msg.id <= RADAR_RAW_TIPI_79_RR ) )
    {
      // Extract the target ID and data from the message:
      uint8_t target_id     = can_msg.data[0];
      float target_snr      = can_msg.data[1];        
      float target_range    = (int16_t)( (can_msg.data[2] << 8) + can_msg.data[3]) / 100.0;
      float target_velocity = (int16_t)( (can_msg.data[4] << 8) + can_msg.data[5]) / 100.0;
      float target_angle    = (int16_t)( (can_msg.data[6] << 8) + can_msg.data[7]) / 100.0 * -1;

      // Compute the target Cartesian position from polar coordinates:
      geometry_msgs::Point p;
      p.x = cos(target_angle / 180 * M_PI) * target_range;
      p.y = sin(target_angle / 180 * M_PI) * target_range;
      p.z = 0.0;
	
      switch( can_msg.id )
	{
	  // TRACKED TARGET MESSAGES:
	case RADAR_TRACK_KANZA:
	  if( use_radar_front_center_tracked_ )
	    {
	      ROS_INFO( "received raw target from KANZA" );
	      marker_tracked_[KANZA].header.frame_id = "/front_center_radar_link";
	      marker_tracked_[KANZA].header.stamp    = ros::Time::now();
	      marker_tracked_[KANZA].id              = target_id;
	      marker_tracked_[KANZA].scale.x              = 5.0;
	      marker_tracked_[KANZA].scale.y              = 5.0;
	      marker_tracked_[KANZA].scale.z              = 0.0;
	      marker_tracked_[KANZA].color.r            = 1.0;
	      marker_tracked_[KANZA].color.g            = 0.0;
	      marker_tracked_[KANZA].color.b            = 1.0;
	      marker_tracked_[KANZA].color.a            = 1.0;
	      marker_tracked_[KANZA].points.push_back( p );

	      visualization_msgs::Marker m;
	      m.header.frame_id = "/front_center_radar_link";
	      m.ns              = "target_text";
	      m.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
	      m.action          = visualization_msgs::Marker::ADD;
	      m.scale.z         = marker_tracked_[KANZA].scale.x;
	      m.lifetime        = ros::Duration( 0.1 );
	      m.header.stamp    = ros::Time::now();
	      m.id              = target_id;
	      m.color.r         = 0.0;
	      m.color.g         = 0.0;
	      m.color.b         = 1.0;
	      m.color.a         = 1.0;
	      m.text            = to_string_with_precision( target_range, 4 );
	      m.pose.position.x = p.x;
	      m.pose.position.y = p.y;
	      m.pose.position.z = p.z;
	      text_marker_[KANZA].push_back( m );
	    }
	  break;

	case RADAR_TRACK_TIPI_79_FL:
	  if( use_radar_front_left_tracked_ )
	    {
	      ROS_INFO( "received tracked target from FRONT LEFT" );
	      marker_tracked_[TIPI_79_FL].header.frame_id = "/front_left_radar_link";
	      marker_tracked_[TIPI_79_FL].header.stamp    = ros::Time::now();
	      marker_tracked_[TIPI_79_FL].id              = target_id;
	      marker_tracked_[TIPI_79_FL].color.r            = 1.0;
	      marker_tracked_[TIPI_79_FL].color.g            = 0.0;
	      marker_tracked_[TIPI_79_FL].color.b            = 0.0;
	      marker_tracked_[TIPI_79_FL].color.a            = 1.0;
	      marker_tracked_[TIPI_79_FL].points.push_back( p );
	    }
	  break;

	case RADAR_TRACK_TIPI_79_FR:
	  if( use_radar_front_right_tracked_ )
	    {
	      ROS_INFO( "received tracked target from FRONT RIGHT" );
	      marker_tracked_[TIPI_79_FR].header.frame_id = "/front_right_radar_link";
	      marker_tracked_[TIPI_79_FR].header.stamp    = ros::Time::now();
	      marker_tracked_[TIPI_79_FR].id              = target_id;
	      marker_tracked_[TIPI_79_FR].color.r            = 0.0;
	      marker_tracked_[TIPI_79_FR].color.g            = 1.0;
	      marker_tracked_[TIPI_79_FR].color.b            = 0.0;
	      marker_tracked_[TIPI_79_FR].color.a            = 1.0;
	      marker_tracked_[TIPI_79_FR].points.push_back( p );
	    }
	  break;
	    
	case RADAR_TRACK_TIPI_79_RL:
	  if( use_radar_rear_left_tracked_ )
	    {
	      ROS_INFO( "received tracked target from REAR LEFT" );
	      marker_tracked_[TIPI_79_RL].header.frame_id = "/rear_left_radar_link";
	      marker_tracked_[TIPI_79_RL].header.stamp    = ros::Time::now();
	      marker_tracked_[TIPI_79_RL].id              = target_id;
	      marker_tracked_[TIPI_79_RL].color.r            = 0.0;
	      marker_tracked_[TIPI_79_RL].color.g            = 0.0;
	      marker_tracked_[TIPI_79_RL].color.b            = 1.0;
	      marker_tracked_[TIPI_79_RL].color.a            = 1.0;
	      marker_tracked_[TIPI_79_RL].points.push_back( p );
	    }
	  break;
	    
	case RADAR_TRACK_TIPI_79_RR:
	  if( use_radar_rear_right_tracked_ )
	    {
	      ROS_INFO( "received tracked target from REAR RIGHT" );
	      marker_tracked_[TIPI_79_RR].header.frame_id = "/rear_right_radar_link";
	      marker_tracked_[TIPI_79_RR].header.stamp    = ros::Time::now();
	      marker_tracked_[TIPI_79_RR].id              = target_id;
	      marker_tracked_[TIPI_79_RR].color.r            = 1.0;
	      marker_tracked_[TIPI_79_RR].color.g            = 1.0;
	      marker_tracked_[TIPI_79_RR].color.b            = 0.0;
	      marker_tracked_[TIPI_79_RR].color.a            = 1.0;
	      marker_tracked_[TIPI_79_RR].points.push_back( p );
	    }
	  break;

	  // RAW TARGET MESSAGES:	      
	case RADAR_RAW_KANZA:
	  if( use_radar_front_center_raw_ )
	    {
	      ROS_INFO( "received raw target from KANZA" );
	      marker_raw_[KANZA].header.frame_id = "/front_center_radar_link";
	      marker_raw_[KANZA].header.stamp    = ros::Time::now();
	      marker_raw_[KANZA].id              = target_id;
	      marker_raw_[KANZA].color.r            = 1.0;
	      marker_raw_[KANZA].color.g            = 0.0;
	      marker_raw_[KANZA].color.b            = 1.0;
	      marker_raw_[KANZA].color.a            = 1.0;
	      marker_raw_[KANZA].points.push_back( p );
	    }
	  break;

	case RADAR_RAW_TIPI_79_FL:
	  if( use_radar_front_left_raw_ )
	    {
	      ROS_INFO( "received raw target from FRONT LEFT" );
	      marker_raw_[TIPI_79_FL].header.frame_id = "/front_left_radar_link";
	      marker_raw_[TIPI_79_FL].header.stamp    = ros::Time::now();
	      marker_raw_[TIPI_79_FL].id              = target_id;
	      marker_raw_[TIPI_79_FL].color.r            = 1.0;
	      marker_raw_[TIPI_79_FL].color.g            = 0.0;
	      marker_raw_[TIPI_79_FL].color.b            = 0.0;
	      marker_raw_[TIPI_79_FL].color.a            = 1.0;
	      marker_raw_[TIPI_79_FL].points.push_back( p );
	    }
	  break;
	    
	case RADAR_RAW_TIPI_79_FR:
	  if( use_radar_front_right_raw_ )
	    {
	      ROS_INFO( "received raw target from FRONT RIGHT" );
	      marker_raw_[TIPI_79_FR].header.frame_id = "/front_right_radar_link";
	      marker_raw_[TIPI_79_FR].header.stamp    = ros::Time::now();
	      marker_raw_[TIPI_79_FR].id              = target_id;
	      marker_raw_[TIPI_79_FR].color.r            = 0.0;
	      marker_raw_[TIPI_79_FR].color.g            = 1.0;
	      marker_raw_[TIPI_79_FR].color.b            = 0.0;
	      marker_raw_[TIPI_79_FR].color.a            = 1.0;
	      marker_raw_[TIPI_79_FR].points.push_back( p );
	    }
	  break;
	    
	case RADAR_RAW_TIPI_79_RL:
	  if( use_radar_rear_left_raw_ )
	    {
	      ROS_INFO( "received raw target from REAR LEFT" );
	      marker_raw_[TIPI_79_RL].header.frame_id = "/rear_left_radar_link";
	      marker_raw_[TIPI_79_RL].header.stamp    = ros::Time::now();
	      marker_raw_[TIPI_79_RL].id              = target_id;
	      marker_raw_[TIPI_79_RL].color.r            = 0.0;
	      marker_raw_[TIPI_79_RL].color.g            = 0.0;
	      marker_raw_[TIPI_79_RL].color.b            = 1.0;
	      marker_raw_[TIPI_79_RL].color.a            = 1.0;
	      marker_raw_[TIPI_79_RL].points.push_back( p );
	    }
	  break;
	    
	case RADAR_RAW_TIPI_79_RR:
	  if( use_radar_rear_right_raw_ )
	    {
	      ROS_INFO( "received raw target from REAR RIGHT" );
	      ROS_INFO( "point data: x: %f, y: %f, z: %f", p.x, p.y, p.z );
	      marker_raw_[TIPI_79_RR].header.frame_id = "/rear_right_radar_link";
	      marker_raw_[TIPI_79_RR].header.stamp    = ros::Time::now();
	      marker_raw_[TIPI_79_RR].id              = target_id;
	      marker_raw_[TIPI_79_RR].color.r            = 1.0;
	      marker_raw_[TIPI_79_RR].color.g            = 1.0;
	      marker_raw_[TIPI_79_RR].color.b            = 0.0;
	      marker_raw_[TIPI_79_RR].color.a            = 1.0;
	      marker_raw_[TIPI_79_RR].points.push_back( p );
	    }
	  break;
	    
	default:
	  ROS_ERROR( "received unknown target message: %02x", can_msg.id );
	  break;
	}
    }
  // Parse out Blind Spot Detection (BSD) data messages:
  else if( can_msg.id == RADAR_BSD_TIPI_79_RL || can_msg.id == RADAR_BSD_TIPI_79_RR )
    {
      // Extract the alarms byte which is parsed bitwise for different alarms:
      uint8_t alarms = can_msg.data[1];
      bool LCA_alarm = ( 1UL << 6 ) & alarms;
      bool CVW_alarm = ( 1UL << 4 ) & alarms;
      bool BSD_alarm = ( 1UL << 2 ) & alarms;

      geometry_msgs::Point p;
	
      switch( can_msg.id )
	{
	case RADAR_BSD_TIPI_79_RL:
	  if( use_radar_rear_left_bsd_ )
	    {
	      ROS_INFO( "received BSD target from REAR LEFT." );
	      if( LCA_alarm )
		{
		  ROS_INFO( "REAR LEFT LCA ALARM" );
		}
	      if( CVW_alarm )
		{
		  ROS_INFO( "REAR LEFT CVW ALARM" );
		}
	      if( BSD_alarm )
		{
		  ROS_INFO( "REAR LEFT BSD ALARM" );
		}

	      // Visualize the alarm regardless of alarm type:
	      BSD_marker_[TIPI_79_RL_BSD].header.frame_id = "/base_link";
	      BSD_marker_[TIPI_79_RL_BSD].header.stamp    = ros::Time::now();
	      BSD_marker_[TIPI_79_RL_BSD].id              = 1;
	      BSD_marker_[TIPI_79_RL_BSD].color.r            = 0.0;
	      BSD_marker_[TIPI_79_RL_BSD].color.g            = 0.0;
	      BSD_marker_[TIPI_79_RL_BSD].color.b            = 1.0;
	      BSD_marker_[TIPI_79_RL_BSD].color.a            = 1.0;
	      p.x = -3.0;
	      p.y = 3.0;
	      p.z = 0.0;
	      BSD_marker_[TIPI_79_RL_BSD].points.push_back( p );
	    }
	  break;
		
	case RADAR_BSD_TIPI_79_RR:
	  if( use_radar_rear_right_bsd_ )
	    {
	      ROS_INFO( "received BSD target from REAR RIGHT." );
	      if( LCA_alarm )
		{
		  ROS_INFO( "REAR RIGHT LCA ALARM" );
		}
	      if( CVW_alarm )
		{
		  ROS_INFO( "REAR RIGHT CVW ALARM" );
		}
	      if( BSD_alarm )
		{
		  ROS_INFO( "REAR RIGHT BSD ALARM" );
		}

	      // Visualize the alarm regardless of alarm type:
	      BSD_marker_[TIPI_79_RR_BSD].header.frame_id = "/base_link";
	      BSD_marker_[TIPI_79_RR_BSD].header.stamp    = ros::Time::now();
	      BSD_marker_[TIPI_79_RR_BSD].id              = 1;
	      BSD_marker_[TIPI_79_RR_BSD].color.r            = 1.0;
	      BSD_marker_[TIPI_79_RR_BSD].color.g            = 1.0;
	      BSD_marker_[TIPI_79_RR_BSD].color.b            = 0.0;
	      BSD_marker_[TIPI_79_RR_BSD].color.a            = 1.0;
	      p.x = -3.0;
	      p.y = -3.0;
	      p.z = 0.0;
	      BSD_marker_[TIPI_79_RR_BSD].points.push_back( p );
	    }
	  break;
	    
	default:
	  ROS_ERROR( "received unknown BSD target message: %02x", can_msg.id );
	  break;
	}
    }    
  else
    {
      ROS_ERROR( "received message with unknown id: %02x", can_msg.id );
    }
}
