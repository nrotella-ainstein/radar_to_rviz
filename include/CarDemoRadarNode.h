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

#ifndef CAR_DEMO_RADAR_NODE_H_
#define CAR_DEMO_RADAR_NODE_H_

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>

#include <array>
#include <mutex>
#include <memory>
#include <thread>
#include <sstream>
#include <iomanip>

enum RadarType
  {
    KANZA = 0,
    TIPI_79_FL,
    TIPI_79_FR,
    TIPI_79_RL,
    TIPI_79_RR,
    N_RADARS
  };

enum BSDRadars
  {
    TIPI_79_RL_BSD = 0,
    TIPI_79_RR_BSD,
    N_BSD_RADARS
  };

#define RADAR_START_STOP       0x100
#define RADAR_START            0x01
#define RADAR_STOP             0x02
#define KANZA_RUN_INFINITE     0x00
#define KANZA_RAW_TARGETS      0x02

#define RADAR_SEND_SPEED             0x130
#define RADAR_SPEED_EFFECTIVE_POS    0x01

#define RADAR_START_STOP_KANZA_RET         0x101
#define RADAR_START_STOP_TIPI_79_FL_RET    0x111
#define RADAR_START_STOP_TIPI_79_FR_RET    0x121
#define RADAR_START_STOP_TIPI_79_RL_RET    0x131
#define RADAR_START_STOP_TIPI_79_RR_RET    0x141

#define RADAR_START_FRAME_KANZA         0x420
#define RADAR_START_FRAME_TIPI_79_FL    0x421
#define RADAR_START_FRAME_TIPI_79_FR    0x422
#define RADAR_START_FRAME_TIPI_79_RL    0x423
#define RADAR_START_FRAME_TIPI_79_RR    0x424

#define RADAR_STOP_FRAME_KANZA         0x480
#define RADAR_STOP_FRAME_TIPI_79_FL    0x481
#define RADAR_STOP_FRAME_TIPI_79_FR    0x482
#define RADAR_STOP_FRAME_TIPI_79_RL    0x483
#define RADAR_STOP_FRAME_TIPI_79_RR    0x484

#define RADAR_SET_ID               0x03
#define RADAR_SET_ID_KANZA         0x01
#define RADAR_SET_ID_TIPI_79_FL    0x02
#define RADAR_SET_ID_TIPI_79_FR    0x03
#define RADAR_SET_ID_TIPI_79_RL    0x04
#define RADAR_SET_ID_TIPI_79_RR    0x05

#define RADAR_SET_DISABLE_BSD    0x00
#define RADAR_SET_ENABLE_BSD     0x01

#define RADAR_ID_KANZA         0x101
#define RADAR_ID_TIPI_79_FL    0x111
#define RADAR_ID_TIPI_79_FR    0x121
#define RADAR_ID_TIPI_79_RL    0x131
#define RADAR_ID_TIPI_79_RR    0x141

#define RADAR_TRACK_KANZA         0x490
#define RADAR_TRACK_TIPI_79_FL    0x491
#define RADAR_TRACK_TIPI_79_FR    0x492
#define RADAR_TRACK_TIPI_79_RL    0x493
#define RADAR_TRACK_TIPI_79_RR    0x494

#define RADAR_RAW_KANZA         0x4A0
#define RADAR_RAW_TIPI_79_FL    0x4A1
#define RADAR_RAW_TIPI_79_FR    0x4A2
#define RADAR_RAW_TIPI_79_RL    0x4A3
#define RADAR_RAW_TIPI_79_RR    0x4A4
 
#define RADAR_BSD_TIPI_79_RL    0x453
#define RADAR_BSD_TIPI_79_RR    0x454

class CarDemoRadarNode
{

public:

  CarDemoRadarNode( void );
  ~CarDemoRadarNode( void );

  void startRadar( void );
  void stopRadar( void );
  void sendSpeedToRadar( double car_speed );
  
  template <typename T> static std::string to_string_with_precision(const T a_value,
								    const int n = 6)
  {
    std::ostringstream out;
    out << std::setprecision(n) << a_value;
    return out.str();
  }

private:

  void speedMsgCallback( const std_msgs::Float64& speed_msg );
  void canMsgCallback( const can_msgs::Frame& can_msg );
  void keepAliveLoop( void );
  
  ros::NodeHandle node_handle_;
  ros::Publisher  pub_radar_;
  ros::Subscriber sub_can_msg_;
  ros::Subscriber sub_speed_msg_;
  
  std::array<ros::Publisher, N_RADARS> pub_target_marker_;
  std::array<visualization_msgs::Marker, N_RADARS> marker_raw_;
  std::array<visualization_msgs::Marker, N_RADARS> marker_tracked_;
  
  std::array<std::vector<visualization_msgs::Marker>, N_RADARS> text_marker_;
  std::array<ros::Publisher, N_BSD_RADARS> pub_BSD_marker_;
  std::array<visualization_msgs::Marker, N_BSD_RADARS> BSD_marker_;

  // For creating a "keep alive" thread:
  bool keep_alive_thread_is_running_;
  std::unique_ptr<std::thread> keep_alive_thread_;
  std::mutex keep_alive_mutex_;

  bool use_radar_front_center_tracked_, use_radar_front_center_raw_;
  bool use_radar_front_left_tracked_, use_radar_front_left_raw_;
  bool use_radar_front_right_tracked_, use_radar_front_right_raw_;
  bool use_radar_rear_left_tracked_, use_radar_rear_left_raw_, use_radar_rear_left_bsd_;
  bool use_radar_rear_right_tracked_, use_radar_rear_right_raw_, use_radar_rear_right_bsd_;
  
};

#endif
