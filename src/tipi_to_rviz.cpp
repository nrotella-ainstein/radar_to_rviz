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

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>

const float PI = 3.14159265359;

class subAndPub
{
public:

  //SubAndPub constructor definition
  subAndPub()
  {     
   
    //set up.
    pubCldFront = nh.advertise<visualization_msgs::MarkerArray>("tipi_marker_array", 10);
    pubRadarCommands = nh.advertise<can_msgs::Frame>("/sent_messages", 10);
  

    //send the start command to the radar
    ros::Duration(1).sleep();
    start_radar();
    ros::Duration(1).sleep();

    //Subscribe to whatever topic is publishing Tipi CAN frames.
    //if you used 'rosrun socketcan_bridge socketcan_to_topic') it is 'received_messages'
    sub = nh.subscribe("received_messages", 10, &subAndPub::callback, this);
    shape = visualization_msgs::Marker::CUBE;

  }



  void start_radar()
  {
    can_msgs::Frame frame;
    frame.header.frame_id = "0";
    frame.header.stamp    = ros::Time::now();
    frame.is_rtr          = false;
    frame.is_extended     = false;
    frame.is_error        = false;
    frame.dlc             = 8;
    frame.id              = 256;
    frame.data[0]         = 0x01;
    frame.data[1]         = 0xff;
    frame.data[2]         = 0xff;
    frame.data[3]         = 0xff;
    frame.data[4]         = 0xff;
    frame.data[5]         = 0xff;
    frame.data[6]         = 0xff;
    frame.data[7]         = 0xff;

    pubRadarCommands.publish(frame);
    ros::Duration(1).sleep();
  }



  void stop_radar()
  {
    can_msgs::Frame frame;
    frame.header.frame_id = "0";
    frame.header.stamp    = ros::Time::now();
    frame.is_rtr          = false;
    frame.is_extended     = false;
    frame.is_error        = false;
    frame.dlc             = 8;
    frame.id              = 256;
    frame.data[0]         = 0x02;
    frame.data[1]         = 0xff;
    frame.data[2]         = 0xff;
    frame.data[3]         = 0xff;
    frame.data[4]         = 0xff;
    frame.data[5]         = 0xff;
    frame.data[6]         = 0xff;
    frame.data[7]         = 0xff;

    pubRadarCommands.publish(frame);
    ros::Duration(1).sleep();
  }


  //SubAndPub callback definition
  void callback(const can_msgs::Frame& input)
  {
    ros::Rate publish_rate(20);   
    //parse input CAN frame
    //parsing is for 'target object'
    uint32_t frame_id     = input.id;
    uint8_t target_id     = input.data[0];
    float target_snr      = input.data[1];        
    float target_range    = (int16_t)( (input.data[2] << 8) + input.data[3]) / 100.0;
    float target_velocity = (int16_t)( (input.data[4] << 8) + input.data[5]) / 100.0;
    float target_angle    = (int16_t)( (input.data[6] << 8) + input.data[7]) / 100.0 * -1;


    //Parse the can frame.
    //check if frame is a target
    if((frame_id >= 1024 and frame_id < 1075) || //this is the tracked detection ID range
       (frame_id >= 1280 and frame_id < 1380) )  //this is the raw detection ID range
    {
      //create a marker for target and initialize
      visualization_msgs::Marker marker;

      marker.header.frame_id = "/map";
      marker.header.stamp    = ros::Time::now();
      marker.ns              = "basic_shapes";
      marker.id              = target_id;
      marker.type            = shape;
      marker.action          = visualization_msgs::Marker::ADD;

      // Set the pose and orientation of the marker.
      // TODO Setting size / color of marker based off SNR / Velocity
      marker.pose.position.x    = cos(target_angle / 180 * PI) * target_range;
      marker.pose.position.y    = sin(target_angle / 180 * PI) * target_range;
      marker.pose.position.z    = 1;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x            = 0.5;
      marker.scale.y            = 0.5;
      marker.scale.z            = 1.0;
      marker.color.r            = 0.3f;
      marker.color.g            = 0.4f;
      marker.color.b            = 0.67f;
      marker.color.a            = 1.0;
      marker.lifetime           = ros::Duration(0.05);

      markerArray.markers.push_back(marker);
    }

    else if (frame_id == 1152) 
    {
      //frame signaling start of target list
      //clear previous iterations target markers
      markerArray.markers.clear();
    }

    else if (frame_id == 1153)
    {
      //frame signaling end of target list
      //used to publish the marker array

        while (pubCldFront.getNumSubscribers() < 1)
        {
          if (!ros::ok())
          {
            return;
          }
          ROS_WARN_ONCE("Please create a subscriber to the marker array");
          sleep(1);
        }
        pubCldFront.publish(markerArray);
        publish_rate.sleep();
    }
  }//end of callback

private:
  ros::NodeHandle nh; 
  ros::Publisher pubCldFront;
  ros::Publisher pubRadarCommands;
  ros::Subscriber sub; 
  uint32_t shape;
  visualization_msgs::MarkerArray markerArray;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tipi_node");

  subAndPub mydood;
  ros::spin();
  return 0;
}
