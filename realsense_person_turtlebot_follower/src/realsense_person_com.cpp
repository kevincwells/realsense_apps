/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>
#include <realsense_person/PersonDetection.h>
#include "dynamic_reconfigure/server.h"
#include "realsense_person_turtlebot_follower/RealSensePersonCoMConfig.h"

#include <depth_image_proc/depth_traits.h>


namespace realsense_person_turtlebot_follower
{

/**
 * The turtlebot follower RealSense Person CoM nodelet. Subscribes to 
 * the RealSense Person "detection_data" topic, extracts the person's
 * center of mass location, and publishes goal point messages. These 
 * are in turn used by the Follower nodelet.
 */
class RealSensePersonCoM : public nodelet::Nodelet
{
public:
  /*!
   * @brief The constructor for the RealSense Person CoM nodelet.
   * Constructor for the RealSense Person CoM nodelet.
   */
  RealSensePersonCoM() : max_z_(0.8)
  {

  }

  ~RealSensePersonCoM()
  {
    delete config_srv_;
  }

private:
  bool enabled_; /**< Enable/disable following; */
  double max_z_; /**< The maximum z position of the points in the box. */

  // Dynamic reconfigure server
  dynamic_reconfigure::Server<realsense_person_turtlebot_follower::RealSensePersonCoMConfig>* config_srv_;

  /*!
   * @brief OnInit method from node handle.
   * OnInit method from node handle. Sets up the parameters
   * and topics.
   */
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("max_z", max_z_);
    
    enabled_ = true;

    pointpub_ = private_nh.advertise<geometry_msgs::Point> ("goal", 1);
    markerpub_ = private_nh.advertise<visualization_msgs::Marker>("marker",1);
    sub_= nh.subscribe<realsense_person::PersonDetection>("person/detection_data", 1, &RealSensePersonCoM::personcb, this);

    srvclient_ = nh.serviceClient<turtlebot_msgs::SetFollowState>("/turtlebot_follower/change_state"); 

    config_srv_ = new dynamic_reconfigure::Server<realsense_person_turtlebot_follower::RealSensePersonCoMConfig>(private_nh);
    dynamic_reconfigure::Server<realsense_person_turtlebot_follower::RealSensePersonCoMConfig>::CallbackType f =
        boost::bind(&RealSensePersonCoM::reconfigure, this, _1, _2);
    config_srv_->setCallback(f);
  }

  void reconfigure(realsense_person_turtlebot_follower::RealSensePersonCoMConfig &config, uint32_t level)
  {
    max_z_ = config.max_z;
  }

  /*!
   * @brief Callback for person detection data.
   * Callback for person detection data. Extracts the person
   * Center of Mass from the data and publishes point messages
   * with this goal.
   * @param person_msg The person detection message.
   */
  void personcb(const realsense_person::PersonDetectionConstPtr& person_msg)
  {
    if(person_msg->detected_person_count > 0){
      float x = person_msg->persons[0].center_of_mass.world.x;
      float y = person_msg->persons[0].center_of_mass.world.y;
      float z = person_msg->persons[0].center_of_mass.world.z;
      
      // If outisde max_z limit
      if(z > max_z_){
        if(enabled_){
          ROS_INFO_THROTTLE(1, "Person too far away %f, stopping the robot\n", z);
          if(setFollowerState(false)){
            enabled_ = false;
          } else {
            ROS_ERROR_THROTTLE(1, "Failed to stop Turtlebot Follower via change_state service");
          }
        }
      } 
      // If inside max_z limit
      else { 
        //Start following if it is stopped.
        if(!enabled_){
          if(setFollowerState(true)){
            enabled_ = true;
          } else {
            ROS_ERROR_THROTTLE(1, "Failed to start Turtlebot Follower via change_state service");
          }
        }
        
        ROS_INFO_THROTTLE(1, "Person at %f %f %f", x, y, z);
        
        publishMarker(x, y, z);

        geometry_msgs::PointPtr pnt(new geometry_msgs::Point());
        pnt->x = x;
        pnt->y = y;
        pnt->z = z;
        pointpub_.publish(pnt);
      }
      
    } else {
      if(enabled_){
        ROS_INFO_THROTTLE(1, "No person detected, stopping the robot\n");
        if(setFollowerState(false)){
          enabled_ = false;
        } else {
          ROS_ERROR_THROTTLE(1, "Failed to stop Turtlebot Follower via change_state service");
        }
      }
    }
    
  }

  bool setFollowerState(bool follow)
  {
    turtlebot_msgs::SetFollowState srv;
    if(follow){
      srv.request.state = srv.request.FOLLOW;
    } else {
      srv.request.state = srv.request.STOPPED;
    }
    return srvclient_.call(srv);
  }

  void publishMarker(double x,double y,double z)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    markerpub_.publish( marker );
  }

  ros::Subscriber sub_;
  ros::Publisher pointpub_;
  ros::Publisher markerpub_;
  ros::ServiceClient srvclient_;
};

PLUGINLIB_DECLARE_CLASS(realsense_person_turtlebot_follower, RealSensePersonCoM, realsense_person_turtlebot_follower::RealSensePersonCoM, nodelet::Nodelet);

}
