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
#include <realsense_person/PersonTracking.h>
#include <realsense_person/StartTracking.h>
#include <realsense_person/StopTracking.h>
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
  bool enable_gesture_stop_;  /**< Enable/disable stopping Turtlebot via gesture */
  bool gesture_tracking_enabled_; /**< Whether the gesture tracking service has been started */
  bool gesture_detected_;  /**< The state of the gesture detection */
  ros::Time gesture_time_;  /**< The time at which a new gesture state was detected. Used to eliminate false positives */
  int gesture_change_duration_ = 1;  /**< How long a new gesture needs to be seen before state change. Used to eliminate false positives */

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
    private_nh.getParam("enable_gesture_stop",enable_gesture_stop_);
    
    enabled_ = true;
    gesture_tracking_enabled_ = false;
    gesture_detected_ = false;
    gesture_time_ = ros::Time::now();

    pointpub_ = private_nh.advertise<geometry_msgs::Point> ("goal", 1);
    markerpub_ = private_nh.advertise<visualization_msgs::Marker>("marker",1);
    person_detection_sub_= nh.subscribe<realsense_person::PersonDetection>("person/detection_data", 1, &RealSensePersonCoM::person_detection_cb, this);
    person_tracking_sub_= nh.subscribe<realsense_person::PersonTracking>("person/tracking_data", 1, &RealSensePersonCoM::person_tracking_cb, this);

    turtlebot_state_srvclient_ = nh.serviceClient<turtlebot_msgs::SetFollowState>("/turtlebot_follower/change_state"); 
    person_tracking_start_srvclient_ = nh.serviceClient<realsense_person::StartTracking>("person/start_tracking_person");
    person_tracking_stop_srvclient_ = nh.serviceClient<realsense_person::StopTracking>("person/stop_tracking");

    config_srv_ = new dynamic_reconfigure::Server<realsense_person_turtlebot_follower::RealSensePersonCoMConfig>(private_nh);
    dynamic_reconfigure::Server<realsense_person_turtlebot_follower::RealSensePersonCoMConfig>::CallbackType f =
        boost::bind(&RealSensePersonCoM::reconfigure, this, _1, _2);
    config_srv_->setCallback(f);
  }

  void reconfigure(realsense_person_turtlebot_follower::RealSensePersonCoMConfig &config, uint32_t level)
  {
    max_z_ = config.max_z;
    enable_gesture_stop_ = config.enable_gesture_stop;
  }

  /*!
   * @brief Callback for person detection data.
   * Callback for person detection data. Extracts the person
   * Center of Mass from the data and publishes point messages
   * with this goal.
   * @param person_detection_msg The person detection message.
   */
  void person_detection_cb(const realsense_person::PersonDetectionConstPtr& person_detection_msg)
  {
    if(person_detection_msg->detected_person_count > 0){
      if(enable_gesture_stop_ && !gesture_tracking_enabled_){
        // Start person tracking
        realsense_person::StartTracking srv;
        srv.request.tracking_id = person_detection_msg->persons[0].person_id.tracking_id;
        if(person_tracking_start_srvclient_.call(srv)){
          gesture_tracking_enabled_ = true;
        }      
      } else if(!enable_gesture_stop_ && gesture_tracking_enabled_){
        // Stop person tracking
        realsense_person::StopTracking srv;
        if(person_tracking_stop_srvclient_.call(srv)){
          gesture_tracking_enabled_ = false;
        }
      }
      
      if(gesture_detected_){
        return;
      }
      
      float x = person_detection_msg->persons[0].center_of_mass.world.x;
      float y = person_detection_msg->persons[0].center_of_mass.world.y;
      float z = person_detection_msg->persons[0].center_of_mass.world.z;
      
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
      // Stop robot
      if(enabled_){
        ROS_INFO_THROTTLE(1, "No person detected, stopping the robot\n");
        if(setFollowerState(false)){
          enabled_ = false;
        } else {
          ROS_ERROR_THROTTLE(1, "Failed to stop Turtlebot Follower via change_state service");
        }
      }
      // Stop person gesture tracking
      gesture_tracking_enabled_ = false;
    }
    
  }
  
  /*!
   * @brief Callback for person tracking data.
   * Callback for person tracking data. Extracts the person
   * skeleton joint points from the data and determines
   * if it should stop the robot.
   * @param person_detection_msg The person detection message.
   */
  void person_tracking_cb(const realsense_person::PersonTrackingConstPtr& person_tracking_msg)
  {
    bool new_gesture_state = false;
    
    realsense_person::SkeletonJoint hand_left;
    realsense_person::SkeletonJoint hand_right;
    for(const realsense_person::SkeletonJoint& joint : person_tracking_msg->body.skeleton_joints){
      if(joint.type == "joint_hand_left"){
        hand_left = joint;
      } else if(joint.type == "joint_hand_right"){
        hand_right = joint;
      }
    }
    
    // If either hand is above the center of mass
    if(hand_left.type != "" && hand_left.point.world.y < person_tracking_msg->person.center_of_mass.world.y){
      new_gesture_state = true;
    } else if(hand_right.type != "" && hand_right.point.world.y < person_tracking_msg->person.center_of_mass.world.y){
      new_gesture_state = true;
    }


    if(new_gesture_state != gesture_detected_){
      if(ros::Time::now().toSec() > (gesture_time_ + ros::Duration(gesture_change_duration_)).toSec()){
        gesture_detected_ = new_gesture_state;
        gesture_time_ = ros::Time::now();
        
        if(gesture_detected_){
          // Stop robot
          if(enabled_){
            ROS_INFO_THROTTLE(1, "Gesture detected, stopping the robot\n");
            if(setFollowerState(false)){
              enabled_ = false;
            } else {
              ROS_ERROR_THROTTLE(1, "Failed to stop Turtlebot Follower via change_state service");
            }
          }
        }
        
      }
    } else {
      gesture_time_ = ros::Time::now();
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
    return turtlebot_state_srvclient_.call(srv);
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

  ros::Subscriber person_detection_sub_;
  ros::Subscriber person_tracking_sub_;
  ros::Publisher pointpub_;
  ros::Publisher markerpub_;
  ros::ServiceClient turtlebot_state_srvclient_;
  ros::ServiceClient person_tracking_start_srvclient_;
  ros::ServiceClient person_tracking_stop_srvclient_;
};

PLUGINLIB_DECLARE_CLASS(realsense_person_turtlebot_follower, RealSensePersonCoM, realsense_person_turtlebot_follower::RealSensePersonCoM, nodelet::Nodelet);

}
