/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
// %EndTag(INCLUDES)%

// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers"); // ROS node name identification
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
// %EndTag(INIT)%

  // Set our initial shape type to be a cube
  // %Tag(SHAPE_INIT)%
  uint32_t shape = visualization_msgs::Marker::CUBE;
  // %EndTag(SHAPE_INIT)%

  //                  0      1        2        3        4      5
  double alpha[6] = {1.0, 0.89413, 0.77016, 0.61034, 0.38508, 0.0 }; // define tags transparency
  // based on: aplha=log(i+1)/1.8 (Octave)
  int i; // for passing seconds

// %Tag(MARKER_INIT)%
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    // marker.header.frame_id = "/my_frame";
    marker.header.frame_id = "/map"; /* indicating which is the fixed frame 
    /* --> msg: [ WARN] [1622338030.750087709]: Please create a subscriber to the marker 
       while Display >> Marker is not activated in RViz ! */
    marker.header.stamp = ros::Time::now();
// %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    // %Tag(NS_ID)%
    // marker.ns = "basic_shapes";
    marker.ns = "Pick_Marker";
    marker.id = 0;
    // %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    // %Tag(TYPE)%
    marker.type = shape;
    // %EndTag(TYPE)%

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    // %Tag(ACTION)%
    marker.action = visualization_msgs::Marker::ADD;
    // %EndTag(ACTION)%

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    // %Tag(POSE)%
    marker.pose.position.x = 3.43;
    marker.pose.position.y = 3.0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    // %Tag(SCALE)%
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    // %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
    // %Tag(COLOR)%
    // Colors ref.: https://www.w3schools.com/colors/colors_picker.asp
    marker.color.r = 0.0f;
    marker.color.g = 0.8f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    // %EndTag(COLOR)%

    // %Tag(LIFETIME)%
    marker.lifetime = ros::Duration();
    // %EndTag(LIFETIME)%

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    // Publish the First marker (pick a object)
    ROS_INFO("1. Showing pick_marker point");
    // marker_pub.publish(marker);
    // ros::Duration(5.0).sleep();
    for (i=0; i<5; i++){
      ros::Duration(1.0).sleep();
      marker.color.a = alpha[i];
      marker_pub.publish(marker);
    }
    // ROS_INFO("   Deleting pick_marker");
    marker.color.a = alpha[i];
    marker_pub.publish(marker);
    ROS_INFO("   Deleted pick_marker");

    ROS_INFO("Letting 5 seconds pass");
    ros::Duration(5.0).sleep();    

    // Publishing second marker
    ROS_INFO("2. Showing place_marker point");
    // %Tag(POSE)%
    marker.pose.position.x = -4.0;
    marker.pose.position.y = -5.0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // %EndTag(POSE)%
    // %Tag(COLOR)%
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.4f;
    // marker.color.a = 1.0;
    // %EndTag(COLOR)%
    for (i=5; i>0; i--){
      ros::Duration(1.0).sleep();
      marker.color.a = alpha[i];
      marker_pub.publish(marker);
    }
    // ROS_INFO("   Deleting pick_marker");
    marker.color.a = alpha[i];
    marker_pub.publish(marker);
    ROS_INFO("3. Allowing 5 seconds to pass (simulating material delivery)");
    ros::Duration(5.0).sleep();
    ROS_INFO("   Deleted pick_marker");
    marker.color.a = 0.0;
    marker_pub.publish(marker);    
    ros::Duration(5.0).sleep(); // just to give some time to read the message before the terminal is shut down
    // %Tag(SLEEP_END)%
    r.sleep();
  }
// %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
