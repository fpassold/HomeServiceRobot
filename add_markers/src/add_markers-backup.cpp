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

#include <string> 
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
  // uint32_t shape = visualization_msgs::Marker::CUBE;
  uint32_t shape = visualization_msgs::Marker::SPHERE; 
  // %EndTag(SHAPE_INIT)%

  //                  0      1        2        3        4      5
  // double alpha[6] = {1.0, 0.89413, 0.77016, 0.61034, 0.38508, 0.0 }; // define tags transparency
  // based on: aplha=log(i+1)/1.8 (Octave)
  int i; // for passing seconds
  int count = 3; // limits number of repetitions of the add-maker loop

// %Tag(MARKER_INIT)%
  while ((ros::ok())&&(count>0))
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
    marker.ns = "Collection_Marker";
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
    /*
    marker.pose.position.x = 3.43;
    marker.pose.position.y = 3.0;
    */
    marker.pose.position.x = 3.0;
    marker.pose.position.y = -3.2;

    marker.pose.position.z = 0.0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    // %Tag(SCALE)%
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.05;
    // %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
    // %Tag(COLOR)%
    // Colors ref.: https://www.w3schools.com/colors/colors_picker.asp
    /* Navy blue:
    marker.color.r = 0.0f; marker.color.g = 0.8f; marker.color.b = 1.0f; */
    /* Light green: */
    marker.color.r = 0.4f; marker.color.g = 1.0f; marker.color.b = 0.2f;
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
      // ROS_WARN_ONCE("Please create a subscriber to the marker");
      ROS_WARN_ONCE("Please Add: Display >> Marker in RViz to activate add_marker node.");
      sleep(1);
    }

    // Publish the First marker (pick a object)
    ROS_INFO("------");
    ROS_INFO("1. Displaying collection point (5 sec)");
    
    marker_pub.publish(marker);
    // ros::Duration(5.0).sleep();
    for (i=0; i<5; i++){
      marker.pose.position.z = 0.0;
      marker.color.a = 1.0; marker_pub.publish(marker);
      ros::Duration(0.5).sleep();
      marker.color.a = 0.0; marker_pub.publish(marker);

      marker.pose.position.z = 0.4;
      marker.color.a = 1.0; marker_pub.publish(marker);      
      ros::Duration(0.5).sleep();
      marker.color.a = 0.0; marker_pub.publish(marker);
    }
    // ROS_INFO("   Deleting pick_marker");

    ROS_INFO("2. Letting 5 seconds pass... simulating delivery... ");
    ros::Duration(5.0).sleep();    

    // Publishing second marker
    ROS_INFO("3. Displaying delivery point (5 sec)");
    // %Tag(POSE)%
    /*
    marker.pose.position.x = -4.0;
    marker.pose.position.y = -5.0;
    */
    marker.pose.position.x = -5.0;
    marker.pose.position.y =  4.0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // %EndTag(POSE)%
    // %Tag(COLOR)%
    /* Strawberry color
    marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.4f;
    */

    // marker.color.a = 1.0;
    // %EndTag(COLOR)%
    for (i=0; i<5; i++){
      marker.pose.position.z = 0.0;
      marker.color.a = 1.0; marker_pub.publish(marker);
      ros::Duration(0.5).sleep();
      marker.color.a = 0.0; marker_pub.publish(marker);

      marker.pose.position.z = 0.4;
      marker.color.a = 1.0; marker_pub.publish(marker);      
      ros::Duration(0.5).sleep();
      marker.color.a = 0.0; marker_pub.publish(marker);
    }
    // ROS_INFO("   Deleting pick_marker");
    count--;
    if (count>0){
      marker.color.a = 0.0; marker_pub.publish(marker); // it seems that ros "forgets" to execute last publish
      ROS_INFO("   Ready for another pick and place cycle (count=%d)", count);
      ROS_INFO("   Waiting for 5 seconds");   
      ros::Duration(5.0).sleep(); 
    }
    // %Tag(SLEEP_END)%
    r.sleep();
  }
// %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
