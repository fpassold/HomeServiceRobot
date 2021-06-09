// add_markera - special edt for 2nd submission - Fernando Passold, 09.06.2021 
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "zones.h" // Definitions of constants associated with pickup and drop off zones
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <nav_msgs/Odometry.h>   // needed to access odometry messages
#include <math.h>
using namespace std;

// global variables
visualization_msgs::Marker marker;
bool near_pickup_zone = false;
bool near_dropoff_zone = false;
double tolerance = 0.2; // distance from robot to marker to be considered hit
bool finish = false;

// just initialize marker, to make code clearer
void InitializeMarker(void){
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  // uint32_t shape = visualization_msgs::Marker::SPHERE; 
    // visualization_msgs::Marker marker; // defined as global variable
    marker.header.frame_id = "/map"; /* indicating which is the fixed frame  */
    marker.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    // marker.ns = "basic_shapes";
    marker.ns = "Collection_Markers";
    marker.id = 0;
    marker.type = shape;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    // Set the SIZE of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.01;
}

// Define marker with pickup position
void SetPickupMarkerZone(void){
    marker.pose.position.x = pickup_zone_x; // 2.8;
    marker.pose.position.y = pickup_zone_y; //-3.2;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0; // 1=Euler; 0=Quaternion
    // Set the color -- be sure to set alpha to something non-zero!
    // Colors ref.: https://www.w3schools.com/colors/colors_picker.asp
    /* Navy blue:
    marker.color.r = 0.0f; marker.color.g = 0.8f; marker.color.b = 1.0f; */
    /* Light green: */
    marker.color.r = 0.4f; marker.color.g = 1.0f; marker.color.b = 0.2f;
    marker.lifetime = ros::Duration();
}

// Define marker with drop off position
void SetDropOffMarkerZone(void){
    /*
    marker.pose.position.x = -4.0;
    marker.pose.position.y = -5.0;
    */
    marker.pose.position.x = drop_off_zone_x; // -5.2;
    marker.pose.position.y = drop_off_zone_y; //  4.0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    /* Strawberry color
     * marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.4f;
    */
    /* Navy blue: */
    marker.color.r = 0.0f; marker.color.g = 0.8f; marker.color.b = 1.0f;
}

double distance(double x1, double y1, double x2, double y2){
  return sqrt(pow((x1 - x2), 2.0) + pow((y1 - y2), 2.0));
}

/*  Based on the odometry data, it checks if the robot is close to any marker 
 *  and activates the corresponding Boolean variable
 */
void OdomCheck(const nav_msgs::Odometry &odom_msg){
  double position_x = odom_msg.pose.pose.position.y;
  double position_y = -odom_msg.pose.pose.position.x; // is "inverted"
  double dist1, dist2;
  
  dist1 = distance(position_x, position_y, pickup_zone_x, pickup_zone_y);
  dist2 = distance(position_x, position_y, drop_off_zone_x, drop_off_zone_y);
  // ROS_INFO("Actual position: [x,y] = [%6.3f, %6.3f] - dist = %6.3f / %6.3f", position_x, position_y, dist1, dist2);

  near_pickup_zone = false;
  near_dropoff_zone = false;
  if (dist1 < tolerance){
    near_pickup_zone = true;
    // ROS_INFO("  Actual position: [x,y] = [%6.3f, %6.3f] - dist = %6.3f", position_x, position_y, dist1);
  }
  if (dist2 < tolerance){
    near_dropoff_zone = true;
    // ROS_INFO("  Actual position: [x,y] = [%6.3f, %6.3f] - dist = %6.3f", position_x, position_y, dist2);
  }
}

int main( int argc, char** argv ){

  ros::init(argc, argv, "add_markers"); // ROS node name identification
  ros::NodeHandle n;

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Subscriber odom_sub = n.subscribe("odom", 2, OdomCheck);

  ROS_INFO("Pickup  zone: x=%5.2f , y=%5.2f", pickup_zone_x, pickup_zone_y);
  ROS_INFO("Dropoff zone: x=%5.2f , y=%5.2f", drop_off_zone_x, drop_off_zone_y);

  ros::Rate rate(2);

  while (ros::ok()){
    InitializeMarker();
    while (marker_pub.getNumSubscribers() < 1){
      if (!ros::ok()){
        return 0;
      }
      // ROS_WARN_ONCE("Please create a subscriber to the marker");
      ROS_WARN_ONCE("Please Add: Display >> Marker in RViz to activate add_markers node.");
      sleep(1);
    }
    // Publish the First marker (pick a object)
    ROS_INFO("------");
    ROS_INFO("Displaying pickup zone");
    SetPickupMarkerZone();
    marker.color.a = 0.6; marker_pub.publish(marker);   
    ROS_INFO("Waiting for robot to reach pickup zone");
    while (!near_pickup_zone){
      // waiting for the robot to reach the collection zone
      ros::spinOnce(); 
    }
    ROS_INFO("Robot reached pickup zone");
    marker.color.a = 0.0; marker_pub.publish(marker); // Deleting pickup marker
    ros::Duration(5.0).sleep(); // simulating the material collection... 
    //  Udacity do not ask for any warning on the screen
    //  Do not show delivery marker while robot moves into this zone,
    //  the user who "guesses" what the robot is doing...
    // Robot moving to material delivery zone
    while (!near_dropoff_zone){
	// waiting for the robot to reach the unloading zone
        ros::spinOnce();
    }
    ROS_INFO("------");
    ROS_INFO("Displaying drop off zone");
    SetDropOffMarkerZone();
    marker.color.a = 1.0; marker_pub.publish(marker); 
    // shows forever the marker for material deposit location as required by Udacity!
    finish = true;        
    if (finish){
      ROS_INFO("Robot successfully completed its mission!");
      ros::Duration(7.0).sleep();
      ROS_INFO("Shutting down add_markers node");
      break; // stops to run this node
    } 
    rate.sleep();
  } //  end while (ros::ok())
  return 0;
} // end main()
