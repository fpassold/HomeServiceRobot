#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "zones.h" // Definitions of constants associated with pickup and drop off zones
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
// #include <geometry_msgs/Twist.h> // needed to access speed commands - not used/monitored
#include <nav_msgs/Odometry.h>   // needed to access odometry messages
#include <math.h>
using namespace std;

// global variables
visualization_msgs::Marker marker;
bool near_pickup_zone = false;
bool near_dropoff_zone = false;
double tolerance = 0.2; // distance from robot to marker to be considered hit
bool finish = false;

#define TIMEOUT 180; // 1.5*60*2 (rate=2Hz) --> timeout de 1,5 minutos

/*
* The lines below can be uncommented when the function file_read_zones() is able 
* to read the file "zones.txt"
* In order to partially replace the functionality that would be achieved if the zones.txt 
* file could be found, the #include "zones.h" file was used, which defines the same global variables 
* as shown below (in this case, in the form of double global constants).
*
double pickup_zone_x = 1.0;
double pickup_zone_y = 1.0;
double drop_off_zone_x = -1.0;
double drop_off_zone_y = -1.0;
*/

/*
* The function below apparently works, but I don't know the path used to retrieve the file.
* "ifstream" does not let you know which folder is being used to read the file.
* It is not the folder where the add_markers.cpp file is located
* Neither /launch nor /src nor /my_robot nor /catkin_ws
* Unfortunately it doesn't work, which would make this program much more flexible
* fpassold@ubuntu1:~/ws_project_7/catkin_ws$ roslaunch my_robot add_markers.launch - don't find the file
* Note: the ROS WEB pages do not tell you how to do this successfully.
*
* Reason why I was forced to use #include "zones.h" or I was forced to comment on this function.
*
* Solution found: copy "zones.txt" file to "~/zones.txt" folder (user home root directory)
* 
*
void file_read_zones(void){
  // tries to read the file "zones.txt" and passes the values of
  // pickup_zone_x, pickup_zone_y, drop_off_zone_x, drop_off_zone_y
  // one line for each parameter in this order
  // Ref.: https://stackoverflow.com/questions/19885481/c-read-from-a-text-file-lines-of-double - Hawaiyon
  // Ref.: https://stackoverflow.com/questions/30429120/converting-a-string-to-a-double-c

  ifstream infile ("~/zones.txt"); // It doesn't work or indicating the user's home directory this way
  // ifstream infile ("zones.txt");   // It also doesn't work looking for file in the same folder as add_markers.cpp
  // As usual.. the ROS WEB pages do not tell you how to do this successfully!!
  if (infile.is_open()){
    string str;
    ROS_INFO("Found zones.txt file"); 
    //read a line into 'str' from 'filein' each time
    getline(infile, str); // 1st line: gets pickup_zone_x
    pickup_zone_x = std::stod(str); // atof(str);
    getline(infile, str); // 2nd line: gets pickup_zone_y
    pickup_zone_y = std::stod(str);
    getline(infile, str); // 3th line: gets drop_off_zone_x
    drop_off_zone_x = std::stod(str);
    getline(infile, str); // 4th line: gets drop_off_zone_y
    drop_off_zone_y = std::stod(str);
    infile.close();
  }
}
*/

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
  int timeout = TIMEOUT; // limits number of repetitions (rates) of while ((ros::ok()) loop

  ros::init(argc, argv, "add_markers"); // ROS node name identification
  ros::NodeHandle n;

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Subscriber odom_sub = n.subscribe("odom", 2, OdomCheck);

  // file_read_zones(); // tries to recover zone data from zones.txt file
  ROS_INFO("Pickup  zone: x=%5.2f , y=%5.2f", pickup_zone_x, pickup_zone_y);
  ROS_INFO("Dropoff zone: x=%5.2f , y=%5.2f", drop_off_zone_x, drop_off_zone_y);

  // ros::Rate rate(1); // Try to run the loop while (ros::ok()) {..} below at the indicated frequency (1 Hz)
  ros::Rate rate(2); // doubling the previous speed (2 Hz)
 /* Really necessary!? It can make the execution of this routine extremely "slow" 
  * or even inefficient (broken, truncated) depending on the time it takes to run 
  * the internal loops.
  * Ref.: https://roboticsbackend.com/ros-rate-roscpy-roscpp/
  */
  while ((ros::ok())&&(timeout>0)){
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
      // timeout--; // this counter is expected to be decremented at the rate of 2 Hz - does not occour in this way!
      ros::spinOnce(); 
      /* ros::spin() and ros::spinOnce() are responsible to handle communication events, e.g. arriving
       * messages. If you are subscribing to messages, services, or actions, you must call 
       * spin to process the events.
       * While ros::spinOnce() handles the events and returns immediately, ros::spin() 
       * blocks until ROS invokes a shutdown
       */
       if (timeout<1) break; // gets out of loop - timeout occurred 
    }
    if (timeout<1){
      ROS_INFO("**Robot problems** to reach pickup zone, timeout reached (%d), wait aborted!", timeout);
    }
    else{
      timeout = TIMEOUT; // reset counter
      ROS_INFO("Robot reached pickup zone. Deleting pickup marker");
      marker.color.a = 0.1; marker_pub.publish(marker);
      ros::Duration(5.0).sleep();
      // ROS_INFO("Robot already collected material");  
      // Publishing second marker
      ROS_INFO("------");
      ROS_INFO("Displaying drop off zone");
      SetDropOffMarkerZone();
      marker.color.a = 0.6; marker_pub.publish(marker);
      ROS_INFO("Waiting for robot to reach drop off zone");
      while (!near_dropoff_zone){
        // timeout--;
        ros::spinOnce();
        if (timeout<1) break; // gets out of loop - timeout occurred
      }
      if (timeout<1){
        ROS_INFO("**Robot problems** to reach drop off zone, timeout reached (%d), wait aborted!", timeout);
      }
      else{
        ROS_INFO("Robot reached drop off zone. Deleting drop off marker");
        marker.color.a = 0.1; marker_pub.publish(marker);
        finish = true;        
      } 
    } // end of second stage, robot reaching delivery zone
    if (timeout<1){
      marker.color.a = 0.1; marker_pub.publish(marker); // it seems that ros "forgets" to execute last publish
      ROS_INFO("Sorry, there were problems for the robot to reach any of the goals");
      return 1; // means error to reach any of the goals (timeout)
    }
    timeout--;
    if (finish){
      ROS_INFO("Robot successfully completed its mission!");
      ROS_INFO("Shutting down add_markers node");
      break; // stops to run this node
    } 
    rate.sleep();
 /* rate.sleep () é apenas um thread de sono com uma duração definida por uma frequência.
  * Faz o looping "dormir" o que falta para completar o rate desejado. (parece pooling)
  * Ref.: https://stackoverflow.com/questions/23227024/difference-between-spin-and-rate-sleep-in-ros
  */
  } //  end while ((ros::ok())&&(timeout>0))
  return 0;
} // end main()
