#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>//there is a more specific lib
#include <libconfig.h>
#include "jpp.h"
#include "popt_pp.h"

JPP_Config jpp_config;
config_t cfg, *cf;
FileStorage calib_file;
const char* output = "astar";
int w = 0;
int counter = 0;

typedef nav_msgs::Path Path;
typedef geometry_msgs::Point GPoint;

Path path;
int path_index = -1;
geometry_msgs::Pose path_init_pose;
geometry_msgs::Pose current_pose;
tf::Pose tf_path_init_pose;
tf::Pose tf_current_pose;
geometry_msgs::Twist current_twist;

//defining path for rviz
vector< GPoint > rvizPath;

//publishers not in main
ros::Publisher vel_pub;
ros::Publisher path_pub;

//constants
double forward_vel = 0., rot_vel = 0.;
double trans_accel = 0.025; // forward acceleration
double trans_decel = 0.1; // forward deceleration
double rot_accel = 0.05; // rotational acceleration
float max_forward_vel = 0.2;//0.6; // maximum forward velocity
double max_rot_vel = 0.5;//1.3; // maximum rotational velocity
float dist_threshold = 0.045;//0.045;//near enough distance

void newPathCallBack(const nav_msgs::Path::ConstPtr& p){
  //get the jpp generated path

  path.header = p->header;
  path.poses = p->poses;
  path_index = path.poses.size() - 1;
  //ROS_INFO("path pose: %d", path.poses.size());
  path_init_pose = current_pose;
  tf_path_init_pose = tf_current_pose;


  //initulizing ros path marker
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "jackal";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "ros_path";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  //fill in line_strip from path
  for(uint32_t i = 0; i < path.poses.size(); i++)
  {
    line_strip.points.push_back(path.poses[i].pose.position);
  }

  //publish
  path_pub.publish(line_strip);
}

//transforms a pose into the reference frame of the robot from odometry
geometry_msgs::Pose transform_pose(geometry_msgs::Pose pose)
{
  tf::Transform trans = tf_current_pose.inverseTimes(tf_path_init_pose);

  tf::Pose tf_pose;
  tf::poseMsgToTF(pose, tf_pose);

  tf_pose = trans*tf_pose;

  geometry_msgs::Pose new_pose;
  tf::poseTFToMsg(tf_pose, new_pose);//getting warnings


  return new_pose;
}

void odometry_call_back(const nav_msgs::Odometry& odom)
{
  current_twist = odom.twist.twist;
  current_pose = odom.pose.pose;
  tf::poseMsgToTF(odom.pose.pose, tf_current_pose);

  //initulizing ros path marker
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "jackal";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "ros_path";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  //fill in line_strip from path
  for(uint32_t i = 0; i < path.poses.size(); i++)
  {
    line_strip.points.push_back(transform_pose(path.poses[i].pose).position);
  }

  //publish
  path_pub.publish(line_strip);
}

float distance_to_point(geometry_msgs::Point p){
  return sqrt(p.x*p.x + p.y*p.y);
}

float angle_to_point(geometry_msgs::Point p){
  return atan(p.y/p.x);
}

pair< double, double > bestVelocity(float radius, int sign)
{
	pair< double, double > velocity (0,0);
	if (radius == -1)
	{
		velocity.first = max_forward_vel;
	  velocity.second = 0;
	}
	else if (radius != 0)
	{
	   velocity.first = max_forward_vel;
	   velocity.second = (max_forward_vel/radius) * sign;
	}
	return velocity;
}

pair< double, double > goto_pose(geometry_msgs::Pose pose)
{
  //for now orientation is egnored
  pair< double, double > velocity(0,0);

  //ROS_INFO("pose.position.x: %f", pose.position.x);
  if (pose.position.x <= 0.01)
  {
    //ROS_INFO("stop!");
    velocity.first = 0.0;
  }
  else 
  {
    //ROS_INFO("go as you please");
    velocity.first = max_forward_vel;
  }

  geometry_msgs::Pose gPose = transform_pose(pose);

  velocity.second = (0.8 * angle_to_point(gPose.position));
  if (velocity.second > 0)
    velocity.second = min(velocity.second, max_rot_vel);
  else
    velocity.second = max(velocity.second, -max_rot_vel);

  return velocity;

}

pair< double, double > follow_path()
{
  //ROS_INFO("dist to point: %f\n", distance_to_point(path.poses[path_index].pose.position));
  //^causes segmentation fault if no path
  if (path_index < 0 || path.poses.size() <= 0)
  {
    return(pair< double, double > (0,0));
  }
  else if (distance_to_point(path.poses[path_index].pose.position) < dist_threshold)
  {
    path_index--;
    return follow_path();
  }
  else 
  {
    //ROS_INFO("path.poses.size(): %d", path.poses.size());
    //ROS_INFO("index: %d", path_index);
    return goto_pose(path.poses[path_index].pose);//transform_pose(path.poses[path_index].pose));
  }
  return(pair< double, double > (0,0));
}

void safeNavigate(const sensor_msgs::JoyConstPtr& msg) {
  // read joystick input
  int R2 = msg->buttons[9];
  int R1 = msg->buttons[11];
  int X = msg->buttons[14];
  int O = msg->buttons[13];
  int triangle = msg->buttons[12];
  //double side = msg->axes[0];
  //double front = msg->axes[1];
  double desired_forward_vel, desired_rot_vel;
  pair< double, double > desired_vel;
  // run the different modes
  if (R1 && R2) {
    //desired_vel = stopInFrontMode(side, front);
  } else if (triangle) {
    desired_vel = follow_path();
    ROS_INFO("wanted V: %f, W: %f", forward_vel, rot_vel);
    //desired_vel = autoNavigateMode(front); // navigation doesn't work yet
  } else if (X) {
    //desired_vel = obstacleAvoidMode(front);
  } else if (O) {
    //desired_vel = stopInFrontMode();
  } else {
    return;
  }
  // accelerate or decelerate accordingly
  desired_forward_vel = desired_vel.first;
  desired_rot_vel = desired_vel.second;
  if (desired_forward_vel < forward_vel) {
    forward_vel = max(desired_forward_vel, forward_vel - trans_decel);
  } else {
    forward_vel = min(desired_forward_vel, forward_vel + trans_accel);
  }
  if (desired_rot_vel < rot_vel) {
    rot_vel = max(desired_rot_vel, rot_vel - rot_accel);
  } else {
    rot_vel = min(desired_rot_vel, rot_vel + rot_accel);
  }
  ROS_INFO("real V: %f, W: %f", forward_vel, rot_vel);
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = forward_vel;
  vel_msg.angular.z = rot_vel;
  vel_pub.publish(vel_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_follower");
  ros::NodeHandle nh;

  path_pub = nh.advertise<visualization_msgs::Marker>("path_marker", 10);
  vel_pub = nh.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);
  // /jackal_velocity_controller/cmd_vel

  ros::Subscriber sub_safe_drive = nh.subscribe("/bluetooth_teleop/joy", 1, safeNavigate);
  ros::Subscriber sub_path = nh.subscribe("/jackal/planned_path", 1, newPathCallBack);
  ros::Subscriber sub_odometry = nh.subscribe("/odometry/filtered", 1, odometry_call_back);

  ros::spin();

  return 0;
}