#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <jpp/ParamsConfig.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <libconfig.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include "jpp/rmse.h"
#include "jpp.h"
#include "popt_pp.h"

JPP *jpp_obj;
JPP_Config jpp_config;
config_t cfg, *cf;
FileStorage calib_file;
const char* output = "astar";
int w = 0;
int v = 0;
int d = 0;
int counter = 0;
int print_counter = 0;
ofstream data_log;

float prev_y_total = 0;
vector< Point > prevPath;
bool path_invalid = true;

//publishers not in main
ros::Publisher pub_path;
ros::Publisher pub_surface_point_cloud;
ros::Publisher pub_surface_check_point_cloud;
ros::ServiceClient rmse_client;

void update_planned_path(vector< Point > path){
  //get the jpp generated path

  //define a path message from path to send to path_follower
  nav_msgs::Path real_path;
  real_path.header.frame_id = "jackal";
  real_path.header.stamp = ros::Time::now();

  path_invalid = true;

  float y_total = 0;
  for(uint32_t i = 0; i < path.size(); i++)
  {
    if (path[i].x > jpp_config.START_X)
    {
      if (path[i].x > jpp_config.START_X + ((jpp_config.MAX_X - jpp_config.START_X)*0.2))
        path_invalid = false;
      //need to divide by 1000 to convert from mm to m
      geometry_msgs::PoseStamped s_pose;
      s_pose.header = real_path.header;
      s_pose.pose.position.x = path[i].x / 1000.0;
      s_pose.pose.position.y = path[i].y / 1000.0;
      s_pose.pose.position.z = 0;
      s_pose.pose.orientation.x = 0;
      s_pose.pose.orientation.y = 0;
      s_pose.pose.orientation.z = 0;
      s_pose.pose.orientation.w = 1;

      y_total += s_pose.pose.position.y;

      real_path.poses.push_back(s_pose);
    }
  }

  if (path_invalid)//if the path is not valid stop, doing this by making the path just the 0 pose
  {
    //if invalid turn left or right
    geometry_msgs::PoseStamped s_pose;
    s_pose.header = real_path.header;
    s_pose.pose.position.x = 0;
    //s_pose.pose.position.y = 0;
    s_pose.pose.position.z = 0;
    s_pose.pose.orientation.x = 0;
    s_pose.pose.orientation.y = 0;
    s_pose.pose.orientation.z = 0;
    s_pose.pose.orientation.w = 1;

    if (prev_y_total > 0.0)//real_path.poses[0].pose.position.y > 0.0)
    {
      ROS_INFO("stop and turn left");
      s_pose.pose.position.y = 1.0;//turn left
    }
    else if (prev_y_total < 0.0)//real_path.poses[0].pose.position.y < 0.0)
    {
      ROS_INFO("stop and turn right");
      s_pose.pose.position.y = -1.0;//turn right
    }
    else
    {
      ROS_INFO("stop //should random trun, but for now left");
      s_pose.pose.position.y = 1.0;//should random trun, but for now left
    }

    real_path.poses.clear();
    real_path.poses.push_back(s_pose);
    ROS_INFO("real_path.poses[real_path.size()]: %f", real_path.poses[real_path.poses.size() - 1].pose.position.x);
  }
  else
  {
    prev_y_total = y_total;
  }

  pub_path.publish(real_path);
}

sensor_msgs::PointCloud update_surface(vector< pair< Point3f, float > > points)
{
  sensor_msgs::PointCloud point_cloud;
  point_cloud.header.frame_id = "jackal";
  point_cloud.header.stamp = ros::Time::now();

  sensor_msgs::ChannelFloat32 c;
  c.name = "intensity";

  for (int i = 0; i < points.size(); i++)
  {
    //filter for one slice
    // if (round(points[i].first.y * 1000.0) != 0.0)
    //   continue;
    geometry_msgs::Point32 p;
    p.x = points[i].first.x; //- 0.02;//ofset for visualization
    p.y = points[i].first.y;
    p.z = points[i].first.z;

    point_cloud.points.push_back(p);

    c.values.push_back(points[i].second);
  }

  point_cloud.channels.push_back(c);

  //pub_surface_point_cloud.publish(point_cloud);
  return point_cloud;
}

void update_surface_checks(vector< pair< Point3f, float > > confpos_points)
{
  sensor_msgs::PointCloud point_cloud;
  point_cloud.header.frame_id = "jackal";
  point_cloud.header.stamp = ros::Time::now();

  sensor_msgs::ChannelFloat32 c;
  c.name = "intensity";

  for (int i = 0; i < confpos_points.size(); i++)
  {
    //filter for one slice
    // if (round(confpos_points[i].first.y * 1000.0) != 0.0)
    //   continue;
    geometry_msgs::Point32 p;
    p.x = confpos_points[i].first.x;
    p.y = confpos_points[i].first.y;
    p.z = confpos_points[i].first.z;

    point_cloud.points.push_back(p);

    c.values.push_back(confpos_points[i].second);
  }

  point_cloud.channels.push_back(c);

  pub_surface_check_point_cloud.publish(point_cloud);
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right)
{
  counter++;
  Mat img_left = cv_bridge::toCvShare(msg_left, "bgr8")->image;
  Mat img_right = cv_bridge::toCvShare(msg_right, "bgr8")->image;
  if (img_left.empty() || img_right.empty())
    return;
  
  char rrt_file_prefix[100], astar_file_prefix[100];
  sprintf(astar_file_prefix, "%s%d", "astar", counter);
  sprintf(rrt_file_prefix, "%s%d", "rrt", counter);
  
  if (d == 1)
    jpp_obj->update_jpp_config(jpp_config);
  
  jpp_obj->load_images(img_left, img_right);
  
  if (strcmp(output, "astar") == 0) {
    //jpp_obj->start_search_space_viz();
    jpp_obj->start_disparity_counter();
    vector< Point > path;
    if (path_invalid)
      path = jpp_obj->plan_astar();
    else
      path = jpp_obj->plan_astar(prevPath);
    //vector< Point > path = jpp_obj->plan_astar(prevPath);
    prevPath = path;
    int disparity_count = jpp_obj->get_disparity_count();
    printf("jpp num_disparity_checks: %d, ", disparity_count);
    //calculate path length:
    float path_length = 0;
    if (path.size() > 1)
    {
      for (uint i = 1; i < path.size(); i++)
      {
        float change_y = fabs(path[i - 1].x - path[i].x)/1000.0;
        float change_x = fabs(path[i - 1].y - path[i].y)/1000.0;
        float newLength = sqrt(change_x*change_x + change_y*change_y);
        path_length += newLength;
      }
    }
    printf("path_length: %f, ", path_length);
    print_counter++;

    update_planned_path(jpp_obj->getPath());
    sensor_msgs::PointCloud point_cloud = update_surface(jpp_obj->get_surface_points());
    update_surface_checks(jpp_obj->get_surface_checks());

    //jpp_obj->search_space_viz();

    jpp::rmse srv;
    srv.request.L = *msg_left;
    srv.request.R = *msg_right;
    srv.request.pc = point_cloud;
    if (rmse_client.call(srv))
    {
      printf("rmse: %f, %d\n", srv.response.rmse, print_counter);
    }
    else
    {
      printf("rmse: failure, %d\n", print_counter);
    }
    pub_surface_point_cloud.publish(point_cloud);

    //data_log << disparity_count << ", " << path_length << ", " << srv.response.rmse << "\n";

    // jpp_obj->start_disparity_counter();
    // jpp_obj->get_disparity_map("nothing", 30, NULL);
    // printf("spp num_disparity_checks: %d\n", jpp_obj->get_disparity_count());

    // update_planned_path(jpp_obj->getPath());
    // update_surface(jpp_obj->get_surface_points());
    // update_surface_checks(jpp_obj->get_surface_checks());
    if (v == 1) {
      pair< Mat, Mat > vis;
      if (w == 1)
        vis = jpp_obj->visualize_jpp(astar_file_prefix);
      else
        vis = jpp_obj->visualize_jpp();
      imshow("PATH", vis.first);
      imshow("CONFIDENCE", vis.second);
    }
  } else if (strcmp(output, "rrt") == 0) {
    vector< Point > path = jpp_obj->plan_rrt();
    update_planned_path(jpp_obj->getPath());
    //update_surface(jpp_obj->get_surface_points());
    if (v == 1) {
      pair< Mat, Mat > vis;
      if (w == 1)
        vis = jpp_obj->visualize_jpp(rrt_file_prefix);
      else
        vis = jpp_obj->visualize_jpp();
      imshow("PATH", vis.first);
      imshow("CONFIDENCE", vis.second);
    }
  } else if (strcmp(output, "debug") == 0) {
    Mat conf_pos = jpp_obj->visualize_conf_pos();
    Mat conf_neg = jpp_obj->visualize_conf_neg();
    imshow("CONF POS", conf_pos);
    imshow("CONF NEG", conf_neg);
  }
  waitKey(30);
}

void paramsCallback(jpp::ParamsConfig &conf, uint32_t level) {
  jpp_config.DR = conf.DR;
  jpp_config.DQ = conf.DQ;
  jpp_config.DT = conf.DT;
  jpp_config.DH = conf.DH;
  jpp_config.BOT_LENGTH = conf.LENGTH;
  jpp_config.BOT_WIDTH = conf.WIDTH;
  jpp_config.BOT_HEIGHT = conf.HEIGHT;
  jpp_config.GRID_SIZE = conf.GRID_SIZE;
  jpp_config.CONF_POS_THRESH = conf.CONF_POS_THRESH;
  jpp_config.CONF_NEG_THRESH = conf.CONF_NEG_THRESH;
  jpp_config.SAD_WINDOW_SIZE = conf.SAD_WINDOW_SIZE;
  jpp_config.SPATIAL_FILTER_WINDOW = conf.SPATIAL_FILTER_WINDOW;
  jpp_config.SPATIAL_FILTER_INC = conf.SPATIAL_FILTER_INC;
  jpp_config.SPATIAL_FILTER_RATIO = conf.SPATIAL_FILTER_RATIO;
  jpp_config.CONF_NEG_INC = conf.CONF_NEG_INC;
  jpp_config.CONF_NEG_FILTER_RATIO = conf.CONF_NEG_FILTER_RATIO;
  jpp_config.START_X = conf.START_X;
  jpp_config.MAX_X = conf.MAX_X;
  jpp_config.MAX_Y = conf.MAX_Y;
  jpp_config.CONVEX_WORLD = conf.CONVEX_WORLD;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "jpp_navigation");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  data_log.open ("jpp_log.txt");

  pub_path = nh.advertise<nav_msgs::Path>("/jackal/planned_path", 1);
  pub_surface_point_cloud = nh.advertise<sensor_msgs::PointCloud>("/jackal/surface", 1);
  pub_surface_check_point_cloud = nh.advertise<sensor_msgs::PointCloud>("/jackal/surface_check", 1);
  rmse_client = nh.serviceClient<jpp::rmse>("/stereo_dense_reconstruction/rootMeanSquareError");
  
  const char* left_img_topic;
  const char* right_img_topic;
  const char* calib_file_name;
  const char* jpp_config_file;
  
  static struct poptOption options[] = {
    { "left_topic",'l',POPT_ARG_STRING,&left_img_topic,0,"Left image topic name","STR" },
    { "right_topic",'r',POPT_ARG_STRING,&right_img_topic,0,"Right image topic name","STR" },
    { "calib_file",'c',POPT_ARG_STRING,&calib_file_name,0,"Stereo calibration file name","STR" },
    { "jpp_config_file",'j',POPT_ARG_STRING,&jpp_config_file,0,"JPP config file name","STR" },
    { "output",'o',POPT_ARG_STRING,&output,0,"Output - astar, rrt, debug","STR" },
    { "visualize",'v',POPT_ARG_INT,&v,0,"Set v=1 for displaying visualizations","NUM" },
    { "write_files",'w',POPT_ARG_INT,&w,0,"Set w=1 for writing visualizations to files","NUM" },
    { "dynamic_reconfigure",'d',POPT_ARG_INT,&d,0,"Set d=1 for enabling dynamic reconfigure","NUM" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}
  
  calib_file = FileStorage(calib_file_name, FileStorage::READ);
  
  // Read JPP config
  cf = &cfg;
  config_init(cf);
  if (!config_read_file(cf, jpp_config_file)) {
    cout << "Could not read config file!" << endl;
    config_destroy(cf);
    return(EXIT_FAILURE);
  }
  jpp_config = JPP_Config(cf);
  
  jpp_obj = new JPP(calib_file, cf);
  
  message_filters::Subscriber<sensor_msgs::Image> sub_img_left(nh, left_img_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_right(nh, right_img_topic, 1);
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img_left, sub_img_right);
  sync.registerCallback(boost::bind(&imgCallback, _1, _2));
  
  dynamic_reconfigure::Server<jpp::ParamsConfig> server;
  dynamic_reconfigure::Server<jpp::ParamsConfig>::CallbackType f;
  f = boost::bind(&paramsCallback, _1, _2);
  server.setCallback(f);

  ros::spin();
  data_log.close();
  return 0;
}