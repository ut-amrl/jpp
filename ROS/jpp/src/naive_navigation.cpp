#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <jpp/ParamsConfig.h>
#include <fstream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "disparitydaisy.h"

using namespace cv;
using namespace std;

Mat img_left, img_right;
Mat XR, XT, Q, P1, P2;
Mat R1, R2, K1, K2, D1, D2, R;
Mat lmapx, lmapy, rmapx, rmapy;
Vec3d T;
jpp::ParamsConfig config;
FileStorage calib_file;
Size out_img_size(320, 180); //kitti - 489x180, 1392x512
Size calib_img_size(640, 360);

ros::Publisher vel_pub;
double forward_vel = 0., rot_vel = 0.;
double trans_accel = 0.025; // forward acceleration
double trans_decel = 0.1; // forward deceleration
double rot_accel = 0.05; // rotational acceleration
float max_forward_vel = 0.6; // maximum forward velocity
double max_rot_vel = 1.3; // maximum rotational velocity
bool is_obstacle = false;

void undistortRectifyImage(Mat& src, Mat& dst, FileStorage& calib_file, int 
left = 1) {
  if (left == 1) {
    remap(src, dst, lmapx, lmapy, cv::INTER_LINEAR);
  } else {
    remap(src, dst, rmapx, rmapy, cv::INTER_LINEAR);
  }
}

void blend_images(Mat& src1, Mat& src2, float alpha, Mat& dst) {
  float beta = ( 1.0 - alpha );
  addWeighted( src1, alpha, src2, beta, 0.0, dst);
}

void visualizeConfPos(Mat& img, DisparityDaisy *daisy) {
  clock_t begin = clock();
  Point start(config.START_X, -config.MAX_Y);
  Point end(config.MAX_X, config.MAX_Y);
  int inc = config.GRID_SIZE;
  float safe_radius = (float)max(config.LENGTH/2, config.WIDTH/2)/1000.;
  #pragma omp parallel for collapse(2)\
  num_threads(omp_get_max_threads())
  for (int x = start.x; x <= end.x; x += inc) {
    for (int y = start.y; y <= end.y; y += inc) {
      Point3f p((float)x/1000.,(float)y/1000.,0.0);
      Point ptl = daisy->projectPointCam(p, 0);
      //if (daisy->confPositive(p)) {
      //if (daisy->isEmptySpace(p)) {
      //if (daisy->isObstacleFreeRegion(p)) {
      if (daisy->isBotClearOfObstacle(p, safe_radius, 
(float)config.GRID_SIZE/1000., false)) {
        Scalar green = Scalar(0,255,0);
        circle(img, ptl, 3, green, -1, 8, 0);
      }
    }
  }
  clock_t stop = clock();
  double elapsed_secs = double(stop - begin) / CLOCKS_PER_SEC;
  cout << "conf pos time: " << elapsed_secs << endl;
}

void visualizeConfNeg(Mat& img, DisparityDaisy *daisy) {
  clock_t begin = clock();
  Point start(config.START_X, -config.MAX_Y);
  Point end(config.MAX_X, config.MAX_Y);
  int inc = config.GRID_SIZE;
  float safe_radius = (float)max(config.LENGTH/2, config.WIDTH/2)/1000.;
  #pragma omp parallel for collapse(3)\
  num_threads(omp_get_max_threads())
  for (int x = start.x; x <= end.x; x += inc) {
    for (int y = start.y; y <= end.y; y += inc) {
      for (int z = config.CONF_NEG_INC; z <= config.HEIGHT; z += 
config.CONF_NEG_INC) {
        Point3f q((float)x/1000.,(float)y/1000.,(float)z/1000.);
        Point qtl = daisy->projectPointCam(q, 0);
        if (!daisy->confNegative(q)) {
          circle(img, qtl, 1, Scalar(0,255,0), -1, 8, 0);
        } else {
          circle(img, qtl, 1, Scalar(0,0,255), -1, 8, 0);
        }
      }
    }
  }
  clock_t stop = clock();
  double elapsed_secs = double(stop - begin) / CLOCKS_PER_SEC;
  cout << "conf neg time: " << elapsed_secs << endl;
}

void imgCallback(const sensor_msgs::CompressedImageConstPtr& msg_left, const 
sensor_msgs::CompressedImageConstPtr& msg_right)
{
  Mat tmp1 = cv::imdecode(cv::Mat(msg_left->data), CV_LOAD_IMAGE_GRAYSCALE);
  Mat tmp2 = cv::imdecode(cv::Mat(msg_right->data), 
CV_LOAD_IMAGE_GRAYSCALE);
  undistortRectifyImage(tmp1, img_left, calib_file, 1);
  undistortRectifyImage(tmp2, img_right, calib_file, 0);
  if (img_left.empty() || img_right.empty())
    return;
  
  DisparityDaisy *daisy = new DisparityDaisy(config);
  daisy->loadImages(img_left, img_right);
  daisy->initDaisyDesc(config.DR, config.DQ, config.DT, config.DH, NRM_FULL);
  daisy->setCamParams(XR, XT, Q, P1, P2);
  Mat img_left_disp;
  cvtColor(img_left, img_left_disp, CV_GRAY2BGR);
  Point start(config.START_X, -config.MAX_Y);
  Point end(config.MAX_X, config.MAX_Y);
  int inc = config.GRID_SIZE;
  float safe_radius = (float)max(config.LENGTH/2, config.WIDTH/2)/1000.;
  double safe_points = 0;
  double total_points = 0;
  for (int x = start.x; x <= end.x; x += inc) {
    for (int y = start.y; y <= end.y; y += inc) {
      Point3f p((float)x/1000.,(float)y/1000.,0.0);
      Point ptl = daisy->projectPointCam(p, 0);
      //if (daisy->confPositive(p)) {
      //if (daisy->isEmptySpace(p)) {
      //if (daisy->isObstacleFreeRegion(p)) {
      if (daisy->isBotClearOfObstacle(p, safe_radius, 
(float)config.GRID_SIZE/1000., false)) {
        Scalar green = Scalar(0,255,0);
        circle(img_left_disp, ptl, 3, green, -1, 8, 0);
        safe_points += 1.0;
      }
      total_points += 1.0;
    }
  }
  double ratio = safe_points / total_points;
  if (ratio > 0.6) {
    is_obstacle = false;
  } else {
    is_obstacle = true;
    cout << ratio << " obstacle!" << endl;
  }
  imshow("VIS", img_left_disp);
  waitKey(30);
  
  delete daisy;
}

void findRectificationMap(FileStorage& calib_file, Size finalSize) {
  Rect validRoi[2];
  cout << "starting rectification" << endl;
  stereoRectify(K1, D1, K2, D2, calib_img_size, R, Mat(T), R1, R2, P1, P2, Q, 
                CV_CALIB_ZERO_DISPARITY, 0, finalSize, &validRoi[0], 
&validRoi[1]);
  cv::initUndistortRectifyMap(K1, D1, R1, P1, finalSize, CV_32F, lmapx, 
lmapy);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, finalSize, CV_32F, rmapx, 
rmapy);
  cout << "done rectification" << endl;
}

void paramsCallback(jpp::ParamsConfig &conf, uint32_t level) {
  config = conf;
}

pair< double, double > stopInFrontMode(double side, double front) {
  double desired_forward_vel = max_forward_vel * front;
  double desired_rot_vel = max_rot_vel * side;
  if (is_obstacle) {
    // if there's an obstacle stop or allow the jackal to move back
    desired_forward_vel = min(desired_forward_vel, 0.);
  }
  return make_pair(desired_forward_vel, desired_rot_vel);
}

void safeNavigate(const sensor_msgs::JoyConstPtr& msg) {
  // read joystick input
  int R2 = msg->buttons[9];
  int R1 = msg->buttons[11];
  int X = msg->buttons[14];
  int O = msg->buttons[13];
  int triangle = msg->buttons[12];
  double side = msg->axes[0];
  double front = msg->axes[1];
  double desired_forward_vel, desired_rot_vel;
  pair< double, double > desired_vel;
  // run the different modes
  if (R1 && R2) {
    desired_vel = stopInFrontMode(side, front);
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
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = forward_vel;
  vel_msg.angular.z = rot_vel;
  vel_pub.publish(vel_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "jpp_naive_navigation");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  
  calib_file = FileStorage(argv[3], FileStorage::READ);
  calib_file["K1"] >> K1;
  calib_file["K2"] >> K2;
  calib_file["D1"] >> D1;
  calib_file["D2"] >> D2;
  calib_file["R"] >> R;
  calib_file["T"] >> T;
  calib_file["XR"] >> XR;
  calib_file["XT"] >> XT;
  findRectificationMap(calib_file, out_img_size);
  
  message_filters::Subscriber<sensor_msgs::CompressedImage> sub_img_left(nh, 
argv[1], 1);
  message_filters::Subscriber<sensor_msgs::CompressedImage> sub_img_right(nh, 
argv[2], 1);
  
  ros::Subscriber sub_safe_drive = nh.subscribe("/bluetooth_teleop/joy", 1, 
safeNavigate);
  vel_pub = 
nh.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);
  
  typedef 
message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, 
sensor_msgs::CompressedImage> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), 
sub_img_left, sub_img_right);
  sync.registerCallback(boost::bind(&imgCallback, _1, _2));
  
  dynamic_reconfigure::Server<jpp::ParamsConfig> server;
  dynamic_reconfigure::Server<jpp::ParamsConfig>::CallbackType 
f;

  f = boost::bind(&paramsCallback, _1, _2);
  server.setCallback(f);

  ros::spin();
  return 0;
}