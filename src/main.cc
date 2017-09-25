#include "popt_pp.h"
#include "jpp.h"

int main(int argc, const char** argv) {
  const char* left_img_file;
  const char* right_img_file;
  const char* calib_file;
  const char* jpp_config_file;
  const char* img_dir;
  const char* output = "astar";
  int w = 0;
  int num_imgs = 0;
  
  static struct poptOption options[] = {
    { "num_imgs",'n',POPT_ARG_INT,&num_imgs,0,"Number of images to be processed","NUM" },
    { "img_dir",'d',POPT_ARG_STRING,&img_dir,0,"Directory containing image pairs (set if n > 0)","STR" },
    { "left_img",'l',POPT_ARG_STRING,&left_img_file,0,"Left image file name","STR" },
    { "right_img",'r',POPT_ARG_STRING,&right_img_file,0,"Right image file name","STR" },
    { "calib_file",'c',POPT_ARG_STRING,&calib_file,0,"Stereo calibration file name","STR" },
    { "jpp_config_file",'j',POPT_ARG_STRING,&jpp_config_file,0,"JPP config file name","STR" },
    { "output",'o',POPT_ARG_STRING,&output,0,"Output - astar, rrt, debug","STR" },
    { "write_files",'w',POPT_ARG_INT,&w,0,"Set w=1 for writing visualizations to files","NUM" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}
  
  // Read JPP config
  config_t cfg, *cf;
  cf = &cfg;
  config_init(cf);
  if (!config_read_file(cf, jpp_config_file)) {
    cout << "Could not read config file!" << endl;
    config_destroy(cf);
    return(EXIT_FAILURE);
  }
  
  // Read stereo calibration info
  FileStorage fs = FileStorage(calib_file, FileStorage::READ);
  
  if (num_imgs == 0) {
    Mat img_left = imread(left_img_file, CV_LOAD_IMAGE_COLOR);
    Mat img_right = imread(right_img_file, CV_LOAD_IMAGE_COLOR);
    
    char dmap_file[100], rrt_file_prefix[100], astar_file_prefix[100];
    sprintf(astar_file_prefix, "%s%d", "astar", 0);
    sprintf(rrt_file_prefix, "%s%d", "rrt", 0);
    sprintf(dmap_file, "%s%d.jpg", "dmap", 0);
    
    JPP jpp(img_left, img_right, fs, cf);
    
    if (strcmp(output, "astar") == 0) {
      pair< Mat, Mat > vis;
      if (w == 1) 
        vis = jpp.plan_astar(astar_file_prefix);
      else
        vis = jpp.plan_astar();
      while (1) {
        imshow("PATH", vis.first);
        imshow("CONFIDENCE MATCHING", vis.second);
        if (waitKey(30) > 0) {
          break;
        }
      }
    } else if (strcmp(output, "rrt") == 0) {
      pair< Mat, Mat > vis;
      if (w == 1) 
        vis = jpp.plan_rrt(rrt_file_prefix);
      else
        vis = jpp.plan_rrt();
      while (1) {
        imshow("PATH", vis.first);
        imshow("CONFIDENCE MATCHING", vis.second);
        if (waitKey(30) > 0) {
          break;
        }
      }
    } else if (strcmp(output, "debug") == 0) {
      Mat conf_pos = jpp.visualize_conf_pos();
      Mat conf_neg = jpp.visualize_conf_neg();
      while (1) {
        imshow("CONF POS", conf_pos);
        imshow("CONF NEG", conf_neg);
        if (waitKey(30) > 0) {
          break;
        }
      }
    }
  } else if (num_imgs > 0) {
    for (int i = 1; i <= num_imgs; i++) {
      cout << "Processing pair " << i << endl;
      char left_img_file[100], right_img_file[100];
      char dmap_file[100], rrt_file_prefix[100], astar_file_prefix[100];
      sprintf(left_img_file, "%s%s%d.jpg", img_dir, "left", i);
      sprintf(right_img_file, "%s%s%d.jpg", img_dir, "right", i);
      sprintf(dmap_file, "%s%d.jpg", "dmap", i);
      sprintf(astar_file_prefix, "%s%d", "astar", i);
      sprintf(rrt_file_prefix, "%s%d", "rrt", i);
      
      Mat img_left = imread(left_img_file, CV_LOAD_IMAGE_COLOR);
      Mat img_right = imread(right_img_file, CV_LOAD_IMAGE_COLOR);
      
      JPP jpp(img_left, img_right, fs, cf);
      
      if (strcmp(output, "astar") == 0) {
        pair< Mat, Mat > vis;
        if (w == 1) 
          vis = jpp.plan_astar(astar_file_prefix);
        else
          vis = jpp.plan_astar();
        while (1) {
          imshow("PATH", vis.first);
          imshow("CONFIDENCE MATCHING", vis.second);
          if (waitKey(30) > 0) {
            break;
          }
        }
      } else if (strcmp(output, "rrt") == 0) {
        pair< Mat, Mat > vis;
        if (w == 1) 
          vis = jpp.plan_rrt(rrt_file_prefix);
        else
          vis = jpp.plan_rrt();
        while (1) {
          imshow("PATH", vis.first);
          imshow("CONFIDENCE MATCHING", vis.second);
          if (waitKey(30) > 0) {
            break;
          }
        }
      } else if (strcmp(output, "debug") == 0) {
        Mat conf_pos = jpp.visualize_conf_pos();
        Mat conf_neg = jpp.visualize_conf_neg();
        while (1) {
          imshow("CONF POS", conf_pos);
          imshow("CONF NEG", conf_neg);
          if (waitKey(30) > 0) {
            break;
          }
        }
      }
    }
  }
  
  config_destroy(cf);
  return 0;
}
