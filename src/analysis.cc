#include "jpp.h"
#include "popt_pp.h"

JPP *jpp;

void compute_epipolar_costs() {
  int max_disp = 50;
  ofstream data_file;
  data_file.open("epipolar_costs.txt");
  data_file << max_disp << endl;
  Mat ground_truth_dmap = jpp->get_disparity_map("spp", max_disp);
  int start_x = ground_truth_dmap.cols/2 - 50;
  int end_x = ground_truth_dmap.cols/2 + 50;
  int start_y = ground_truth_dmap.rows/2 - 50;
  int end_y = ground_truth_dmap.rows/2 + 50;
  for (int i = start_x; i < end_x; i++) {
    for (int j = start_y; j < end_y; j++) {
      cout << i << ", " << j << endl;
      vector< double > costs = jpp->get_epipolar_costs(Point(i, j), max_disp);
      for (int k = 0; k < costs.size(); k++) {
        data_file << costs[k] << " ";
      }
      data_file << ((int)ground_truth_dmap.at<uchar>(j, i)) << endl;
    }
  }
}

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
  
  JPP jpp(fs, cf);
  
  if (num_imgs == 0) {
    Mat img_left = imread(left_img_file, CV_LOAD_IMAGE_COLOR);
    Mat img_right = imread(right_img_file, CV_LOAD_IMAGE_COLOR);
    
    char dmap_file[100], rrt_file_prefix[100], astar_file_prefix[100];
    sprintf(astar_file_prefix, "%s%d", "astar", 0);
    sprintf(rrt_file_prefix, "%s%d", "rrt", 0);
    sprintf(dmap_file, "%s%d.jpg", "dmap", 0);
    
    jpp.load_images(img_left, img_right);
    /*
    int max_disp = 50;
    Mat dmap = jpp->get_disparity_map("spp", max_disp);
    for (int i = 0; i < dmap.cols; i++) {
      for (int j = 0; j < dmap.rows; j++) {
        int d = (int)dmap.at<uchar>(j, i);
        dmap.at<uchar>(j, i) = (int)(((double)d/(double)max_disp)*255.0);
      }
    }
    while (1) {
      imshow("DMAP", dmap);
      imshow("LEFT", jpp->get_img_left());
      if (waitKey(30) > 0) {
        break;
      }
    }
    */
    compute_epipolar_costs();
    
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
      
      jpp.load_images(img_left, img_right);
    }
  }
  
  config_destroy(cf);
  return 0;
}