#ifndef JPP_H
#define JPP_H

#include "astarplanner.h"
#include "rrt.h"

class JPP {
private:
  JPP_Config _jpp_config;
  Stereo *_stereo;
  
  void _get_jpp_config(config_t *cf);
public:
  JPP();
  JPP(Mat& img_left, Mat& img_right, FileStorage& fs, config_t *cf);
  ~JPP();
  Mat get_disparity_map(const char* method, int max_disp, const char* outfile = NULL);
  Mat visualize_conf_pos();
  Mat visualize_conf_neg();
  Mat visualize_empty_cols();
  pair< Mat, Mat > plan_astar(const char* outfile = NULL);
  pair< Mat, Mat > plan_rrt(const char* outfile = NULL);
  void update_jpp_config(JPP_Config& config);
  vector< double > get_epipolar_costs(Point p, int max_disp);
  void reset();
  Mat get_img_left();
  Mat get_img_right();
  Mat get_Q_matrix();
};

#endif // JPP_H