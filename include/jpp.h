#ifndef JPP_H
#define JPP_H

#include "astarplanner.h"
#include "rrt.h"

class JPP {
private:
  JPP_Config _jpp_config;
  Stereo *_stereo;
  
  void _reset();
  void _get_jpp_config(config_t *cf);
  vector< Point > _path;
public:
  JPP();
  JPP(FileStorage& fs, config_t *cf);
  ~JPP();
  void load_images(const Mat& left, const Mat& right);
  Mat get_disparity_map(const char* method, int max_disp, const char* outfile = NULL);
  Mat visualize_conf_pos();
  Mat visualize_conf_neg();
  Mat visualize_empty_cols();
  vector< Point > plan_astar();
  vector< Point > plan_astar(vector< Point > prevPath);
  vector< Point > plan_rrt();
  void start_disparity_counter();
  int get_disparity_count();
  pair< Mat, Mat > visualize_jpp(const char* outfile = NULL);
  vector< Point > getPath();
  vector< pair< Point3f, float > > get_surface_points();
  vector< pair< Point3f, float > > get_surface_checks();
  void update_jpp_config(JPP_Config& config);
  vector< double > get_epipolar_costs(Point p, int max_disp);
  Mat get_img_left();
  Mat get_img_right();
  Mat get_Q_matrix();
};

#endif // JPP_H