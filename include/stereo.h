#ifndef STEREO_H
#define STEREO_H

#include "util.h"

using namespace std;
using namespace cv;
using namespace kutility;

class Stereo {
private:
  JPP_Config _jpp_config;
  Mat _img_left, _img_right;
  Mat _XR, _XRINV, _XT, _Q, _P1, _P2;
  Mat _R1, _R2, _K1, _K2, _D1, _D2, _R;
  Mat _lmapx, _lmapy, _rmapx, _rmapy;
  Vec3d _T;
  Eigen::Matrix3f _cam2robot_R;
  Eigen::Vector3f _cam2robot_T;
  Eigen::Matrix< float, 3, 4 > _eP1, _eP2;
  daisy *_desc_left, *_desc_right;
  Mat _disparityMap;
  vector< int > _obstacleCache;
  vector< int > _obstacleRangeCache;
  vector< int > _colCache;
  vector< int > _confNegCache;
  vector< int > _descLeftSet, _descRightSet;
  vector< float* > _descLeftCache, _descRightCache;
  Mat _cacheVis;
  int num_disparity_checks;
  //2-dim array of surface_p struct to keep track of confpos checks
  typedef struct _confident_z
  {
    float z;
    double cost;
  }confident_z;

  typedef struct _surface_p
  {
    bool valid;
    bool discovered;
    bool median_filtered;
    bool layer_median_filtered;
    bool slope_change_calculated;
    float z;
    float median_filtered_z;
    float layer_median_filtered_z;
    float slope_change;
    vector< confident_z > confident_Zvalues;
    vector< confident_z > confident_Zvalues2;
    vector< pair< Point3f, float > > confpos;
    _surface_p(bool v, bool d, bool mf, bool lmf, bool scc, float zed, float mfzed, float lmfzed, float sc)
    {
      valid = v;
      discovered = d;
      median_filtered = mf;
      layer_median_filtered = lmf;
      slope_change_calculated = scc;
      z = zed;
      median_filtered_z = mfzed;
      layer_median_filtered_z = lmfzed;
      slope_change = sc;
    }
    void insert_confident_z(confident_z cz)
    {
      //niave imiplementation::
      for(vector< confident_z >::iterator it = confident_Zvalues.begin(); it != confident_Zvalues.end(); ++it)
      {
        if (cz.cost < (*it).cost)
        {
          confident_Zvalues.insert(it, cz);
          return;
        }
      }
      confident_Zvalues.push_back(cz);

      //use binary search and insert
      /*if (confident_Zvalues.empty())
      {
        confident_Zvalues.push_back(cp);
      }
      else
      {
        int index = (confident_Zvalues.size() - 1)/2;
        while(true)
        {
          if (index + 1 > confident_Zvalues.size() - 1 || confident_Zvalues[index + 1].cost //something
        }
      }*/
    }
  }surface_p;

  vector< vector< surface_p > > surface;



  void _init_rectification_map(const FileStorage& fs);
  void _rectify_images(const Mat& left, const Mat& right);
  void _compute_dense_descriptors();
  void _reallocate_cache();
public:
  Stereo();
  Stereo(FileStorage& fs, JPP_Config& config);
  Stereo& operator=(Stereo& s);
  ~Stereo();
  void start_disparity_counter();
  int get_disparity_count();
  Point3f surface_point(int i, int j);
  void surface_index(const Point3f p, int *i, int *j);
  void load_images(const Mat& left, const Mat& right);
  bool in_img(int x, int y);
  Point project_point_cam(const Point3f p, int cam);
  void init_daisy_descriptors(double rad, int radq, int thq, int histq, int nrm_type);
  double desc_cost(Point left, Point right, int w);
  double desc_cost_SAD(Point left, Point right, int w);
  bool conf_positive(const Point3f p);
  bool conf_positive(const Point3f p, float z_start, float z_end);
  bool find_surface(const Point3f p, float range);
  bool find_surface(int ix, int iy, float range);
  Point3f median_filter(int ix, int iy, int neighbor_window_size);
  Point3f layer_median_filter(int ix, int iy, int neighbor_window_size);
  float change_in_slope(Point3f p);
  float change_in_slope(int ix, int iy);
  bool conf_negative(const Point3f p);
  void calc_z_range(const Point3f p, float *z_min, float *z_max);
  bool orientation_valid(Eigen::MatrixXf *points);
  bool is_obstacle_free_region(const Point3f p);
  bool is_empty_col(const Point3f p);
  bool is_bot_clear(const Point3f p, float safe_radius, float inc, bool col_check);
  float roughness(const Point3f p, float safe_radius, float inc, bool col_check);
  bool is_bot_clear_blind_ground(const Point3f p, float safe_radius, float inc, bool col_check);
  vector < pair< Point3f, float > > get_surface_points();
  vector < pair< Point3f, float > > get_surface_checks();
  int compute_disparity(Point p, int ndisp, int w);
  void compute_disparity_map(int ndisp, int w);
  void jpp_visualizations(Mat& confPos, Mat& confNeg);
  void blend_images(Mat& src1, Mat& src2, float alpha, Mat& dst);
  void update_jpp_config(JPP_Config& config);
  Mat get_img_left();
  Mat get_img_right();
  Mat get_Q_matrix();
  Mat get_disparity_map();
};

#endif // STEREO_H