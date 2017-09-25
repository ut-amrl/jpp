#include "jpp.h"

JPP::JPP()
{

}

JPP::JPP(Mat& img_left, Mat& img_right, FileStorage& fs, config_t* cf)
{
  _get_jpp_config(cf);
  _stereo = new Stereo(img_left, img_right, fs, _jpp_config);
}

Mat JPP::get_disparity_map(const char* method, int max_disp, const char* outfile)
{
  Mat dmap;
  _stereo->init_daisy_descriptors(_jpp_config.DR, _jpp_config.DQ, _jpp_config.DT, _jpp_config.DH, NRM_FULL);
  _stereo->compute_disparity_map(max_disp, _jpp_config.SAD_WINDOW_SIZE);
  dmap = _stereo->get_disparity_map();
  if (outfile != NULL) {
    imwrite(outfile, dmap);
  }
  return dmap;
}

JPP::~JPP()
{
  if (_stereo != NULL)
    delete _stereo;
}

Mat JPP::visualize_conf_pos()
{
  _stereo->init_daisy_descriptors(_jpp_config.DR, _jpp_config.DQ, _jpp_config.DT, _jpp_config.DH, NRM_FULL);
  Mat vis_img = _stereo->get_img_left();
  cvtColor(vis_img, vis_img, CV_GRAY2BGR);
  Point start(_jpp_config.START_X, -_jpp_config.MAX_Y);
  Point end(_jpp_config.MAX_X, _jpp_config.MAX_Y);
  int inc = _jpp_config.GRID_SIZE;
  float safe_radius = (float)max(_jpp_config.BOT_LENGTH/2, _jpp_config.BOT_WIDTH/2)/1000.;
  //#pragma omp parallel for collapse(2)\
  //num_threads(omp_get_max_threads())
  for (int x = start.x; x <= end.x; x += inc) {
    for (int y = start.y; y <= end.y; y += inc) {
      Point3f p((float)x/1000.,(float)y/1000.,0.0);
      Point ptl = _stereo->project_point_cam(p, 0);
      //if (_stereo->conf_positive(p)) {
      if (_stereo->is_bot_clear(p, safe_radius, (float)_jpp_config.GRID_SIZE/1000., false)) {
        Scalar green = Scalar(0, 255, 0);
        circle(vis_img, ptl, 3, green, -1, 8, 0);
      }
    }
  }
  return vis_img;
}

Mat JPP::visualize_conf_neg()
{
  _stereo->init_daisy_descriptors(_jpp_config.DR, _jpp_config.DQ, _jpp_config.DT, _jpp_config.DH, NRM_FULL);
  Mat vis_img = _stereo->get_img_left();
  cvtColor(vis_img, vis_img, CV_GRAY2BGR);
  Point start(_jpp_config.START_X, -_jpp_config.MAX_Y);
  Point end(_jpp_config.MAX_X, _jpp_config.MAX_Y);
  int inc = _jpp_config.GRID_SIZE;
  //#pragma omp parallel for collapse(3)\
  //num_threads(omp_get_max_threads())
  for (int x = start.x; x <= end.x; x += inc) {
    for (int y = start.y; y <= end.y; y += inc) {
      for (int z = _jpp_config.CONF_NEG_INC; z <= _jpp_config.BOT_HEIGHT; z += _jpp_config.CONF_NEG_INC) {
        Point3f q((float)x/1000.,(float)y/1000.,(float)z/1000.);
        Point qtl = _stereo->project_point_cam(q, 0);
        if (!_stereo->conf_negative(q)) {
          circle(vis_img, qtl, 1, Scalar(0,255,0), -1, 8, 0);
        } else {
          circle(vis_img, qtl, 1, Scalar(0,0,255), -1, 8, 0);
        }
      }
    }
  }
  return vis_img;
}

Mat JPP::visualize_empty_cols()
{
  _stereo->init_daisy_descriptors(_jpp_config.DR, _jpp_config.DQ, _jpp_config.DT, _jpp_config.DH, NRM_FULL);
  Mat vis_img = _stereo->get_img_left();
  cvtColor(vis_img, vis_img, CV_GRAY2BGR);
  Point start(_jpp_config.START_X, -_jpp_config.MAX_Y);
  Point end(_jpp_config.MAX_X, _jpp_config.MAX_Y);
  int inc = _jpp_config.GRID_SIZE;
  //#pragma omp parallel for collapse(3)\
  //num_threads(omp_get_max_threads())
  for (int x = start.x; x <= end.x; x += inc) {
    for (int y = start.y; y <= end.y; y += inc) {
      Point3f q((float)x/1000.,(float)y/1000.,0);
      Point qtl = _stereo->project_point_cam(q, 0);
      if (_stereo->is_empty_col(q)) {
        circle(vis_img, qtl, 1, Scalar(0,255,0), -1, 8, 0);
      } else {
        circle(vis_img, qtl, 1, Scalar(0,0,255), -1, 8, 0);
      }
    }
  }
  return vis_img;
}

pair< Mat, Mat > JPP::plan_astar(const char* outfile)
{
  _stereo->init_daisy_descriptors(_jpp_config.DR, _jpp_config.DQ, _jpp_config.DT, _jpp_config.DH, NRM_FULL);
  AStarPlanner planner(_jpp_config);
  Point start(_jpp_config.START_X, 0);
  Point end(_jpp_config.MAX_X, 0);
  bool convex_world = _jpp_config.CONVEX_WORLD;
  int safe_radius = max(_jpp_config.BOT_LENGTH/2, _jpp_config.BOT_WIDTH/2);
  int inc = _jpp_config.GRID_SIZE;
  planner.setParams(start, end, _jpp_config.MAX_Y, inc, safe_radius, _jpp_config.BOT_HEIGHT, convex_world);
  planner.findPath(_stereo);
  vector< Point > path = planner.getPath();
  
  // visualizations
  Mat vis_path = _stereo->get_img_left();
  cvtColor(vis_path, vis_path, CV_GRAY2BGR);
  for (int i = 0; i < path.size()-1; i++) {
    Point p = _stereo->project_point_cam(Point3f(path[i].x/1000.,path[i].y/1000.,0),0);
    Point q = _stereo->project_point_cam(Point3f(path[i+1].x/1000.,path[i+1].y/1000.,0),0);
    Scalar green = Scalar(0, 255, 0);
    line(vis_path, p, q, green, 2, 8 ,0);
  }
  
  Mat confPos = _stereo->get_img_left();
  cvtColor(confPos, confPos, CV_GRAY2BGR);
  Mat confNeg = Mat(confPos.rows, confPos.cols, CV_8UC3, Scalar(0,0,0));
  _stereo->jpp_visualizations(confPos, confNeg);
  _stereo->blend_images(confPos, confNeg, 0.6, confPos);
  
  if (outfile != NULL) {
    char vis_path_file[100], confPos_file[100];
    sprintf(vis_path_file, "%s%s.jpg", outfile, "-path");
    sprintf(confPos_file, "%s%s.jpg", outfile, "-vis");
    imwrite(vis_path_file, vis_path);
    imwrite(confPos_file, confPos);
  }
  return make_pair< Mat, Mat >(vis_path, confPos);
}

pair< Mat, Mat > JPP::plan_rrt(const char* outfile)
{
  _stereo->init_daisy_descriptors(_jpp_config.DR, _jpp_config.DQ, _jpp_config.DT, _jpp_config.DH, NRM_FULL);
  Point start(_jpp_config.START_X, 0);
  Point end(_jpp_config.MAX_X, 0);
  bool convex_world = _jpp_config.CONVEX_WORLD;
  int safe_radius = max(_jpp_config.BOT_LENGTH/2, _jpp_config.BOT_WIDTH/2);
  int inc = _jpp_config.GRID_SIZE;
  RRT planner;
  planner.initRRT(start, end, inc, 500, 0.6, safe_radius, _jpp_config.BOT_HEIGHT, convex_world, _jpp_config);
  planner.findPath(_stereo);
  vector< Point > path = planner.getPath();
  
  // visualizations
  Mat vis_path = _stereo->get_img_left();
  cvtColor(vis_path, vis_path, CV_GRAY2BGR);
  for (int i = 0; i < path.size()-1; i++) {
    Point p = _stereo->project_point_cam(Point3f(path[i].x/1000.,path[i].y/1000.,0),0);
    Point q = _stereo->project_point_cam(Point3f(path[i+1].x/1000.,path[i+1].y/1000.,0),0);
    Scalar green = Scalar(0, 255, 0);
    line(vis_path, p, q, green, 2, 8 ,0);
  }
  
  Mat confPos = _stereo->get_img_left();
  cvtColor(confPos, confPos, CV_GRAY2BGR);
  Mat confNeg = Mat(confPos.rows, confPos.cols, CV_8UC3, Scalar(0,0,0));
  _stereo->jpp_visualizations(confPos, confNeg);
  _stereo->blend_images(confPos, confNeg, 0.6, confPos);
  
  if (outfile != NULL) {
    char vis_path_file[100], confPos_file[100];
    sprintf(vis_path_file, "%s%s.jpg", outfile, "-path");
    sprintf(confPos_file, "%s%s.jpg", outfile, "-vis");
    imwrite(vis_path_file, vis_path);
    imwrite(confPos_file, confPos);
  }
  return make_pair< Mat, Mat >(vis_path, confPos);
}

void JPP::update_jpp_config(JPP_Config& config)
{
  _jpp_config = config;
  _stereo->update_jpp_config(config);
}

vector< double > JPP::get_epipolar_costs(Point p, int max_disp)
{
  vector< double > costs;
  for (int d = 0; d <= max_disp; d++) {
    double cost = _stereo->desc_cost_SAD(p, Point(p.x-d,p.y), _jpp_config.SAD_WINDOW_SIZE);
    costs.push_back(cost);
  }
  return costs;
}

void JPP::reset()
{
  _stereo->init_daisy_descriptors(_jpp_config.DR, _jpp_config.DQ, _jpp_config.DT, _jpp_config.DH, NRM_FULL);
}

Mat JPP::get_img_left()
{
  return _stereo->get_img_left();
}

Mat JPP::get_img_right()
{
  return _stereo->get_img_right();
}

Mat JPP::get_Q_matrix()
{
  return _stereo->get_Q_matrix();
}

///////// PRIVATE FUNCTIONS /////////

void JPP::_get_jpp_config(config_t* cf)
{
  _jpp_config = JPP_Config(cf);
}