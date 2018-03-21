#include "stereo.h"

Stereo::Stereo()
{
  
}

Stereo::Stereo(FileStorage& fs, JPP_Config& config)
{
  _jpp_config = config;
  _init_rectification_map(fs);
  _desc_left = new daisy;
  _desc_right = new daisy;
  int res = _jpp_config.RECT_IMG_WIDTH * _jpp_config.RECT_IMG_HEIGHT;
  _obstacleCache = vector< int >(res, 0);
  _obstacleRangeCache = vector< int >(res, 0);
  _colCache = vector< int >(res, 0);
  _confNegCache = vector< int >(res, 0);
  _descLeftSet = vector< int >(res, 0);
  _descRightSet = vector< int >(res, 0);
  _descLeftCache = vector< float* >(res, 0);
  _descRightCache = vector< float* >(res, 0);
  _reallocate_cache();

  num_disparity_checks = 0;

  surface = vector< vector< surface_p > >((_jpp_config.MAX_X - _jpp_config.START_X)/_jpp_config.GRID_SIZE + 1,
    vector< surface_p >((_jpp_config.MAX_Y*2)/_jpp_config.GRID_SIZE + 1, surface_p(false, false, false, false, 0, 0, 0)) );

  printf("surface-x: %lu, surface-y: %lu\n", surface.size(), surface[0].size());
}

Stereo& Stereo::operator=(Stereo& s)
{
  if (this == &s) return *this;
  this->_jpp_config = s._jpp_config;
  this->_img_left = s._img_left.clone();
  this->_img_right = s._img_right.clone();
  this->_XR = s._XR.clone();
  this->_XRINV = s._XRINV.clone();
  this->_XT = s._XT.clone();
  this->_Q = s._Q.clone();
  this->_P1 = s._P1.clone();
  this->_P2 = s._P2.clone();
  this->_R1 = s._R1.clone();
  this->_R2 = s._R2.clone();
  this->_D1 = s._D1.clone();
  this->_D2 = s._D2.clone();
  this->_R = s._R.clone();
  this->_lmapx = s._lmapx.clone();
  this->_lmapy = s._lmapy.clone();
  this->_rmapx = s._rmapx.clone();
  this->_rmapy = s._rmapy.clone();
  this->_T = s._T;
  this->_obstacleCache = s._obstacleCache;
  this->_obstacleRangeCache = s._obstacleRangeCache;
  this->_colCache = s._colCache;
  this->_confNegCache = s._confNegCache;
  //this->_descLeftCache = s._descLeftCache;
  //this->_descRightCache = s._descRightCache;
  this->_descLeftSet = s._descRightSet;
  this->_cacheVis = s._cacheVis;
  this->_desc_left = new daisy(*s._desc_left);
  this->_desc_right = new daisy(*s._desc_right);
  this->surface = s.surface;
  return *this;
}

Stereo::~Stereo()
{
  if (_desc_left != NULL)
    delete _desc_left;
  if (_desc_right != NULL)
    delete _desc_right;
  for (int i = 0; i < _img_left.cols * _img_left.rows; i++) {
    delete[] _descLeftCache[i];
    delete[] _descRightCache[i];
  }
}

void Stereo::start_disparity_counter()
{
  num_disparity_checks = 0;
}
int Stereo::get_disparity_count()
{
  return num_disparity_checks;
}

//finds the point in the surface for the given index
Point3f Stereo::surface_point(int i, int j)
{
  Point3f p;
  p.x = (float)(_jpp_config.GRID_SIZE*i + _jpp_config.START_X)/1000.0;
  p.y = (float)(_jpp_config.GRID_SIZE*j - _jpp_config.MAX_Y)/1000.0;
  p.z = surface[i][j].median_filtered_z;//surface[i][j].z;
  return p;
}

//finds the index in the surface for the given point
void Stereo::surface_index(const Point3f p, int *i, int *j)
{
  *i = ((int)round(p.x*1000.0) - _jpp_config.START_X)/_jpp_config.GRID_SIZE;
  *j = ((int)round(p.y*1000.0) + _jpp_config.MAX_Y)/_jpp_config.GRID_SIZE;
}

bool Stereo::inSurface(int ix, int iy)
{
  if (ix >= 0 && ix < (int)surface.size() && iy >= 0 && iy < (int)surface[ix].size())
  {
    return true;
  }
  return false;
}

void Stereo::load_images(const Mat& left, const Mat& right)
{
  _rectify_images(left, right);
 
  fill(_obstacleCache.begin(), _obstacleCache.end(), 0);
  fill(_obstacleRangeCache.begin(), _obstacleRangeCache.end(), 0);
  fill(_colCache.begin(), _colCache.end(), 0);
  fill(_confNegCache.begin(), _confNegCache.end(), 0);
  fill(_descLeftSet.begin(), _descLeftSet.end(), 0);
  fill(_descRightSet.begin(), _descRightSet.end(), 0);
  //_disparityMap = Mat(_img_left.rows, _img_left.cols, CV_8UC1, Scalar(0));
  //_dMapVis = Mat(img_left.rows, img_left.cols, CV_8UC3, Scalar(0,0,0));
  //_cacheVis = Mat(_img_left.rows, _img_left.cols, CV_8UC3, Scalar(0,0,0));
  cvtColor(_img_left, _cacheVis, CV_GRAY2BGR);
  for(uint i = 0; i < surface.size(); i++)
  {
    fill(surface[i].begin(), surface[i].end(), surface_p(false, false, false, false, 0, 0, 0));
  }
}

bool Stereo::in_img(int x, int y)
{
  if (x >= 0 && x < _img_left.cols && y >= 0 && y < _img_left.rows)
    return true;
  return false;
}

Point Stereo::project_point_cam(const Point3f p, int cam)
{
  Point imgc;
  Eigen::Vector3f pt3d(p.x, p.y, p.z);
  Eigen::Vector3f pt3d_cam = _cam2robot_R * (pt3d - _cam2robot_T);
  Eigen::Vector4f pt3d_cam_hom(pt3d_cam(0), pt3d_cam(1), pt3d_cam(2), 1.0);
  Eigen::Vector3f img_coord;
  if (cam == 0) {
    img_coord = _eP1 * pt3d_cam_hom;
  } else {
    img_coord = _eP2 * pt3d_cam_hom;
  }
  imgc.x = img_coord(0)/img_coord(2);
  imgc.y = img_coord(1)/img_coord(2);
  return imgc;
}

void Stereo::init_daisy_descriptors(double rad, int radq, int thq, int histq, int nrm_type)
{
  int verbose_level = 0;
  //bool disable_interpolation = true;
  // associate pointer
  uchar *imL = _img_left.data;
  uchar *imR = _img_right.data;
  int h = _img_left.rows;
  int w = _img_left.cols;

  _desc_left->release_auxilary();
  _desc_left->reset();
  _desc_left->set_image(imL, h, w);
  _desc_left->verbose(verbose_level);
  _desc_left->set_parameters(rad, radq, thq, histq);
  _desc_left->set_normalization(nrm_type);
  _desc_left->initialize_single_descriptor_mode();
  
  _desc_right->release_auxilary();
  _desc_right->reset();
  _desc_right->set_image(imR, h, w);
  _desc_right->verbose(verbose_level);
  _desc_right->set_parameters(rad, radq, thq, histq);
  _desc_right->set_normalization(nrm_type);
  _desc_right->initialize_single_descriptor_mode();
  
  //_descLeftCache = new float*[_img_left.cols * _img_left.rows];
  //_descRightCache = new float*[_img_right.cols * _img_right.rows];
}

double Stereo::desc_cost(Point left, Point right, int w)
{
  num_disparity_checks++;
  int width = _img_left.cols;
  double cost = 0;
  for (int j = -w; j <= w; j++) {
    for (int k = -w; k <= w; k++) {
      if (!in_img(left.x+j, left.y+k) || !in_img(right.x+j, right.y+k))
        continue;
      int idx_l = (left.y+k)*width + left.x+j;
      int idx_r = (right.y+k)*width + right.x+j;
      if (_descLeftSet[idx_l] != 1) {
        _desc_left->get_descriptor(left.y+k, left.x+j, 0, _descLeftCache[idx_l]);
        _descLeftSet[idx_l] = 1;
      }
      if (_descRightSet[idx_r] != 1) {
        _desc_right->get_descriptor(right.y+k, right.x+j, 0, _descRightCache[idx_r]);
        _descRightSet[idx_r] = 1;
      }
      for (int zz = 0; zz < _desc_left->descriptor_size(); zz++) {
        cost += fabs(_descLeftCache[idx_l][zz] - _descRightCache[idx_r][zz]);
      }
    }
  }
  return cost;
}

double Stereo::desc_cost_SAD(Point left, Point right, int w)
{
  num_disparity_checks++;
  double cost = 0;
  for (int j = -w; j <= w; j++) {
    for (int k = -w; k <= w; k++) {
      if (!in_img(left.x+j, left.y+k) || !in_img(right.x+j, right.y+k))
        continue;
      cost += abs(_img_left.at<uchar>(left.y+k,left.x+j) - _img_right.at<uchar>(right.y+k, right.x+j));
    }
  }
  return cost;
}

bool Stereo::conf_positive(const Point3f p)
{
  Point ptl = project_point_cam(p, 0);
  Point ptr = project_point_cam(p, 1);
  if (!in_img(ptl.x,ptl.y) || !in_img(ptr.x,ptr.y)) {
    return false;
  }
  int idx = ptl.y * _img_left.cols + ptl.x;
  if (_obstacleCache[idx] == 1) // obstacle free
    return true;
  if (_obstacleCache[idx] == 2) // obstacle
    return false;
  
  int w = _jpp_config.SAD_WINDOW_SIZE;
  double cost = desc_cost(ptl, ptr, w);
  cost /= (double)((2*w+1)*(2*w+1));
  if (cost < _jpp_config.CONF_POS_THRESH) {
    _obstacleCache[idx] = 1;
    return true;
  } else {
    _obstacleCache[idx] = 2;
  }
  return false;
}

bool Stereo::conf_negative(const Point3f p)
{
  Point ptl = project_point_cam(p, 0);
  Point ptr = project_point_cam(p, 1);
  if (!in_img(ptl.x,ptl.y) || !in_img(ptr.x,ptr.y)) {
    return false;
  }
  int idx = ptl.y * _img_left.cols + ptl.x;
  if (_confNegCache[idx] == 1) // obstacle free
    return true;
  if (_confNegCache[idx] == 2) // obstacle
    return false;
  int w = _jpp_config.SAD_WINDOW_SIZE;
  double cost = desc_cost(ptl, ptr, w);
  cost /= (double)((2*w+1)*(2*w+1));
  if (cost > _jpp_config.CONF_NEG_THRESH) {
    _confNegCache[idx] = 1;
    return true;
  }
  _confNegCache[idx] = 2;
  return false;
}

bool Stereo::find_surface(const Point3f p, float range)
{
  int ix, iy;
  surface_index(p, &ix, &iy);
  return find_surface(ix, iy, range);
}

bool Stereo::find_surface(int ix, int iy, float range)
{
  if (surface[ix][iy].discovered)
  {
    return surface[ix][iy].valid;
  }

  surface[ix][iy].discovered = true;

  float optimum_z = 0;
  float center = 0;
  float sum = 0;
  float num = 1; //this adds a zero to the average
  for (int nx = ix - 1; nx <= ix + 1; nx++)
  {
    //check valid x index
    if (nx >= 0 && nx < (int)surface.size())
    {
      for (int ny = iy - 1; ny <= iy + 1; ny++)
      {
        //check valid y index
        if (ny >= 0 && ny < (int)surface[nx].size())
        {
          if (surface[nx][ny].discovered)
          {
            sum += surface[nx][ny].z;
            num++;
          }
        }
      }
    }
  }
  optimum_z = sum/num;
  center = optimum_z;

  float z_min = center - (range/2.0);
  float z_max = center + (range/2.0);

  Point3f lineS = surface_point(ix, iy);
  lineS.z = z_min;
  Point3f lineE = surface_point(ix, iy);
  lineE.z = z_max;

  search_space.push_back(make_pair(lineS, lineE));

  //visualize_find_surface(lineS, lineE);

  //check that zmin and zmax don't overlap
  if (z_min >= z_max)
  {
    return false;
  }

  float inc = (float)_jpp_config.Z_INC/1000.0;


  Point3f search_point = surface_point(ix, iy);

  int w = _jpp_config.SAD_WINDOW_SIZE;
  double min_cost = 9999.9;
  float best_z = 0;
  double cost = 9999.9;
  Point ptl = project_point_cam(search_point, 0);
  Point ptr = project_point_cam(search_point, 1);
  float z_error_weight = _jpp_config.ZERR_WEIGHT;

  vector< pair< Point3f, float > > confpos;
  //vector< pair< float, float > > minima;

  int num_in_image = 0;
  for (float z = z_min; z < z_max; z += inc)
  {
    search_point.z = z;
    ptl = project_point_cam(search_point, 0);
    ptr = project_point_cam(search_point, 1);
    if (in_img(ptl.x,ptl.y) && in_img(ptr.x,ptr.y)) {
      num_in_image++;
      float z_error = fabs(z - optimum_z);
      cost = (desc_cost(ptl, ptr, w)/((double)((2*w+1)*(2*w+1)))) + z_error*z_error_weight;

      confpos.push_back(pair< Point3f, float >(search_point, cost));

      confident_z cz;
      cz.cost = cost;
      cz.z = z;
      surface[ix][iy].insert_confident_z(cz);

      if (cost < min_cost) {
        min_cost = cost;
        best_z = z;
      }
    }
  }

  //visualize_find_surface(ix, iy);
  
  if (num_in_image == 0 || min_cost > _jpp_config.CONF_POS_THRESH)
  {
    surface[ix][iy].z = 10.0;
    surface[ix][iy].median_filtered_z = 10.0;
    surface[ix][iy].confpos = confpos;
    return false;
  }

  surface[ix][iy].z = best_z;
  surface[ix][iy].median_filtered_z = best_z;
  surface[ix][iy].confpos = confpos;
  return true;
}

Point3f Stereo::median_filter(int ix, int iy, int neighbor_window_size)
{
  //check if already filtered
  if (surface[ix][iy].median_filtered)
  {
    return surface_point(ix, iy);
  }

  if (neighbor_window_size == 0)
  {
    surface[ix][iy].valid = true;
    surface[ix][iy].median_filtered = true;
    find_surface(ix, iy, (float)_jpp_config.SEARCH_RANGE/1000.0);
    surface[ix][iy].median_filtered_z = surface[ix][iy].z;
  }

  surface[ix][iy].valid = true;
  surface[ix][iy].median_filtered = true;

  vector< float > neighbors;
  neighbors.push_back(-9999.9);
  neighbors.push_back(9999.9);
  for (int nx = ix - neighbor_window_size; nx <= ix + neighbor_window_size; nx++)
  {
    //check valid x index
    if (nx >= 0 && nx < (int)surface.size())
    {
      for (int ny = iy - neighbor_window_size; ny <= iy + neighbor_window_size; ny++)
      {
        //check valid y index
        if (ny >= 0 && ny < (int)surface[nx].size())
        {
          if (!surface[nx][ny].discovered)
          {
            find_surface(nx, ny, (float)_jpp_config.SEARCH_RANGE/1000.0);//range will be determined by something else
          }
          if (true)//surface[nx][ny].valid)
          {
            //add neigbor

            //niave implementation
            for(vector< float >::iterator it = neighbors.begin(); it != neighbors.end(); ++it)
            {
              if (surface[nx][ny].z <= *it)
              {
                neighbors.insert(it, surface[nx][ny].z);
                break;
              }
            }
          }
        }
      }
    }
  }

  int middle = (neighbors.size())/2;

  surface[ix][iy].median_filtered_z = neighbors[middle];
  return surface_point(ix, iy);
}

float Stereo::roughness(Point3f p)
{
  int ix, iy;
  surface_index(p, &ix, &iy);
  return roughness(ix, iy);
}

float Stereo::roughness(int ix, int iy)
{
  //check if already calculated
  if (surface[ix][iy].roughness_calculated)
  {
    return surface[ix][iy].roughness;
  }

  int x_min, unimportant_y;
  Point3f x_min_p;
  x_min_p.x = (float)_jpp_config.START_X/1000.0;
  x_min_p.y = 0.0;
  x_min_p.z = 0.0;
  surface_index(x_min_p, &x_min, &unimportant_y);

  //find and filter the area around the point
  for (int nx = ix - 1; nx <= ix + 1; nx++)
  {
    //check valid x index
    if (nx >= x_min && nx < (int)surface.size())
    {
      for (int ny = iy - 1; ny <= iy + 1; ny++)
      {
        //check valid y index
        if (ny >= 0 && ny < (int)surface[nx].size())
        {
          median_filter(nx, ny, _jpp_config.MEDIAN_FILTER_RANGE);
        }
      }
    }
  }

  //do better initualization:
  vector< pair< float, float > > neighbor_z_pairs;
  if (inSurface(ix, iy - 1) && surface[ix][iy - 1].valid 
    && inSurface(ix, iy + 1) && surface[ix][iy + 1].valid)
    neighbor_z_pairs.push_back(pair< float, float >(surface[ix][iy - 1].median_filtered_z, surface[ix][iy + 1].median_filtered_z));//bottom to top
  if (inSurface(ix - 1, iy) && surface[ix - 1][iy].valid 
    && inSurface(ix + 1, iy) && surface[ix + 1][iy].valid)
    neighbor_z_pairs.push_back(pair< float, float >(surface[ix - 1][iy].median_filtered_z, surface[ix + 1][iy].median_filtered_z));//left to right
  if (inSurface(ix - 1, iy - 1) && surface[ix - 1][iy - 1].valid 
    && inSurface(ix + 1, iy + 1) && surface[ix + 1][iy + 1].valid)
    neighbor_z_pairs.push_back(pair< float, float >(surface[ix - 1][iy - 1].median_filtered_z, surface[ix + 1][iy + 1].median_filtered_z));//bottom left to top right
  if (inSurface(ix - 1, iy + 1) && surface[ix - 1][iy + 1].valid 
    && inSurface(ix + 1, iy - 1) && surface[ix + 1][iy - 1].valid)
  neighbor_z_pairs.push_back(pair< float, float >(surface[ix - 1][iy + 1].median_filtered_z, surface[ix + 1][iy - 1].median_filtered_z));//bottom right to top left

  float target_z = surface[ix][iy].median_filtered_z;

  float greatest_slope_change = 0;

  for(uint i = 0; i < neighbor_z_pairs.size(); i++)
  {
    float slope1 = fabs((target_z - neighbor_z_pairs[i].first)/(_jpp_config.GRID_SIZE/1000.0));
    float slope2 = fabs((target_z - neighbor_z_pairs[i].second)/(_jpp_config.GRID_SIZE/1000.0));
    float slope_change = fabs(slope2 - slope1);

    if (slope_change > greatest_slope_change)
    {
      greatest_slope_change = slope_change;
    }
  }

  surface[ix][iy].roughness = greatest_slope_change;
  surface[ix][iy].roughness_calculated = true;

  //visualize
  Point ptl = project_point_cam(surface_point(ix, iy), 0);
  //Point ptr = project_point_cam(surface_point(ix, iy), 1);
  if (in_img(ptl.x, ptl.y))
  {
    int idx = ptl.y * _img_left.cols + ptl.x;
    _obstacleCache[idx] = (int)(1.0 + greatest_slope_change*200.0);
    if (_obstacleCache[idx] > 255)
    {
      _obstacleCache[idx] = 255;
    }
  }

  return greatest_slope_change;
}

bool Stereo::is_obstacle_free_region(const Point3f p)
{
  Point ptl = project_point_cam(p, 0);
  int idx = ptl.y * _img_left.cols + ptl.x;
  if (_obstacleRangeCache[idx] == 1) // obstacle free range
    return true;
  if (_obstacleRangeCache[idx] == 2) // obstacle range
    return false;
  int count = 0;
  int total_points = 0;
  float w = (float)_jpp_config.SPATIAL_FILTER_WINDOW/1000.;
  float inc = (float)_jpp_config.SPATIAL_FILTER_INC/1000.;
  for (float x = 0; x <= w; x += inc) {
    for (float y = -w; y <= w; y += inc) {
      Point3f q(p.x+x, p.y+y, 0);
      if (conf_positive(q))
        count++;
      total_points++;
    }
  }
  float ratio = _jpp_config.SPATIAL_FILTER_RATIO;
  if (count > (float)total_points * ratio ) {
    _obstacleRangeCache[idx] = 1;
    return true;
  } else {
    _obstacleRangeCache[idx] = 2;
  }
  return false;
}

bool Stereo::is_empty_col(const Point3f p)
{
  Point ptl = project_point_cam(p, 0);
  int idx = ptl.y * _img_left.cols + ptl.x;
  if (_colCache[idx] == 1) // obstacle free col
    return true;
  if (_colCache[idx] == 2) // obstacle col
    return false;
  float inc = (float)_jpp_config.CONF_NEG_INC/1000.;
  int total = 0;
  int match = 0;
  float h = (float)_jpp_config.BOT_HEIGHT/1000.;
  for (float z = inc; z <= h; z += inc) {
    Point3f q(p.x,p.y,z);
    Point ptl = project_point_cam(q, 0);
    Point ptr = project_point_cam(q, 1);
    if (!in_img(ptl.x,ptl.y) || !in_img(ptr.x,ptr.y)) {
      continue;  
    }
    total++;
    if (!conf_negative(q)) {
      match++;
    }
  }
  if (total < 3) {
    _colCache[idx] = 2;
    return false;
  }
  /*if (total == 0)
  {
    _colCache[idx] = 1;
    return true;
  }*/
  if ((float)match > (float)total * _jpp_config.CONF_NEG_FILTER_RATIO) {
    _colCache[idx] = 1;
    return true;
  }
  _colCache[idx] = 2;
  return false;
  return true;
}

bool Stereo::is_bot_clear(const Point3f p, float safe_radius, float inc, bool col_check)
{
  bool isFree = true;
  for (float y = -safe_radius; y <= safe_radius; y += inc) {
    for (float x = 0; x <= safe_radius; x += inc) {
      Point3f q(p.x+x,p.y+y,0.0);

      // if (change_in_slope(q) > 1.3)
      // {
      //   isFree = false;
      //   return false;
      // }

      find_surface(q, 0.4);
      int ix, iy;
      surface_index(q, &ix, &iy);
      //average_filter(ix, iy, 1);
      //median_filter(ix, iy, 2);
      //just going to visualize;
    }
  }
  return isFree;
}

float Stereo::traversability(const Point3f p, float safe_radius, float inc, bool col_check)
{
  float sum = 0;
  float num = 0;

  for (float y = -safe_radius; y <= safe_radius; y += inc) {
    for (float x = 0; x <= safe_radius; x += inc) {
      Point3f q(p.x+x,p.y+y,0.0);

      if (q.x < 0 || q.x > _jpp_config.MAX_X/1000.0 || 
        q.y < -_jpp_config.MAX_Y/1000.0 || q.y > _jpp_config.MAX_Y/1000.0)
      {
        continue;
      }
      float r = roughness(q);

      if (r > _jpp_config.ROUGHNESS_THRESH)
      {
        return -1.0;
      }
      sum += r;
      num++;
    }
  }

  if (num != 0)
  {
    return sum/num;
  }
  printf("not enough points for stereo::traversability\n");
  return -1.0;
}

bool Stereo::is_bot_clear_blind_ground(const Point3f p, float safe_radius, float inc, bool col_check)
{
  bool isFree = true;
  if (col_check) {
    for (float y = -safe_radius; y <= safe_radius; y += inc) {
      for (float x = 0; x <= safe_radius; x += inc) {
        Point3f q(p.x+x,p.y+y,0.0);
          if (!is_empty_col(q)) {
            isFree = false;
            break;
          }
      }
    }
  }
  return isFree;
}

vector < pair< Point3f, float > > Stereo::get_surface_points(){
  vector < pair< Point3f, float > > surface_points;
  for (uint i = 0; i < surface.size(); i++){
    for (uint j = 0; j < surface[i].size(); j++)
    {
      if (surface[i][j].discovered && surface[i][j].roughness_calculated)
      {
        pair< Point3f, float > sp;
        sp.first = surface_point(i, j);
        sp.second = surface[i][j].roughness;
        surface_points.push_back(sp);
      }
    }
  }
  return surface_points;
}

vector < pair< Point3f, float > > Stereo::get_surface_checks(){
  vector < pair< Point3f, float > > confpos_points;
  for (uint i = 0; i < surface.size(); i++){
    for (uint j = 0; j < surface[i].size(); j++)
    {
      if (surface[i][j].discovered && surface[i][j].valid)
      {
        for (uint k = 0; k < surface[i][j].confpos.size(); k++)
        {
          confpos_points.push_back(surface[i][j].confpos[k]);
        }
      }
    }
  }
  return confpos_points;
}

int Stereo::compute_disparity(Point p, int ndisp, int w)
{
  double min_cost = 1e9;
  int disparity = 0;
  for (int i = p.x-ndisp; i <= p.x; i++) {
    double cost = desc_cost_SAD(p, Point(i,p.y), w);
    if (cost < min_cost) {
      min_cost = cost;
      disparity = p.x - i;
    }
  }
  return disparity;
}

void Stereo::compute_disparity_map(int ndisp, int w)
{
  /*
  for (int i = ndisp; i < _img_left.cols; i++) {
    for (int j = 0; j < _img_left.rows; j++) {
      int d = compute_disparity(Point(i, j), ndisp, w);
      //_disparityMap.at<uchar>(j, i) = d;
    }
  }
  */
}

void Stereo::jpp_visualizations(Mat& confPos, Mat& confNeg)
{
  for (int i = 0; i < confPos.cols; i++) {
    for (int j = 0; j < confPos.rows; j++) {
      int idx = j * _img_left.cols + i;
      if (_obstacleCache[idx] != 0) {
        circle(confPos,Point(i,j),2,Scalar(0, 255 - _obstacleCache[idx], _obstacleCache[idx]),-1,8,0);
      }
    }
  }

  //old:

  // for (int i = 0; i < confPos.cols; i++) {
  //   for (int j = 0; j < confPos.rows; j++) {
  //     int idx = j * _img_left.cols + i;
  //     if (_obstacleCache[idx] == 1) { // obstacle free
  //       if (!_jpp_config.CONVEX_WORLD && _colCache[idx] == 2) {
  //         circle(confPos,Point(i,j),2,Scalar(0,0,255),-1,8,0);
  //       } else {
  //         circle(confPos,Point(i,j),2,Scalar(0,255,0),-1,8,0);
  //       }
  //       //cacheVis.at<Vec3b>(j,i) = Vec3b(0,255,0);
  //     }
  //     else if (_obstacleCache[idx] == 2) { // obstacle
  //       circle(confPos,Point(i,j),3,Scalar(0,200,200),-1,8,0);
  //       //cacheVis.at<Vec3b>(j,i) = Vec3b(0,0,255);
  //     } 
  //     else if (_obstacleCache[idx] == 3) { // obstacle
  //       circle(confPos,Point(i,j),3,Scalar(0,0,255),-1,8,0);
  //       //cacheVis.at<Vec3b>(j,i) = Vec3b(0,0,255);
  //     }
  //     else {
  //       //int col = (int)img_left.at<uchar>(j,i);
  //       //cacheVis.at<Vec3b>(j,i) = Vec3b(col,col,col);
  //     }
  //     if (_confNegCache[idx] == 2) { // obstacle free
  //       circle(confNeg,Point(i,j),1,Scalar(0,200,200),-1,8,0);
  //     }
  //     else if (_confNegCache[idx] == 1) { // obstacle
  //       circle(confNeg,Point(i,j),1,Scalar(255,0,255),-1,8,0);
  //       //cacheVis.at<Vec3b>(j,i) = Vec3b(0,0,255);
  //     } else {
  //     }
  //   }
  // }
}

pair< Mat, Mat> Stereo::visualize_find_surface(Point3f p, Point3f q)
{
  Scalar green = Scalar(0, 255, 0);

  Point lp = project_point_cam(p, 0);
  Point lq = project_point_cam(q, 0);

  Mat left_search_space = _img_left;
  cvtColor(left_search_space, left_search_space, CV_GRAY2BGR);
  line(left_search_space, lp, lq, green, 2, 8, 0);

  Point rp = project_point_cam(p, 1);
  Point rq = project_point_cam(q, 1);

  Mat right_search_space = _img_right;
  cvtColor(right_search_space, right_search_space, CV_GRAY2BGR);
  line(right_search_space, rp, rq, green, 2, 8, 0);

  imshow("left_search_space1", left_search_space);
  imshow("right_search_space1", right_search_space);

  waitKey(30);

  return make_pair(left_search_space, right_search_space);
}

pair< Mat, Mat> Stereo::visualize_find_surface(int ix, int iy)
{
  Point3f search_point = surface_point(ix, iy);

  Mat left_search_space = _img_left;
  Mat right_search_space = _img_right;
  cvtColor(left_search_space, left_search_space, CV_GRAY2BGR);
  cvtColor(right_search_space, right_search_space, CV_GRAY2BGR);
  for(uint i = 0; i < surface[ix][iy].confident_Zvalues.size(); i++)
  {
    search_point.z = surface[ix][iy].confident_Zvalues[i].z;
    Point lp = project_point_cam(search_point, 0);
    Point rp = project_point_cam(search_point, 1);
    int cost = (int)(surface[ix][iy].confident_Zvalues[i].cost * 200.0);
    if (cost > 255)
      cost = 255;
    Scalar color = Scalar(cost, 0, 255 - cost);
    circle(left_search_space, lp, 2, color, -1, 8, 0);
    circle(right_search_space, rp, 2, color, -1, 8, 0);
  }

  imshow("left_search_space2", left_search_space);
  imshow("right_search_space2", right_search_space);

  waitKey(30);

  return make_pair(left_search_space, right_search_space);
}

void Stereo::blend_images(Mat& src1, Mat& src2, float alpha, Mat& dst)
{
  float beta = (1.0 - alpha);
  addWeighted( src1, alpha, src2, beta, 0.0, dst);
}

void Stereo::update_jpp_config(JPP_Config& config)
{
  _jpp_config = config;
  _reallocate_cache();
}

Mat Stereo::get_img_left()
{
  return _img_left;
}

Mat Stereo::get_img_right()
{
  return _img_right;
}

Mat Stereo::get_Q_matrix()
{
  return _Q;
}

Mat Stereo::get_disparity_map()
{
  return _disparityMap;
}

///////// PRIVATE FUNCTIONS /////////

void Stereo::_init_rectification_map(const FileStorage& fs)
{
  fs["K1"] >> _K1;
  fs["K2"] >> _K2;
  fs["D1"] >> _D1;
  fs["D2"] >> _D2;
  fs["R"] >> _R;
  fs["T"] >> _T;
  fs["XR"] >> _XR;
  fs["XT"] >> _XT;
  Rect validRoi[2];
  Size calib_img_size = Size(_jpp_config.CALIB_IMG_WIDTH, _jpp_config.CALIB_IMG_HEIGHT);
  Size rect_img_size = Size(_jpp_config.RECT_IMG_WIDTH, _jpp_config.RECT_IMG_HEIGHT);
  
  stereoRectify(_K1, _D1, _K2, _D2, calib_img_size, _R, Mat(_T), _R1, _R2, _P1, _P2, _Q, 
                CV_CALIB_ZERO_DISPARITY, 0, rect_img_size, &validRoi[0], &validRoi[1]);
  initUndistortRectifyMap(_K1, _D1, _R1, _P1, rect_img_size, CV_32F, _lmapx, _lmapy);
  initUndistortRectifyMap(_K2, _D2, _R2, _P2, rect_img_size, CV_32F, _rmapx, _rmapy);
  
  _XRINV = _XR.inv();
  _cam2robot_T = Eigen::Vector3f(_XT.at<double>(0,0), _XT.at<double>(1,0), _XT.at<double>(2,0));
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      _cam2robot_R(i, j) = _XRINV.at<double>(i, j);
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      _eP1(i, j) = _P1.at<double>(i, j);
      _eP2(i, j) = _P2.at<double>(i, j);
    }
  }
}

void Stereo::_rectify_images(const Mat& left, const Mat& right)
{
  remap(left, _img_left, _lmapx, _lmapy, cv::INTER_LINEAR);
  remap(right, _img_right, _rmapx, _rmapy, cv::INTER_LINEAR);
  cvtColor(_img_left, _img_left, CV_BGR2GRAY);
  cvtColor(_img_right, _img_right, CV_BGR2GRAY);
}

void Stereo::_reallocate_cache()
{
  int desc_size = (_jpp_config.DQ * _jpp_config.DT + 1) * _jpp_config.DH;
  for (int i = 0; i < _jpp_config.RECT_IMG_HEIGHT * _jpp_config.RECT_IMG_WIDTH; i++) {
    if (_descLeftCache[i])
      delete[] _descLeftCache[i];
    _descLeftCache[i] = new float[desc_size];
    if (_descRightCache[i])
      delete[] _descRightCache[i];
    _descRightCache[i] = new float[desc_size];
  }
}

void Stereo::_compute_dense_descriptors()
{
  /*
  _desc_left->compute_descriptors();
  _desc_left->normalize_descriptors();
  int descSize;
  int h = _img_left.rows;
  int w = _img_left.cols;
  descSize = _desc_left->descriptor_size();
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      float* thor = NULL;
      _desc_left->get_descriptor(y, x, thor);
      memcpy(_descLeftCache.ptr(y*w+x), thor, descSize*sizeof(float));
    }
  }
  _desc_right->compute_descriptors();
  _desc_right->normalize_descriptors();
  h = _img_right.rows;
  w = _img_right.cols;
  descSize = _desc_right->descriptor_size();
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      float* thor = NULL;
      _desc_right->get_descriptor(y, x, thor);
      memcpy(_descRightCache.ptr(y*w+x), thor, descSize*sizeof(float));
    }
  }
  _descLeftSet = Mat(_img_left.rows, _img_left.cols, CV_8UC1, Scalar(1));
  _descRightSet = Mat(_img_left.rows, _img_left.cols, CV_8UC1, Scalar(1));
  */
}