#ifndef ASTARPLANNER_H
#define ASTARPLANNER_H

#include "stereo.h"

class AStarPlanner
{
private:
  struct node {
    int id;
    Point p;
    float f, g, h;
    bool operator < (const node& rhs) const {
      return f <= rhs.f;
    }
    bool operator == (const node& rhs) const {
      return (p.x == rhs.p.x && p.y == rhs.p.y);
    }
  };
  node start, end;
  int max_x, max_y, inc;
  bool inGrid(Point p);
  float L1Dist(Point p, Point q);
  float L2Dist(Point p, Point q);
  multiset< node > open_list;
  vector< Point > path;
  int safe_radius;
  int bot_height;
  bool convex_world;
  Mat closed_list;
  JPP_Config _config;
public:
  int parent[100000];
  map< int, Point > grid_id;
  vector< node > graph;
  AStarPlanner(JPP_Config& conf);
  void setParams(Point s, Point e, int my, int i, int r, int bh, bool cw);
  void findPath(Stereo* stereo);
  void setPath(node n);
  vector< Point > getPath();
  Point clCoord(Point p);
  int getPathLength();
};

#endif // LOCALPLANNER_H
