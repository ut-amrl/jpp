#ifndef RRT_H
#define RRT_H

#include "stereo.h"

class RRT
{
private:
  struct Node {
    Node *parent;
    vector<Node *> children;
    Point position;
  };
  Node *root;
  Point start, end;
  int step_size, max_iter;
  int obstacle_step_size;
  float goal_bias;
  int safe_radius;
  int bot_height;
  bool convex;
  JPP_Config _config;
public:
  vector< Node* > nodes;
  vector< Node* > path;
  int nSAD, nSADConvex, nSADCache, nSADConvexCache;
  
  RRT();
  void initRRT(Point s, Point e, int step, int iter, float bias, int r, int bh, bool c, JPP_Config conf);
  Point getRandomStateSpacePoint(int range_x, int range_y);
  Node* nearest(Point p);
  Point extend(Node *n, Point to, Stereo* stereo);
  void add(Node *q, Node *qNew);
  vector< Point > getPointsInLine(Point p, Point q);
  bool isBotClearOfObstacle(Point p, Stereo* stereo);
  void findPath(Stereo* stereo);
  vector< Point > getPath();
  int getPathLength();
  void freeMemory();
};

#endif // RRT_H
