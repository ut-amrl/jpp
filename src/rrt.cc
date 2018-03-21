#include "rrt.h"

RRT::RRT()
{
  nSAD = nSADCache = nSADConvex = nSADConvexCache = 0;
}

void RRT::initRRT(Point s, Point e, int step, int iter, float bias, int r, int bh, bool c, JPP_Config conf)
{
  srand(time(NULL));
  root = new Node;
  root->parent = NULL;
  root->position = s;
  start = s;
  end = e;
  step_size = step;
  max_iter = iter;
  goal_bias = bias;
  nodes.push_back(root);
  obstacle_step_size = step_size/2;
  safe_radius = r;
  bot_height = bh;
  convex = c;
  _config = conf;
}

Point RRT::getRandomStateSpacePoint(int range_x, int range_y)
{
  float r = rand() / (float)RAND_MAX;
  if (r < goal_bias)
    return end;
  int x = rand() % range_x;
  int y = rand() % (2*range_y) - range_y;
  return Point(x,y);
}

RRT::Node* RRT::nearest(Point p)
{
  float minDist = 1e9;
  Node *closest = NULL;
  for(int i = 0; i < (int)nodes.size(); i++) {
    float dist = norm(p - nodes[i]->position);
    if (dist < minDist) {
      minDist = dist;
      closest = nodes[i];
    }
  }
  return closest;
}

Point RRT::extend(Node* n, Point to, Stereo* stereo)
{
  Point from = n->position;
  Point i = to - from;
  float norm_i = norm(i);
  Point unit_i((float)(i.x*step_size)/norm_i,(float)(i.y*step_size)/norm_i);
  Point q = from + unit_i;
  Point3f q3d((float)q.x/1000.,(float)q.y/1000.,0.);
  Point imgl = stereo->project_point_cam(q3d, 0);
  Point imgr = stereo->project_point_cam(q3d, 1);
  if (!stereo->in_img(imgl.x,imgl.y) || !stereo->in_img(imgr.x,imgr.y))
    return Point(0,0);
  return q;
}

void RRT::add(RRT::Node* q, RRT::Node* qNew)
{
  qNew->parent = q;
  q->children.push_back(qNew);
  nodes.push_back(qNew);
}

vector< Point > RRT::getPointsInLine(Point p, Point q)
{
  vector< Point > pts;
  int s = obstacle_step_size;
  Point a, b;
  if (p.x == q.x) {
    if (p.y < q.y) {
      a = p;
      b = q;
    } else {
      a = q;
      b = p;
    }
    for (int y = a.y; y <= b.y; y += s) {
      pts.push_back(Point(p.x, y));
    }
  } else {
    if (p.x < q.x) {
      a = p;
      b = q;
    } else {
      a = q;
      b = p;
    }
    float slope = (float)(b.y - a.y)/(float)(b.x - a.x);
    for (int x = a.x; x <= b.x; x += s) {
      int y = a.y + slope * (x - a.x);
      pts.push_back(Point(x,y));
    }
  }
  return pts;
}

bool RRT::isBotClearOfObstacle(Point p, Stereo* stereo)
{
  Point3f pt3d = Point3f((float)p.x/1000.,(float)p.y/1000.,0);
  return (stereo->is_bot_clear(pt3d, (float)safe_radius/1000., (float)step_size/1000., !convex));
}

void RRT::findPath(Stereo* stereo)
{
  for (int zz = 0; zz < max_iter; zz++) {
    //cout << zz << endl;
    Point rand_config = getRandomStateSpacePoint(6000,3000);
    Node* qNearest = nearest(rand_config);
    if (norm(qNearest->position-rand_config) > step_size) {
      Point newConfig = extend(qNearest, rand_config, stereo);
      if (newConfig == Point(0,0))
        continue;
      // start obstacle check
      bool inObstacle = false;
      vector< Point > pts = getPointsInLine(qNearest->position, newConfig);
      for (u_int i = 0; i < pts.size(); i++) {
        if (!isBotClearOfObstacle(pts[i], stereo)) {
          inObstacle = true;
          break;
        }
      }
      // end obstacle check
      if (inObstacle)
        continue;
      Node *qNew = new Node;
      qNew->position = newConfig;
      add(qNearest, qNew);
      if (norm(newConfig-end) < 50) {
        break;
      }
    }
  }
  Node *q = nearest(end);
  while (q != NULL) {
    path.push_back(q);
    q = q->parent;
  }
}

vector< Point > RRT::getPath()
{
  vector< Point > ret;
  for (u_int i = 0; i < path.size(); i++) {
    ret.push_back(path[i]->position);
  }
  return ret;
}

int RRT::getPathLength()
{
  int length = 0;
  if (!path.empty())
  {
    for (u_int i = 0; i < path.size()-1; i++) {
      Point p = path[i]->position;
      Point q = path[i+1]->position;
      length += norm(p-q);
    }
  }
  return length;
}

void RRT::freeMemory()
{
  for (u_int i = 0; i < nodes.size(); i++) {
    if (nodes[i] != NULL)
      delete nodes[i];
  }
}