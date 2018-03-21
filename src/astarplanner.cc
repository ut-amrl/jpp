#include "astarplanner.h"

AStarPlanner::AStarPlanner(JPP_Config& conf)
{
  closed_list = Mat(1000, 1000, CV_8UC1, Scalar(0));
  _config = conf;
  closedList = vector< int >((_config.MAX_X/_config.GRID_SIZE) * 
    ((_config.MAX_Y*2)/_config.GRID_SIZE), 0);
  // node init;
  // init.id = -1;
  // openListPointer = vector< node >((_config.MAX_X/_config.GRID_SIZE) * 
  //   ((_config.MAX_Y*2)/_config.GRID_SIZE), init);
  prevPath = {};
}

AStarPlanner::AStarPlanner(JPP_Config& conf, vector< Point > new_prevPath)
{
  closed_list = Mat(1000, 1000, CV_8UC1, Scalar(0));
  _config = conf;
  closedList = vector< int >((_config.MAX_X/_config.GRID_SIZE) * 
    ((_config.MAX_Y*2)/_config.GRID_SIZE), 0);
  // node init;
  // init.id = -1;
  // openListPointer = vector< node >((_config.MAX_X/_config.GRID_SIZE) * 
  //   ((_config.MAX_Y*2)/_config.GRID_SIZE), init);
  prevPath = new_prevPath;
}

int AStarPlanner::pointToIndex(Point p)
{
  return (p.x/inc)*((2*max_y)/inc) + (p.y + max_y)/inc;
}

bool AStarPlanner::inGrid(Point p)
{
  //return (p.x >= start.p.x && p.x <= max_x && p.y >= -max_y && p.y <= max_y);
  return (p.x >= 0 && p.x <= max_x && p.y >= -max_y && p.y <= max_y);
}

float AStarPlanner::turncost(node n, Point successor)
{
  if (n.id == -1)
    return -0.1;

  Point p1 = grid_id[parent[n.id]];
  Point p2 = grid_id[n.id];
  Point p3 = successor;

  // printf("p1.x: %d, p1.y: %d\n", p1.x, p1.y);
  // printf("p2.x: %d, p2.y: %d\n", p2.x, p2.y);
  // printf("p3.x: %d, p3.y: %d\n", p3.x, p3.y);

  Eigen::Vector2f v1(p2.x - p1.x, p2.y - p1.y);
  Eigen::Vector2f v2(p3.x - p2.x, p3.y - p2.y);

  // cout << "v1: \n" << v1 << endl;
  // cout << "v2: \n" << v2 << endl;
  v1 = v1.normalized();
  v2 = v2.normalized();
  // cout << "v1: \n" << v1 << endl;
  // cout << "v2: \n" << v2 << endl;
  float cost = fabs((acos(v1.dot(v2))*2)/M_PI);
  //cout << "turncost: " << cost << endl;
  return cost;
}

float AStarPlanner::L1Dist(Point p, Point q)
{
  float s = fabs(p.x - q.x) + fabs(p.y - q.y);
  return s;
}

float AStarPlanner::L2Dist(Point p, Point q)
{
  float s = sqrt((p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y));
  return s;
}

float AStarPlanner::dist_to_prevPath(Point p)
{
  if (prevPath.empty())
  {
    return 0;
  }
  float mindist = 9999.9;
  for (uint i = 0; i < prevPath.size(); i++)
  {
    float dist = L1Dist(prevPath[i], p);
    if (dist < mindist)
    {
      mindist = dist;
    }
  }
  return mindist;
}

void AStarPlanner::setParams(Point s, Point e, int my, int i, int r, int bh, 
bool cw)
{
  max_x = e.x;
  max_y = my;
  inc = i;
  start.p = s;
  start.g = start.h = 0;
  start.f = 0;
  start.id = 0;

  //pach
  Point realStart;
  realStart.x = 0;
  realStart.y = 0;

  grid_id[0] = realStart; //s;
  parent[0] = -1;
  end.p = e;
  safe_radius = r;
  convex_world = cw;
  bot_height = bh;

  Point goal = e;
  if (goal.x > _config.MAX_X) //when past grid
  {
    printf("goal.x: %d, goal.y: %d\n", goal.x, goal.y);
    //project goal to edge of planner grid
    int y_intercept = (int)round(((float)goal.y*(float)_config.MAX_X)/(float)goal.x);
    printf("y_intercept: %d\n", y_intercept);
    if (y_intercept > -_config.MAX_Y && y_intercept < _config.MAX_Y)
    {
      end.p.x = _config.MAX_X;
      end.p.y = y_intercept;
    }
    else
    {
      printf("bad\n");
      if (goal.y > 0)
      {
        path.push_back(Point(0, _config.MAX_X));
      }
      else
      {
        path.push_back(Point(0, -_config.MAX_X));
      }
    }
  }
  else if (goal.x >= _config.START_X && goal.x <= _config.MAX_X && // when in grid
    goal.y >= -_config.MAX_Y && goal.y <= _config.MAX_Y)
  {
    //do nothing
    printf("good\n");
  }
  else 
  {
    printf("duno\n");
    if (goal.y > 0)
    {
      path.push_back(Point(0, _config.MAX_X));
    }
    else
    {
      path.push_back(Point(0, -_config.MAX_X));
    }
}
}

Point AStarPlanner::clCoord(Point p)
{
  Point q = Point(-p.y + 3000, 9000 - p.x);
  Point r = Point(q.x * 0.1, q.y * 0.1);
  return r;
}

void AStarPlanner::printOpenList()
{
  // for(int i = 0; i < open_list.size(); i++)
  // {
  //   printf("(%f, %f, %f), ", open_list[i].p.x, open_list[i].p.y, open_list[i].p.z);
  // }
  // printf("\n");

  for (std::multiset<node>::const_iterator i = open_list.begin(); i != open_list.end(); ++i)
  {
    printf("(%d, %d), ", (*i).p.x, (*i).p.y);
  }
  printf("\n");
}

void AStarPlanner::printClosedList()
{
  for (int i = 0; i < max_x; i += inc)
  {
    for (int j = -max_y; j < max_y; j += inc)
    {
      Point p;
      p.x = i;
      p.y = j;
      //printf("%d, ", closedList[pointToIndex(p)]);
    }
    printf("\n");
  }
  printf("\n");
}

void AStarPlanner::findPath(Stereo* stereo)
{

  node robotCenter;
  robotCenter.p.x = 0;
  robotCenter.p.y = 0;
  robotCenter.g = 0;
  robotCenter.f = 0;
  robotCenter.id = 0;

  open_list.insert(robotCenter);//start
  node closestnode = robotCenter;//start
  closestnode.f = 1e9;
  float mindist = 1e9;
  multiset< node >::iterator ito;
  int x = 0;
  bool stop = false;
  for (;!open_list.empty() && !stop;) {
    ito = open_list.begin();
    node q = *ito;
    // if (closed_list_x.find(q.p.x) != closed_list_x.end() &&
    //     closed_list_y.find(q.p.y) != closed_list_y.end())
    // {
    //   //cout << "open list contains closed list!" << endl;
    //   open_list.erase(ito);
    //   continue;
    // }
    
    //printOpenList();
    closed_list_x.insert(q.p.x);
    closed_list_y.insert(q.p.y);

    // if (closedList[pointToIndex(q.p)] == 0)
    // {
    // printf("x: %d, y: %d\n", q.p.x/inc, (q.p.y + max_y)/inc);
    // printClosedList();
    // printf("cl: %d\n", closedList[pointToIndex(q.p)]);
    // closedList[pointToIndex(q.p)] = 1;
    // printClosedList();
    // printf("cl: %d\n", closedList[pointToIndex(q.p)]);
    // }

    // node empty;
    // empty.id = -1;
    // openListPointer[pointToIndex(q.p)] = empty;
    open_list.erase(ito);
    Point qcl = clCoord(q.p);

    // if (closedList[pointToIndex(q.p)] == 1)
    // {
    //   continue;
    // }
    // closedList[pointToIndex(q.p)] = 1;
    if ((int)closed_list.at<uchar>(qcl) == 255) {
      continue;
    }
    //world->img_world.at<Vec3b>(q_sim_coord) = Vec3b(0,0,255);
    //circle(closed_list, qcl, 1, Scalar(255), -1, 8, 0);
    closed_list.at<uchar>(qcl) = 255;
    graph.push_back(q);
    //cout << "add close list: " << q.p << " ------ " << endl;
    vector< Point > neighbours;
    neighbours.push_back(Point(inc,0));
    neighbours.push_back(Point(inc,inc));
    neighbours.push_back(Point(inc,-inc));
    neighbours.push_back(Point(0,inc));
    neighbours.push_back(Point(0,-inc));
    //neighbours.push_back(Point(-inc,inc));
    //neighbours.push_back(Point(-inc,0));
    //neighbours.push_back(Point(-inc,-inc));
    for (u_int i = 0; i < neighbours.size(); i++) {
      Point cur_pt = q.p + neighbours[i];
      if (!inGrid(cur_pt)) {
        continue;
      }
      Point curpt_cl = clCoord(cur_pt);
      if ((int)closed_list.at<uchar>(curpt_cl) == 255) {
        //cout << "closed " << cur_pt << endl;
        continue;
      }
      // if (closed_list_x.find(cur_pt.x) != closed_list_x.end() &&
      //   closed_list_y.find(cur_pt.y) != closed_list_y.end())
      // {
      //   continue;
      // }
      // if (closedList[pointToIndex(cur_pt)] == 1)
      // {
      //   continue;
      // }
      // obstacle check
      Point3f pt3d = Point3f((float)cur_pt.x/1000.,(float)cur_pt.y/1000.,0);
      
      //obstical check within grid
      float traversability = 0;
      if (cur_pt.x >= start.p.x)
      {
        traversability = stereo->traversability(pt3d, (float)safe_radius/1000., (float)inc/1000., !convex_world);
        //printf("traversability: %f\n", traversability);
        if (traversability == -1)
        {
          continue;
        }
      }
      //if (cur_pt.x >= start.p.x && !stereo->is_bot_clear(pt3d, (float)safe_radius/1000., (float)inc/1000., !convex_world))
        //continue;
      //obstical check blind to ground area before grid
      //else if (!stereo->is_bot_clear_blind_ground(pt3d, (float)safe_radius/1000., (float)inc/1000., !convex_world))
        //continue;
      
      // create successor node
      node suc;
      suc.p = cur_pt;
      // if goal is reached terminate
      if (L2Dist(cur_pt, end.p) < 2 * inc) {
        end = suc;
        end.id = ++x;
        parent[end.id] = q.id;
        grid_id[end.id] = cur_pt;
        setPath(end);
        stop = true;
        break;
      }

      // update scores of successor node
      //suc.g = q.g + L2Dist(suc.p, q.p) + turncost(q, suc.p)*float(inc);
      suc.g = q.g + L2Dist(suc.p, q.p); //+ traversability*2.0;
      //cout << "movment cost: " << suc.g << endl << flush;
      suc.h = L1Dist(end.p, cur_pt) + dist_to_prevPath(cur_pt)*1.1;
      suc.f = suc.g + suc.h;
      
      ito = open_list.find(suc);
      // if (openListPointer[pointToIndex(cur_pt)].id != -1)
      // {
      //   printf("should be in open list\n");
      //   ito = open_list.find(openListPointer[pointToIndex(cur_pt)]);
      // }
      // else
      // {
      //   ito = open_list.end();
      // }

      //float eps = 1e-5;
      if (ito != open_list.end()) {
        printf("updated open_list for (%d, %d)\n", (*ito).p.x, (*ito).p.y);
        node n = *ito;
        if (suc.g < n.g) {
          suc.id = n.id;
          parent[suc.id] = q.id;
          grid_id[suc.id] = cur_pt;
          open_list.erase(ito);
          open_list.insert(suc);
        }
      } else {
        x++;
        suc.id = x;
        grid_id[suc.id] = cur_pt;
        parent[suc.id] = q.id;

        open_list.insert(suc);
        //openListPointer[pointToIndex(suc.p)] = suc;
        //printf("no update, \n");
      }
      
      if (norm(suc.p - end.p) < mindist) {
        mindist = norm(suc.p - end.p);
        closestnode = suc;
      }

      if(x >= 10000)
      {
        //stop = true;
        cerr << "map maxed out!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl << flush;
        setPath(start);
        return;
      }
    }
  }
  if (!stop)
    setPath(closestnode);
}

void AStarPlanner::setPath(AStarPlanner::node n)
{
  cout << "path size: " << path.size() << endl;
  int id = n.id;
  if (id == -1) return;
  while (id != -1) {
    path.push_back(grid_id[id]);
    id = parent[id];
  }
}

vector< Point > AStarPlanner::getPath()
{
  return path;
}

int AStarPlanner::getPathLength()
{
  int length = 0;
  if (!path.empty())
  {
    for (u_int i = 0; i < path.size()-1; i++) {
      Point p = path[i];
      Point q = path[i+1];
      length += norm(p-q);
    }
  }
  return length;
}