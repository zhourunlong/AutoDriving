// Copyright @2019 Pony AI Inc. All rights reserved.

#include "homework3/single_frame_detector.h"
#include <bits/stdc++.h>
#include "glog/logging.h"
#include <tr1/unordered_map>

struct hashfunc {
  template<typename T, typename U>
  size_t operator()(const std::pair<T, U> &i) const {
    return std::hash<T>()(i.first) ^ std::hash<U>()(i.second);
  }
};

inline void cmin(double &a, double b) {if (b < a) a = b;}
inline void cmax(double &a, double b) {if (b > a) a = b;}
inline void upd(double &a, double &b, double &c) {
    if (c < a) a = c;
    if (c > b) b = c;
}
const int dx[4] = {-1, 0, 1, 0}, dy[4] = {0, 1, 0, -1};

struct vec{double x, y;};
vec operator -(vec a, vec b) {return (vec){a.x - b.x, a.y - b.y};}
double cross(vec a, vec b) {return a.x * b.y - a.y * b.x;}
bool operator <(vec a, vec b) {
    if (fabs(a.x - b.x) > 1e-9) return a.x < b.x;
    return a.y < b.y;
}
double len(vec a) {return a.x * a.x + a.y * a.y;}

std::string segmentation(double area, double maxD, double height) {
    if (height < 0.7) return "fence";
    if (height < 3) {
        if (area > 2 && maxD / area > 2) return "car";
        if (maxD / area < 2) return "wall/fence";
        return "pedestrian";
    }
    if (height < 5) return "tree";
    if (area > 10) return "building";
    return "tree";
}

namespace {

// If you want to put all your implementation inside this file, you can add some functions here.
void ASampleFunction() {
  LOG(ERROR) << "Ground points have been detected.";
}

}  // namespace

SingleFrameDetector::SingleFrameDetector() {
  // TODO(you): optional, if you need to do anything to initialize the detector, you can do it here.
  // For example, you can load offline computed ground information here.
}

void SingleFrameDetector::GetGroundAndObstacles(
    const PointCloud point_cloud[3],
    std::vector<Eigen::Vector3d>* ground_points,
    std::vector<Obstacle>* obstacles) {
  CHECK(ground_points != nullptr);
  CHECK(obstacles != nullptr);
  // TODO(you): add some code to detect the ground and put all the points insidethe ground_points.
  //  I provide some trivial code here. Please replace them with a better implementation.
  std::tr1::unordered_map <std::pair<int, int>, double, hashfunc> map_pt;
  std::tr1::unordered_map <std::pair<int, int>, int, hashfunc> map_pt_cnt;
  std::vector <double> vZ;
  for (int i = 0; i < 3; ++i) 
    for (const Eigen::Vector3d& point : point_cloud[i].points) {
      Eigen::Vector3d real_pt = point_cloud[i].rotation * point + point_cloud[i].translation;
      int cx = real_pt.x() * 2, cy = real_pt.y() * 2;
      std::pair <int, int> pos = std::make_pair(cx, cy);
      vZ.push_back(real_pt.z());
      if (map_pt.find(pos) == map_pt.end()) map_pt[pos] = real_pt.z();
      else cmin(map_pt[pos], real_pt.z());
    }
  double minZ;
  nth_element(vZ.begin(), vZ.size() * 0.01 + vZ.begin(), vZ.end()); 
  minZ = vZ[vZ.size() * 0.01];
  for (int i = 0; i < 3; ++i) 
    for (const Eigen::Vector3d& point : point_cloud[i].points) {
      Eigen::Vector3d real_pt = point_cloud[i].rotation * point + point_cloud[i].translation;
      int cx = real_pt.x() * 2, cy = real_pt.y() * 2;
      std::pair <int, int> pos = std::make_pair(cx, cy);
      if (real_pt.z() < map_pt[pos] + 0.2 && real_pt.z() < minZ + 10) {
        if (i == 1) ground_points->push_back(real_pt);
      } else {
        std::pair <int, int> pos = std::make_pair(int(real_pt.x()), int(real_pt.y()));
        ++map_pt_cnt[pos];
      }
    }
  // TODO(you): Run flood fill or other algorithms to get the polygons for the obstacles.
  // Still, I provide a fake implementation, please replace it.
  ASampleFunction();
  std::tr1::unordered_map <std::pair<int, int>, int, hashfunc> :: iterator itr;
  int cnt = 0;
  for (itr = map_pt_cnt.begin(); itr != map_pt_cnt.end(); ++itr)
    if (itr->second < 30) map_pt_cnt.erase(itr);
  for (itr = map_pt_cnt.begin(); itr != map_pt_cnt.end(); ++itr)
    itr->second = cnt++;
  std::vector <int> edge[cnt];
  for (itr = map_pt_cnt.begin(); itr != map_pt_cnt.end(); ++itr) {
    int x = itr->first.first, y = itr->first.second;
    for (int k = 0; k < 4; ++k) {
      int tx = x + dx[k], ty = y + dy[k];
      std::pair <int, int> pos = std::make_pair(tx, ty);
      if (map_pt_cnt.find(pos) != map_pt_cnt.end()) {
        int q = map_pt_cnt[pos];
        edge[itr->second].push_back(q);
        edge[q].push_back(itr->second);
      }
    }
  }
  int group_cnt = 0;
  int vis[cnt], grp[cnt];
  for (int i = 0; i < cnt; ++i) vis[i] = 0;
  for (int i = 0; i < cnt; ++i)
    if (!vis[i]) {
      std::queue <int> Q;
      vis[i] = 1; Q.push(i);
      while (!Q.empty()) {
        int j = Q.front(); Q.pop(); grp[j] = group_cnt;
        for (auto k : edge[j])
          if (!vis[k]) {vis[k] = 1; Q.push(k);}
      }
      ++group_cnt;
    }
  std::vector <Eigen::Vector3d> lst[group_cnt];
  for (int i = 0; i < 3; ++i) 
    for (const Eigen::Vector3d& point : point_cloud[i].points) {
      Eigen::Vector3d real_pt = point_cloud[i].rotation * point + point_cloud[i].translation;
      int cx = real_pt.x(), cy = real_pt.y();
      std::pair <int, int> pos = std::make_pair(cx, cy);
      if (map_pt_cnt.find(pos) != map_pt_cnt.end())
          lst[grp[map_pt_cnt[pos]]].push_back(real_pt); 
    }
  for (int i = 0; i < group_cnt; ++i) {
    if (lst[i].size() < 100) continue;
    double minZ = 999999999, maxZ = -999999999;
    std::vector <vec> Q, convex;
    for (auto real_pt : lst[i]) {
      upd(minZ, maxZ, real_pt.z());
      Q.push_back((vec){real_pt.x(), real_pt.y()});
    }
    sort(Q.begin(), Q.end());
    convex.push_back(Q[0]);
    convex.push_back(Q[1]);
    int top = 1;
    for (int j = 2; j < Q.size(); ++j) {
      while (top >= 1 && cross(Q[j] - convex[top - 1], convex[top] - convex[top - 1]) > -1e-9) {
        convex.pop_back();
        --top;
      }
      convex.push_back(Q[j]); ++top;
    }
    convex.push_back(Q[Q.size() - 2]);
    int top2 = top + 1;
    for (int j = Q.size() - 1; j > -1; --j) {
      while (top2 > top && cross(Q[j] - convex[top2 - 1], convex[top2] - convex[top2 - 1]) > -1e-9) {
        convex.pop_back();
        --top2;
      }
      convex.push_back(Q[j]); ++top2;
    }
    double area = cross(convex[top2], convex[0]);
    for (int j = 0; j < top2; ++j)
      area += cross(convex[j], convex[j + 1]);
    area /= 2;
    if (area < 0.5 || maxZ - minZ < 0.5) continue;
    double maxD = 0;
    for (int i = 1; i < top2; ++i)
      for (int j = i + 1; j <= top2; ++j)
        cmax(maxD, fabs(len(convex[j] - convex[i])));
    obstacles->emplace_back();
    for (int j = 0; j <= top2; ++j)
      obstacles->back().polygon.emplace_back(convex[j].x, convex[j].y);
    obstacles->back().floor = minZ - 0.1;
    obstacles->back().ceiling = maxZ + 0.1;
    obstacles->back().id = segmentation(area, maxD, maxZ - minZ);
  }
}
