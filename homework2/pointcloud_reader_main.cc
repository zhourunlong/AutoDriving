// Copyright @2018 Pony AI Inc. All rights reserved.

#include <iostream>

#include "homework2/pointcloud.h"

int main() {
  // ATTENTION!!! : please use absolute path for reading the data file.
  const PointCloud pointcloud = ReadPointCloudFromTextFile(
      "/home/vectorzhou/PublicCourse/homework2/data/src.txt");
  /*
  std::cout << "Total points read: " << pointcloud.points.size() << std::endl;
  std::cout << "Rotation: " << std::endl;
  std::cout << pointcloud.rotation << std::endl;
  std::cout << "Translation: " << std::endl;
  std::cout << pointcloud.translation.transpose() << std::endl;
  */
  freopen("/home/vectorzhou/PublicCourse/homework2/histogram/pts.txt", "w", stdout);
  std::cout << pointcloud.points.size() << std::endl;
  for (auto &p : pointcloud.points)
    std::cout << p(0) << "\n" << p(1) << "\n" << p(2) << "\n";
  fclose(stdout);
  return 0;
}
