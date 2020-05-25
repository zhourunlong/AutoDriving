// Copyright @2018 Pony AI Inc. All rights reserved.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <bits/stdc++.h>

using namespace cv;

void distort(const Mat img, Mat &ret, double k1, double k2) {
    int H = img.rows, W = img.cols;
    uchar *img_data = (uchar *)img.data, *ret_data = (uchar *)ret.data;
    float fx = W / 2.0, fy = H / 2.0;
    for (int j = 0; j < H; ++j)
        for (int i = 0; i < W; ++i) {
            float X = (i - W / 2.0) / fx, Y = (j - H / 2.0) / fy;
            float r2 = X * X + Y * Y, r4;
            r4 = r2 * r2;
            int nX = X * (1 + k1 * r2 + k2 * r4) * fx + W / 2.0, nY = Y * (1 + k1 * r2 + k2 * r4) * fy + H / 2.0;
            if (nX < 0 || nX >= W || nY < 0 || nY >= H) continue;
            int id = j * W * 3 + i * 3, nid = nY * W * 3 + nX * 3;
            for (int k = 0; k < 3; ++k) ret_data[id + k] = img_data[nid + k];
        }
}

int main() {
  Mat image, distorted;
  // ATTENTION!!! : please use absolute path for reading the data file.
  image = imread("/home/vectorzhou/AutoDriving/homework2/chessboard/chessboard_undistorted.png", CV_LOAD_IMAGE_COLOR);
  distorted = image.clone();
  distorted.setTo(0);
  distort(image, distorted, 0.1, 0.1);
  imshow("chessboard_undistorted", image);
  imshow("chessboard_distorted", distorted);
  imwrite("/home/vectorzhou/AutoDriving/homework2/chessboard/chessboard_distorted.png", distorted);
  waitKey(0);
  return 0;
}
