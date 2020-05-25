// Copyright @2018 Pony AI Inc. All rights reserved.
//
#include <gtest/gtest.h>
#include "homework1/protobuf/canvas.h"

namespace homework1 {

TEST(Canvas, Serialization) {
  Canvas canvas;
  canvas.AddPoint(1.0, 2.0, 3.0);

  const std::string serialization = canvas.SerializeToString();

  Canvas other_canvas;
  other_canvas.ParseFromString(serialization);
  ASSERT_EQ(1, other_canvas.point_size());
  EXPECT_NEAR(1.0, other_canvas.GetPoint(0).x(), 1e-6);
  EXPECT_NEAR(2.0, other_canvas.GetPoint(0).y(), 1e-6);
  EXPECT_NEAR(3.0, other_canvas.GetPoint(0).z(), 1e-6);
}

TEST(Canvas, ComputeLength) {
  Canvas canvas;
  EXPECT_NEAR(0.0, canvas.ComputeLength(), 1e-6);
  canvas.AddPoint(0.0, 0.0, 0.0);
  EXPECT_NEAR(0.0, canvas.ComputeLength(), 1e-6);
  canvas.AddPoint(1.0, 2.0, 3.0);
  EXPECT_NEAR(3.7416573, canvas.ComputeLength(), 1e-6);
  canvas.AddPoint(4.0, 5.0, 6.0);
  EXPECT_NEAR(8.9378098, canvas.ComputeLength(), 1e-6);
  canvas.AddPoint(996.0, 251.0, -233.0);
  EXPECT_NEAR(1058.5572647, canvas.ComputeLength(), 1e-6);
}

}  // namespace homework1
