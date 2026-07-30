//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <gtest/gtest.h>

#include <chrono>
#include <future>

#include "lds_lidar.h"

using namespace std::chrono_literals;

TEST(LdsShutdown, RequestExitWakesBlockedDataThreads) {
  auto * lds = livox_ros::LdsLidar::GetInstance(10.0);

  auto pointcloud_wait = std::async(std::launch::async, [lds]() {
    lds->pcd_semaphore_.Wait();
  });
  auto imu_wait = std::async(std::launch::async, [lds]() {
    lds->imu_semaphore_.Wait();
  });

  EXPECT_EQ(pointcloud_wait.wait_for(20ms), std::future_status::timeout);
  EXPECT_EQ(imu_wait.wait_for(20ms), std::future_status::timeout);

  lds->RequestExit();

  EXPECT_EQ(pointcloud_wait.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(imu_wait.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(lds->IsRequestExit());
}
