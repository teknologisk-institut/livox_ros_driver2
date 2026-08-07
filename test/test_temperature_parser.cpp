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

#include <cstring>
#include <vector>

#include "lds_lidar.h"

TEST(TemperatureParser, ConvertsHundredthsCelsius) {
  std::vector<uint8_t> storage(
    sizeof(LivoxLidarDiagInternalInfoResponse) +
    sizeof(LivoxLidarKeyValueParam) + sizeof(int32_t));
  auto *response = reinterpret_cast<LivoxLidarDiagInternalInfoResponse *>(storage.data());
  response->ret_code = 0;
  response->param_num = 1;

  auto *kv = reinterpret_cast<LivoxLidarKeyValueParam *>(response->data);
  kv->key = kKeyCoreTemp;
  kv->length = sizeof(int32_t);
  const int32_t raw_temperature = 4237;
  memcpy(kv->value, &raw_temperature, sizeof(raw_temperature));

  double temperature_c = 0.0;
  ASSERT_TRUE(livox_ros::LdsLidar::ParseCoreTemperature(response, temperature_c));
  EXPECT_DOUBLE_EQ(temperature_c, 42.37);
}

TEST(TemperatureParser, RejectsMissingCoreTemperature) {
  std::vector<uint8_t> storage(
    sizeof(LivoxLidarDiagInternalInfoResponse) +
    sizeof(LivoxLidarKeyValueParam) + sizeof(uint8_t));
  auto *response = reinterpret_cast<LivoxLidarDiagInternalInfoResponse *>(storage.data());
  response->ret_code = 0;
  response->param_num = 1;

  auto *kv = reinterpret_cast<LivoxLidarKeyValueParam *>(response->data);
  kv->key = kKeyCurWorkState;
  kv->length = sizeof(uint8_t);
  kv->value[0] = 1;

  double temperature_c = 0.0;
  EXPECT_FALSE(livox_ros::LdsLidar::ParseCoreTemperature(response, temperature_c));
}
