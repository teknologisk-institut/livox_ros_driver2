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

#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#ifdef WIN32
#include <winsock2.h>
#include <ws2def.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif // WIN32
#include "livox_lidar_api.h"
#include "comm/comm.h"
#include "comm/pub_handler.h"

#include "parse_cfg_file/parse_cfg_file.h"
#include "parse_cfg_file/parse_livox_lidar_cfg.h"

#include "call_back/lidar_common_callback.h"
#include "call_back/livox_lidar_callback.h"

using namespace std;

namespace livox_ros {

/** Const varible ------------------------------------------------------------*/
/** For callback use only */
LdsLidar *g_lds_ldiar = nullptr;

/** Global function for common use -------------------------------------------*/

/** Lds lidar function -------------------------------------------------------*/
LdsLidar::LdsLidar(double publish_freq)
    : Lds(publish_freq, kSourceRawLidar), 
      auto_connect_mode_(true),
      whitelist_count_(0),
      is_initialized_(false) {
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));
  ResetLdsLidar();
}

LdsLidar::~LdsLidar() {}

void LdsLidar::ResetLdsLidar(void) { ResetLds(kSourceRawLidar); }



bool LdsLidar::InitLdsLidar(const std::string& path_name) {
  if (is_initialized_) {
    printf("Lds is already inited!\n");
    return false;
  }

  if (g_lds_ldiar == nullptr) {
    g_lds_ldiar = this;
  }

  path_ = path_name;
  if (!InitLidars()) {
    return false;
  }
  SetLidarPubHandle();
  if (!Start()) {
    return false;
  }
  is_initialized_ = true;
  return true;
}

bool LdsLidar::InitLidars() {
  if (!ParseSummaryConfig()) {
    return false;
  }
  std::cout << "config lidar type: " << static_cast<int>(lidar_summary_info_.lidar_type) << std::endl;

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!InitLivoxLidar()) {
      return false;
    }
  }
  return true;
}


bool LdsLidar::Start() {
  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!LivoxLidarStart()) {
      return false;
    }
  }
  return true;
}

bool LdsLidar::ParseSummaryConfig() {
  return ParseCfgFile(path_).ParseSummaryInfo(lidar_summary_info_);
}

bool LdsLidar::InitLivoxLidar() {

  DisableLivoxSdkConsoleLogger();

  // parse user config
  LivoxLidarConfigParser parser(path_);
  std::vector<UserLivoxLidarConfig> user_configs;
  if (!parser.Parse(user_configs)) {
    std::cout << "failed to parse user-defined config" << std::endl;
  }

  // SDK initialization
  if (!LivoxLidarSdkInit(path_.c_str())) {
    std::cout << "Failed to init livox lidar sdk." << std::endl;
    return false;
  }

  // fill in lidar devices
  for (auto& config : user_configs) {
    uint8_t index = 0;
    int8_t ret = g_lds_ldiar->cache_index_.GetFreeIndex(kLivoxLidarType, config.handle, index);
    if (ret != 0) {
      std::cout << "failed to get free index, lidar ip: " << IpNumToString(config.handle) << std::endl;
      continue;
    }
    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[index]);
    p_lidar->lidar_type = kLivoxLidarType;
    p_lidar->livox_config = config;
    p_lidar->handle = config.handle;
    RegisterLidarHandle(config.handle);

    LidarExtParameter lidar_param;
    lidar_param.handle = config.handle;
    lidar_param.lidar_type = kLivoxLidarType;
    if (config.pcl_data_type == kLivoxLidarCartesianCoordinateLowData) {
      // temporary resolution
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x / 10;
      lidar_param.param.y     = config.extrinsic_param.y / 10;
      lidar_param.param.z     = config.extrinsic_param.z / 10;
    } else {
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x;
      lidar_param.param.y     = config.extrinsic_param.y;
      lidar_param.param.z     = config.extrinsic_param.z;
    }
    pub_handler().AddLidarsExtParam(lidar_param);
  }

  SetLivoxLidarInfoChangeCallback(LivoxLidarCallback::LidarInfoChangeCallback, g_lds_ldiar);
  return true;
}

void LdsLidar::SetLidarPubHandle() {
  pub_handler().SetPointCloudsCallback(LidarCommonCallback::OnLidarPointClounCb, g_lds_ldiar);
  pub_handler().SetImuDataCallback(LidarCommonCallback::LidarImuDataCallback, g_lds_ldiar);

  double publish_freq = Lds::GetLdsFrequency();
  pub_handler().SetPointCloudConfig(publish_freq);
}

void LdsLidar::RegisterLidarHandle(uint32_t handle) {
  std::lock_guard<std::mutex> lock(temperature_mutex_);
  temperature_samples_.emplace(handle, LidarTemperatureSample{});
}

void LdsLidar::QueryTemperature(uint32_t handle) {
  RegisterLidarHandle(handle);
  const livox_status status = QueryLivoxLidarInternalInfo(
    handle, LdsLidar::TemperatureCallback, this);
  if (status != kLivoxLidarStatusSuccess) {
    std::lock_guard<std::mutex> lock(temperature_mutex_);
    temperature_samples_[handle].last_query_status = status;
  }
}

void LdsLidar::QueryTemperatures() {
  std::vector<uint32_t> handles;
  {
    std::lock_guard<std::mutex> lock(temperature_mutex_);
    for (const auto &item : temperature_samples_) {
      handles.push_back(item.first);
    }
  }
  for (const uint32_t handle : handles) {
    QueryTemperature(handle);
  }
}

std::map<uint32_t, LidarTemperatureSample> LdsLidar::GetTemperatureSamples() const {
  std::lock_guard<std::mutex> lock(temperature_mutex_);
  return temperature_samples_;
}

bool LdsLidar::ParseCoreTemperature(
    const LivoxLidarDiagInternalInfoResponse *response, double &temperature_c) {
  if (response == nullptr || response->ret_code != 0) {
    return false;
  }

  uint32_t offset = 0;
  for (uint16_t i = 0; i < response->param_num; ++i) {
    const auto *kv = reinterpret_cast<const LivoxLidarKeyValueParam *>(
      &response->data[offset]);
    if (kv->key == kKeyCoreTemp && kv->length == sizeof(int32_t)) {
      int32_t core_temp_hundredths_c = 0;
      memcpy(&core_temp_hundredths_c, kv->value, sizeof(core_temp_hundredths_c));
      temperature_c = static_cast<double>(core_temp_hundredths_c) / 100.0;
      return true;
    }
    offset += sizeof(kv->key) + sizeof(kv->length) + kv->length;
  }
  return false;
}

void LdsLidar::TemperatureCallback(
    livox_status status, uint32_t handle,
    LivoxLidarDiagInternalInfoResponse *response, void *client_data) {
  if (client_data == nullptr) {
    return;
  }

  auto *lds_lidar = static_cast<LdsLidar *>(client_data);
  double temperature_c = 0.0;
  const bool valid = status == kLivoxLidarStatusSuccess &&
    ParseCoreTemperature(response, temperature_c);

  std::lock_guard<std::mutex> lock(lds_lidar->temperature_mutex_);
  auto &sample = lds_lidar->temperature_samples_[handle];
  sample.last_query_status = status;
  if (valid) {
    sample.valid = true;
    sample.temperature_c = temperature_c;
    sample.received_at = std::chrono::steady_clock::now();
  }
}

bool LdsLidar::LivoxLidarStart() {
  return true;
}

int LdsLidar::DeInitLdsLidar(void) {
  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    LivoxLidarSdkUninit();
    printf("Livox Lidar SDK Deinit completely!\n");
  }

  return 0;
}

void LdsLidar::PrepareExit(void) { DeInitLdsLidar(); }

}  // namespace livox_ros
