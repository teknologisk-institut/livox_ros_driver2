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

#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <chrono>
#include <iomanip>
#include <limits>
#include <sstream>

namespace livox_ros {

namespace {

diagnostic_msgs::msg::KeyValue MakeKeyValue(
    const std::string &key, const std::string &value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

}  // namespace

DriverNode& DriverNode::GetNode() noexcept {
  return *this;
}

DriverNode::~DriverNode() {
  temperature_timer_.reset();
  lddc_ptr_->lds_->RequestExit();
  exit_signal_.set_value();
  pointclouddata_poll_thread_->join();
  imudata_poll_thread_->join();
}

void DriverNode::TemperaturePoll() {
  auto *lds_lidar = dynamic_cast<LdsLidar *>(lddc_ptr_->lds_);
  if (lds_lidar == nullptr) {
    return;
  }

  lds_lidar->QueryTemperatures();

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = now();
  const auto steady_now = std::chrono::steady_clock::now();
  for (const auto &item : lds_lidar->GetTemperatureSamples()) {
    const uint32_t handle = item.first;
    const LidarTemperatureSample &sample = item.second;
    const std::string ip = IpNumToString(handle);

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "lidar/temperature/" + ip;
    status.hardware_id = ip;
    status.values.push_back(
      MakeKeyValue("last_query_status", std::to_string(sample.last_query_status)));

    const double age_s = sample.valid ?
      std::chrono::duration<double>(steady_now - sample.received_at).count() :
      std::numeric_limits<double>::infinity();
    if (!sample.valid || age_s > temperature_stale_timeout_s_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = sample.valid ? "Temperature data stale" : "No temperature data";
    } else {
      sensor_msgs::msg::Temperature temperature;
      temperature.header = diagnostics.header;
      temperature.header.frame_id = temperature_frame_id_;
      temperature.temperature = sample.temperature_c;
      temperature.variance = 0.0;
      temperature_pub_->publish(temperature);

      if (sample.temperature_c >= temperature_error_c_) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Core temperature critical";
      } else if (sample.temperature_c >= temperature_warn_c_) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Core temperature high";
      } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "Core temperature normal";
      }
    }

    if (sample.valid) {
      std::ostringstream temperature_value;
      temperature_value << std::fixed << std::setprecision(2) << sample.temperature_c;
      status.values.push_back(MakeKeyValue("temperature_c", temperature_value.str()));
      status.values.push_back(MakeKeyValue("age_s", std::to_string(age_s)));
    }
    status.values.push_back(MakeKeyValue("warn_c", std::to_string(temperature_warn_c_)));
    status.values.push_back(MakeKeyValue("error_c", std::to_string(temperature_error_c_)));
    diagnostics.status.push_back(status);
  }

  diagnostics_pub_->publish(diagnostics);
}

} // namespace livox_ros




