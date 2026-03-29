// Copyright 2026 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "serial_driver/gpio_direction_controller.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <thread>

namespace drivers
{
namespace serial_driver
{

GpioDirectionController::GpioDirectionController(int gpio_number, const std::string & sysfs_root)
: m_gpio_number(gpio_number),
  m_sysfs_root(sysfs_root)
{
}

bool GpioDirectionController::is_enabled() const
{
  return m_gpio_number >= 0;
}

int GpioDirectionController::gpio_number() const
{
  return m_gpio_number;
}

std::string GpioDirectionController::gpio_directory() const
{
  return m_sysfs_root + "/gpio" + std::to_string(m_gpio_number);
}

std::string GpioDirectionController::direction_path() const
{
  return gpio_directory() + "/direction";
}

std::string GpioDirectionController::value_path() const
{
  return gpio_directory() + "/value";
}

std::string GpioDirectionController::export_path() const
{
  return m_sysfs_root + "/export";
}

bool GpioDirectionController::path_exists(const std::string & path)
{
  return std::filesystem::exists(path);
}

void GpioDirectionController::wait_for_path(const std::string & path) const
{
  constexpr int max_attempts = 50;
  constexpr auto delay = std::chrono::milliseconds(10);

  for (int i = 0; i < max_attempts; ++i) {
    if (path_exists(path)) {
      return;
    }
    std::this_thread::sleep_for(delay);
  }

  throw std::runtime_error("Timed out waiting for GPIO path: " + path);
}

void GpioDirectionController::write_file(const std::string & path, const std::string & value) const
{
  std::ofstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open GPIO sysfs file: " + path);
  }

  file << value;
  if (!file) {
    throw std::runtime_error("Failed to write GPIO sysfs file: " + path);
  }
}

void GpioDirectionController::initialize()
{
  if (!is_enabled() || m_initialized) {
    return;
  }

  if (!path_exists(m_sysfs_root)) {
    throw std::runtime_error("GPIO sysfs root does not exist: " + m_sysfs_root);
  }

  if (!path_exists(gpio_directory())) {
    write_file(export_path(), std::to_string(m_gpio_number));
    wait_for_path(gpio_directory());
  }

  write_file(direction_path(), "out");
  write_file(value_path(), "1");
  m_initialized = true;
}

void GpioDirectionController::set_tx()
{
  if (!is_enabled()) {
    return;
  }

  initialize();
  write_file(value_path(), "0");
}

void GpioDirectionController::set_rx()
{
  if (!is_enabled()) {
    return;
  }

  initialize();
  write_file(value_path(), "1");
}

}  // namespace serial_driver
}  // namespace drivers
