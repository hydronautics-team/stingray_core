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

#ifndef SERIAL_DRIVER__GPIO_DIRECTION_CONTROLLER_HPP_
#define SERIAL_DRIVER__GPIO_DIRECTION_CONTROLLER_HPP_

#include <string>

namespace drivers
{
namespace serial_driver
{

class GpioDirectionController
{
public:
  explicit GpioDirectionController(
    int gpio_number = -1,
    const std::string & sysfs_root = "/sys/class/gpio");

  bool is_enabled() const;
  int gpio_number() const;

  void initialize();
  void set_tx();
  void set_rx();

private:
  std::string gpio_directory() const;
  std::string direction_path() const;
  std::string value_path() const;
  std::string export_path() const;

  static bool path_exists(const std::string & path);
  void wait_for_path(const std::string & path) const;
  void write_file(const std::string & path, const std::string & value) const;

  int m_gpio_number;
  std::string m_sysfs_root;
  bool m_initialized{false};
};

}  // namespace serial_driver
}  // namespace drivers

#endif  // SERIAL_DRIVER__GPIO_DIRECTION_CONTROLLER_HPP_
