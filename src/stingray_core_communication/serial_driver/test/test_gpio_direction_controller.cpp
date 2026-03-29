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

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "serial_driver/gpio_direction_controller.hpp"

using drivers::serial_driver::GpioDirectionController;

namespace
{
class ScopedTempDir
{
public:
  ScopedTempDir()
  {
    const auto timestamp = std::chrono::steady_clock::now().time_since_epoch().count();
    m_path = std::filesystem::temp_directory_path() /
      ("serial_driver_gpio_direction_test_" + std::to_string(timestamp));
    std::filesystem::create_directories(m_path);
  }

  ~ScopedTempDir()
  {
    std::error_code ec;
    std::filesystem::remove_all(m_path, ec);
  }

  const std::filesystem::path & path() const
  {
    return m_path;
  }

private:
  std::filesystem::path m_path;
};

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream file(path);
  std::string value;
  std::getline(file, value);
  return value;
}
}  // namespace

TEST(GpioDirectionControllerTest, DisabledModeIsNoOp)
{
  ScopedTempDir temp_dir;
  GpioDirectionController controller(-1, temp_dir.path().string());

  EXPECT_FALSE(controller.is_enabled());
  EXPECT_NO_THROW(controller.initialize());
  EXPECT_NO_THROW(controller.set_tx());
  EXPECT_NO_THROW(controller.set_rx());
}

TEST(GpioDirectionControllerTest, InitTxRxSequence)
{
  ScopedTempDir temp_dir;
  const auto gpio_root = temp_dir.path();
  const auto gpio_dir = gpio_root / "gpio23";

  std::filesystem::create_directories(gpio_dir);
  std::ofstream(gpio_root / "export").close();
  std::ofstream(gpio_dir / "direction").close();
  std::ofstream(gpio_dir / "value").close();

  GpioDirectionController controller(23, gpio_root.string());

  ASSERT_TRUE(controller.is_enabled());
  ASSERT_NO_THROW(controller.initialize());
  EXPECT_EQ(read_file(gpio_dir / "direction"), "out");
  EXPECT_EQ(read_file(gpio_dir / "value"), "0");

  ASSERT_NO_THROW(controller.set_tx());
  EXPECT_EQ(read_file(gpio_dir / "value"), "1");

  ASSERT_NO_THROW(controller.set_rx());
  EXPECT_EQ(read_file(gpio_dir / "value"), "0");
}

TEST(GpioDirectionControllerTest, MissingSysfsRootThrows)
{
  ScopedTempDir temp_dir;
  const auto missing_root = temp_dir.path() / "missing";

  GpioDirectionController controller(12, missing_root.string());

  EXPECT_THROW(controller.initialize(), std::runtime_error);
}
