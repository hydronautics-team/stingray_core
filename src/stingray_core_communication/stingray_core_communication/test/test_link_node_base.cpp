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

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "link_node_base.hpp"

namespace
{
class SerialLoopbackNode : public rclcpp::Node
{
public:
  SerialLoopbackNode()
  : Node("serial_loopback_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_read", 20);
    subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "serial_write", 20,
      [this](const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
      {
        publisher_->publish(*msg);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
};

class MasterLinkNode : public stingray_core::baseLink::LinkNodeBase
{
public:
  MasterLinkNode()
  : LinkNodeBase("test_master_link_node", 2, 1)
  {
  }

  void send_write(const std::vector<uint8_t> & data, unsigned address)
  {
    serialWrite(data.data(), static_cast<int>(address), static_cast<unsigned>(data.size()));
  }

  void send_read(void * data, unsigned address, unsigned length)
  {
    serialRead(data, static_cast<int>(address), length);
  }
};

class SlaveLinkNode : public stingray_core::baseLink::LinkNodeBase
{
public:
  SlaveLinkNode()
  : LinkNodeBase(
      "test_slave_link_node", 1, 2,
      [this](void * buffer, unsigned address, unsigned length)
      {
        return this->memory_read(buffer, address, length);
      },
      [this](const void * buffer, unsigned address, unsigned length)
      {
        return this->memory_write(buffer, address, length);
      })
  {
    for (size_t i = 0; i < memory_.size(); ++i) {
      memory_[i] = static_cast<uint8_t>(i);
    }
  }

  int write_calls() const
  {
    return write_calls_.load();
  }

  int read_calls() const
  {
    return read_calls_.load();
  }

  unsigned last_written_address() const
  {
    return last_written_address_.load();
  }

  std::vector<uint8_t> last_written_data() const
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    return last_written_data_;
  }

  std::vector<uint8_t> memory_slice(unsigned address, unsigned length) const
  {
    if (address + length > memory_.size()) {
      return {};
    }

    return std::vector<uint8_t>(
      memory_.begin() + static_cast<std::ptrdiff_t>(address),
      memory_.begin() + static_cast<std::ptrdiff_t>(address + length));
  }

private:
  hydrolib::ReturnCode memory_read(void * buffer, unsigned address, unsigned length)
  {
    read_calls_.fetch_add(1);

    if (address + length > memory_.size()) {
      return hydrolib::ReturnCode::FAIL;
    }

    std::memcpy(buffer, memory_.data() + address, length);
    return hydrolib::ReturnCode::OK;
  }

  hydrolib::ReturnCode memory_write(const void * buffer, unsigned address, unsigned length)
  {
    write_calls_.fetch_add(1);
    last_written_address_.store(address);

    if (address + length > memory_.size()) {
      return hydrolib::ReturnCode::FAIL;
    }

    {
      std::lock_guard<std::mutex> lock(write_mutex_);
      const auto * bytes = static_cast<const uint8_t *>(buffer);
      last_written_data_.assign(bytes, bytes + length);
    }

    std::memcpy(memory_.data() + address, buffer, length);
    return hydrolib::ReturnCode::OK;
  }

  std::array<uint8_t, 256> memory_{};
  std::atomic<int> write_calls_{0};
  std::atomic<int> read_calls_{0};
  std::atomic<unsigned> last_written_address_{0};
  mutable std::mutex write_mutex_;
  std::vector<uint8_t> last_written_data_;
};

bool spin_until(
  const std::function<bool()> & predicate,
  rclcpp::executors::SingleThreadedExecutor & executor,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }

    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  return predicate();
}

class LinkNodeBaseProtocolTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(LinkNodeBaseProtocolTest, MasterWriteIsProcessedBySlave)
{
  auto loopback = std::make_shared<SerialLoopbackNode>();
  auto master = std::make_shared<MasterLinkNode>();
  auto slave = std::make_shared<SlaveLinkNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(loopback);
  executor.add_node(master);
  executor.add_node(slave);

  const std::vector<uint8_t> payload{0x10, 0x20, 0x30, 0x40};
  constexpr unsigned address = 7;
  master->send_write(payload, address);

  const auto got_write = spin_until(
    [slave, payload]()
    {
      return slave->write_calls() > 0 && slave->last_written_address() == address &&
             slave->last_written_data() == payload;
    },
    executor, std::chrono::milliseconds(500));

  EXPECT_TRUE(got_write);
  EXPECT_EQ(slave->memory_slice(address, static_cast<unsigned>(payload.size())), payload);

  executor.remove_node(slave);
  executor.remove_node(master);
  executor.remove_node(loopback);
}

TEST_F(LinkNodeBaseProtocolTest, MasterReadReceivesSlaveMemory)
{
  auto loopback = std::make_shared<SerialLoopbackNode>();
  auto master = std::make_shared<MasterLinkNode>();
  auto slave = std::make_shared<SlaveLinkNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(loopback);
  executor.add_node(master);
  executor.add_node(slave);

  constexpr unsigned address = 5;
  constexpr unsigned length = 6;
  std::array<uint8_t, length> response{};

  master->send_read(response.data(), address, length);

  const auto expected = slave->memory_slice(address, length);
  const auto got_response = spin_until(
    [&response, &expected, slave]()
    {
      return slave->read_calls() > 0 &&
             std::vector<uint8_t>(response.begin(), response.end()) == expected;
    },
    executor, std::chrono::milliseconds(500));

  EXPECT_TRUE(got_response);
  EXPECT_EQ(std::vector<uint8_t>(response.begin(), response.end()), expected);

  executor.remove_node(slave);
  executor.remove_node(master);
  executor.remove_node(loopback);
}

}  // namespace
