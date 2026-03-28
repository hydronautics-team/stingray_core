#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "hydrolib_bus_application_master.hpp"
#include "hydrolib_bus_application_slave.hpp"
#include "hydrolib_bus_datalink_stream.hpp"
#include "hydrolib_log_distributor.hpp"
#include "hydrolib_return_codes.hpp"
#include "hydrolib_ring_queue.h"

namespace stingray_core::baseLink
{
class TestLogStream
{
};

inline int write([[maybe_unused]] TestLogStream &stream,
                 [[maybe_unused]] const void *dest,
                 unsigned length)
{
    return static_cast<int>(length);
}

constexpr char *kSerializer = "Serializer";
constexpr char *kLogFormat = "[%s] [%l] %m\n";

inline TestLogStream log_stream;
inline hydrolib::logger::LogDistributor<TestLogStream> distributor{kLogFormat,
                                                                    log_stream};
inline hydrolib::logger::Logger<
    hydrolib::logger::LogDistributor<TestLogStream>>
    logger{kSerializer, 0, distributor};

class LinkNodeBase;
int write(LinkNodeBase &stream, const void *data, unsigned length);
int read(LinkNodeBase &stream, void *data, unsigned length);

class LinkNodeBase : public rclcpp::Node
{
public:
    using DistributorType = hydrolib::logger::LogDistributor<TestLogStream>;
    using StreamManagerType =
        hydrolib::bus::datalink::StreamManager<LinkNodeBase, DistributorType>;
    using StreamType =
        hydrolib::bus::datalink::Stream<LinkNodeBase, DistributorType>;
    using MasterType = hydrolib::bus::application::Master<StreamType,
                                                          DistributorType>;
    using MemoryReadCallback =
        std::function<hydrolib::ReturnCode(void *, unsigned, unsigned)>;
    using MemoryWriteCallback =
        std::function<hydrolib::ReturnCode(const void *, unsigned, unsigned)>;

    LinkNodeBase(std::string node_name, int self_addr, int dev_addr)
        : LinkNodeBase(std::move(node_name), self_addr, dev_addr,
                       MemoryReadCallback{}, MemoryWriteCallback{})
    {
    }

    LinkNodeBase(std::string node_name, int self_addr, int dev_addr,
                 MemoryReadCallback read_cb, MemoryWriteCallback write_cb)
        : Node(std::move(node_name)),
          mode_(read_cb && write_cb ? Mode::kSlave : Mode::kMaster),
          stream_manager_(self_addr, *this, logger),
          stream_(stream_manager_, dev_addr),
          master_(stream_, logger),
          slave_memory_(std::move(read_cb), std::move(write_cb)),
          slave_(stream_, slave_memory_, logger)
    {
        serial_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "serial_write", 10);
        serial_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "serial_read", 10,
            std::bind(&LinkNodeBase::readCallback, this, std::placeholders::_1));
        hydrolib_RingQueue_Init(&tx_queue_, work_buf_tx_queue_.data(), kQueueLen);

        RCLCPP_INFO(this->get_logger(), "Link node initialized in %s mode",
                    mode_ == Mode::kMaster ? "master" : "slave");
    }

public:
    int write(const void *data, unsigned length)
    {
        auto serial_msg = std_msgs::msg::UInt8MultiArray();
        serial_msg.data = createPacket(data, length);
        serial_pub_->publish(serial_msg);
        return static_cast<int>(length);
    }

    int read(void *data, unsigned length)
    {
        const auto len = hydrolib_RingQueue_GetLength(&tx_queue_);
        const auto bytes_to_read = static_cast<uint16_t>(
            length > len ? len : static_cast<uint16_t>(length));
        hydrolib_RingQueue_Pull(&tx_queue_, data, bytes_to_read);
        return bytes_to_read;
    }

    void serialWrite(const void *data, int memory_addr, unsigned length)
    {
        master_.Write(data, memory_addr, length);
    }

    void serialRead(void *data, int memory_addr, unsigned length)
    {
        master_.Read(data, memory_addr, length);
    }

protected:
    void processIncoming()
    {
        stream_manager_.Process();

        if (mode_ == Mode::kSlave)
        {
            slave_.Process();
            return;
        }

        (void)master_.Process();
    }

private:
    enum class Mode
    {
        kMaster,
        kSlave,
    };

    class SlaveMemory
    {
    public:
        SlaveMemory(MemoryReadCallback read_cb = {},
                    MemoryWriteCallback write_cb = {})
            : read_cb_(std::move(read_cb)), write_cb_(std::move(write_cb))
        {
        }

        hydrolib::ReturnCode Read(void *read_buffer, unsigned address,
                                  unsigned length)
        {
            if (!read_cb_)
            {
                return hydrolib::ReturnCode::FAIL;
            }
            return read_cb_(read_buffer, address, length);
        }

        hydrolib::ReturnCode Write(const void *write_buffer, unsigned address,
                                   unsigned length)
        {
            if (!write_cb_)
            {
                return hydrolib::ReturnCode::FAIL;
            }
            return write_cb_(write_buffer, address, length);
        }

    private:
        MemoryReadCallback read_cb_;
        MemoryWriteCallback write_cb_;
    };

    using SlaveType =
        hydrolib::bus::application::Slave<SlaveMemory, DistributorType,
                                          StreamType>;

    static std::vector<uint8_t> createPacket(const void *data, unsigned length)
    {
        const auto *value = static_cast<const uint8_t *>(data);
        return std::vector<uint8_t>(value, value + length);
    }

    void readCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        if (msg->data.empty())
        {
            return;
        }

        const void *data = msg->data.data();
        const auto length = static_cast<unsigned>(msg->data.size());
        hydrolib_RingQueue_Push(&tx_queue_, data, static_cast<uint16_t>(length));
        processIncoming();
    }

    Mode mode_;

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_pub_;

    StreamManagerType stream_manager_;
    StreamType stream_;
    MasterType master_;
    SlaveMemory slave_memory_;
    SlaveType slave_;

    hydrolib_RingQueue tx_queue_;
    static constexpr uint16_t kQueueLen{UINT8_MAX};
    std::array<uint8_t, kQueueLen> work_buf_tx_queue_{};
};

inline int write(LinkNodeBase &stream, const void *data, unsigned length)
{
    return stream.write(data, length);
}

inline int read(LinkNodeBase &stream, void *data, unsigned length)
{
    return stream.read(data, length);
}

} // namespace stingray_core::baseLink
