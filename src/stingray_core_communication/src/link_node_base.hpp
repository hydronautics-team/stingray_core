#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "hydrolib_bus_application_master.hpp"
#include "hydrolib_bus_application_slave.hpp"
#include "hydrolib_bus_datalink_stream.hpp"
#include "hydrolib_log_distributor.hpp"
#include "hydrolib_ring_queue.h"

namespace stingray_core::baseLink
{
class TestLogStream
{
};

int write([[maybe_unused]] TestLogStream &stream, const void *dest, unsigned length)
{
    for (unsigned i = 0; i < length; i++)
    {
    }
    return length;
}

constexpr char *_ser = "Serializer";
constexpr char *_distr = "[%s] [%l] %m\n";

TestLogStream log_stream;
hydrolib::logger::LogDistributor<TestLogStream> distributor{_distr, log_stream};
hydrolib::logger::Logger<hydrolib::logger::LogDistributor<TestLogStream>> logger{_ser, 0, distributor};

class LinkNodeBase;
int write(LinkNodeBase &stream, const void *data, unsigned lenght);
int read(LinkNodeBase &stream, void *data, unsigned lenght);

class LinkNodeBase : public rclcpp::Node
{
public:
    LinkNodeBase(std::string node_name, int self_addr, int dev_addr)
        : Node(node_name), _stream_manager(self_addr, *this, logger), _stream(_stream_manager, dev_addr), _master(_stream, logger)
    {
        serial_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", 10);
        serial_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "serial_read", 10, std::bind(&LinkNodeBase::readCallback, this, std::placeholders::_1));
        hydrolib_RingQueue_Init(&txQueue_, workBufTxQueue.data(), Len);
    }

public:
    int write(const void *data, unsigned lenght)
    {
        std::vector<uint8_t> packet = createPacket(data, lenght);
        auto serial_msg = std_msgs::msg::UInt8MultiArray();
        serial_msg.data = packet;
        serial_pub_->publish(serial_msg);
        return lenght;  // Fix: added missing return statement
    }

    int read(void *data, unsigned lenght)
    {
        if (uint16_t len = hydrolib_RingQueue_GetLength(&txQueue_); lenght > len)
        {
            hydrolib_RingQueue_Pull(&txQueue_, data, len);
            return len;
        }
        else
        {
            // Fix: cast lenght to uint16_t to match expected parameter type
            hydrolib_RingQueue_Pull(&txQueue_, data, static_cast<uint16_t>(lenght));
            return lenght;
        }
    }

    void serialWrite(const void *data, int Memory_addr, unsigned lenght) { _master.Write(data, Memory_addr, lenght); }
    void serialRead(void *data, int Memory_addr, unsigned lenght) { _master.Read(data, Memory_addr, lenght); }

private:
    std::vector<uint8_t> createPacket(const void *data, unsigned lenght)
    {
        std::vector<uint8_t> packet;
        auto value = static_cast<const uint8_t *>(data);
        // Fix: use unsigned loop variable to avoid signed/unsigned mismatch
        for (unsigned i = 0; i < lenght; i++)
        {
            packet.push_back(value[i]);
        }

        return packet;
    }

    void readCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        if (!msg->data.empty())
        {
            const void *data = msg->data.data();
            unsigned length = static_cast<unsigned>(msg->data.size());
            // Fix: added missing comma and cast length to uint16_t
            hydrolib_RingQueue_Push(&txQueue_, data, static_cast<uint16_t>(length));
            RCLCPP_INFO(this->get_logger(), "Received %zu bytes to serial", static_cast<size_t>(length));
        }
    };

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_pub_;
    hydrolib::bus::datalink::StreamManager<LinkNodeBase, hydrolib::logger::LogDistributor<TestLogStream>> _stream_manager;
    hydrolib::bus::datalink::Stream<LinkNodeBase, hydrolib::logger::LogDistributor<TestLogStream>> _stream;
    hydrolib::bus::application::Master<
        hydrolib::bus::datalink::Stream<LinkNodeBase, hydrolib::logger::LogDistributor<TestLogStream>>,
        hydrolib::logger::LogDistributor<TestLogStream>>
        _master;

    hydrolib_RingQueue txQueue_;
    static constexpr uint16_t Len{UINT8_MAX};
    std::array<uint8_t, Len> workBufTxQueue;
};

int write(LinkNodeBase &stream, const void *data, unsigned lenght) { return stream.write(data, lenght); }
int read(LinkNodeBase &stream, void *data, unsigned lenght) { return stream.read(data, lenght); }

} // namespace stingray_core::baseLink