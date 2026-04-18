#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "hydrolib_bus_application_master.hpp"
#include "hydrolib_bus_datalink_stream.hpp"
#include "hydrolib_log_distributor.hpp"
#include "hydrolib_ring_queue.h"

namespace stingray_core::PPbaseLink
{
class TestLogStream
{
};

inline int write([[maybe_unused]] TestLogStream &stream, [[maybe_unused]] const void *dest, unsigned length)
{
    return static_cast<int>(length);
}

inline char kSerializer[] = "Serializer";
inline char kLogFormat[] = "[%s] [%l] %m\n";

inline TestLogStream log_stream;
inline hydrolib::logger::LogDistributor<TestLogStream> distributor{kLogFormat, log_stream};
inline hydrolib::logger::Logger<hydrolib::logger::LogDistributor<TestLogStream>> logger{kSerializer, 0, distributor};

class PanelPressureLinkNodeBase;
inline int write(PanelPressureLinkNodeBase &stream, const void *data, unsigned length);
inline int read(PanelPressureLinkNodeBase &stream, void *data, unsigned length);

class PanelPressureLinkNodeBase : public rclcpp::Node
{
public:
    using DistributorType = hydrolib::logger::LogDistributor<TestLogStream>;
    using StreamManagerType = hydrolib::bus::datalink::StreamManager<PanelPressureLinkNodeBase, DistributorType>;
    using StreamType = hydrolib::bus::datalink::Stream<PanelPressureLinkNodeBase, DistributorType>;
    using MasterType = hydrolib::bus::application::Master<StreamType, DistributorType>;

    PanelPressureLinkNodeBase(std::string node_name, int self_addr, int disp_addr, int pressure_addr)
        : Node(std::move(node_name)),
          self_addr_(declareAddressParameter("self_addr", self_addr)),
          disp_addr_(declareDisplayAddressParameter(disp_addr)),
          pressure_addr_(declareAddressParameter("pressure_addr", pressure_addr)),
          stream_manager_(self_addr_, *this, logger),
          disp_stream_(stream_manager_, disp_addr_),
          pressure_stream_(stream_manager_, pressure_addr_),
          disp_master_(disp_stream_, logger),
          pressure_master_(pressure_stream_, logger)
    {
        serial_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", 20);
        serial_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "serial_read", 20, std::bind(&PanelPressureLinkNodeBase::readCallback, this, std::placeholders::_1));
        hydrolib_RingQueue_Init(&tx_queue_, work_buf_tx_queue_.data(), kQueueLen);

        RCLCPP_INFO(
            this->get_logger(),
            "Pressure panel link node initialized (self_addr=%d, disp_addr=%d, pressure_addr=%d)",
            self_addr_, disp_addr_, pressure_addr_);
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
        const auto bytes_to_read = static_cast<uint16_t>(length > len ? len : static_cast<uint16_t>(length));
        hydrolib_RingQueue_Pull(&tx_queue_, data, bytes_to_read);
        return bytes_to_read;
    }

    void DispWrite(const void *data, int memory_addr, unsigned length) { disp_master_.Write(data, memory_addr, length); }
    void DispRead(void *data, int memory_addr, unsigned length) { disp_master_.Read(data, memory_addr, length); }
    void PressureWrite(const void *data, int memory_addr, unsigned length)
    {
        pressure_master_.Write(data, memory_addr, length);
    }
    void PressureRead(void *data, int memory_addr, unsigned length) { pressure_master_.Read(data, memory_addr, length); }

protected:
    void processIncoming()
    {
        stream_manager_.Process();
        (void)disp_master_.Process();
        (void)pressure_master_.Process();
    }

private:
    int declareAddressParameter(const char *name, int default_value)
    {
        return this->declare_parameter<int>(name, default_value);
    }

    int declareDisplayAddressParameter(int default_value)
    {
        const auto legacy_dev_addr = declareAddressParameter("dev_addr", default_value);
        return declareAddressParameter("disp_addr", legacy_dev_addr);
    }

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

        const auto length = static_cast<unsigned>(msg->data.size());
        hydrolib_RingQueue_Push(&tx_queue_, msg->data.data(), static_cast<uint16_t>(length));

        auto previous_queue_length = hydrolib_RingQueue_GetLength(&tx_queue_);
        while (previous_queue_length > 0)
        {
            processIncoming();
            const auto current_queue_length = hydrolib_RingQueue_GetLength(&tx_queue_);
            if (current_queue_length >= previous_queue_length)
            {
                break;
            }
            previous_queue_length = current_queue_length;
        }
    }

    int self_addr_;
    int disp_addr_;
    int pressure_addr_;

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_pub_;

    StreamManagerType stream_manager_;
    StreamType disp_stream_;
    StreamType pressure_stream_;
    MasterType disp_master_;
    MasterType pressure_master_;

    hydrolib_RingQueue tx_queue_;
    static constexpr uint16_t kQueueLen{UINT8_MAX};
    std::array<uint8_t, kQueueLen> work_buf_tx_queue_{};
};

inline int write(PanelPressureLinkNodeBase &stream, const void *data, unsigned length) { return stream.write(data, length); }

inline int read(PanelPressureLinkNodeBase &stream, void *data, unsigned length) { return stream.read(data, length); }

} // namespace stingray_core::PPbaseLink
