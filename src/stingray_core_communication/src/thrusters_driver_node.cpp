#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "hydrolib_log_distributor.hpp"
#include "hydrolib_bus_datalink_stream.hpp"
#include "hydrolib_bus_application_master.hpp"

namespace stingray_core
{

    class TestLogStream
    {
    };

    int write([[maybe_unused]] TestLogStream &stream, const void *dest, unsigned length)
    {
        for (unsigned i = 0; i < length; i++)
        {
            std::cout << (reinterpret_cast<const char *>(dest))[i];
        }
        return length;
    }
    class ThrustersDriverNode;
    int write(ThrustersDriverNode &stream, const void *data, unsigned lenght);
    int read(ThrustersDriverNode &stream, void *data, unsigned lenght);

    constexpr char *_ser = "Serializer";
    constexpr char *_distr = "[%s] [%l] %m\n";

    TestLogStream log_stream;
    hydrolib::logger::LogDistributor<TestLogStream> distributor{_distr, log_stream};
    hydrolib::logger::Logger<hydrolib::logger::LogDistributor<TestLogStream>> logger{_ser, 0, distributor};

    class ThrustersDriverNode : public rclcpp::Node
    {
    public:
        ThrustersDriverNode() : Node("thrusters_driver_node"),
                                _stream_manager(2, *this, logger),
                                _stream(_stream_manager, 1),
                                _master(_stream, logger)
        {
            thrusters_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
                "/thrusters/cmd", 10, std::bind(&ThrustersDriverNode::thrustersCallback, this, std::placeholders::_1));

            serial_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", 10);

            RCLCPP_INFO(this->get_logger(), "Thrusters driver node initialized");
        }

        int write(const void *data, unsigned lenght)
        {

            std::vector<uint8_t> packet = createPacket(data, lenght);
            auto serial_msg = std_msgs::msg::UInt8MultiArray();
            serial_msg.data = packet;
            serial_pub_->publish(serial_msg);
        }

        int read(void *data, unsigned lenght)
        {
            return 0; /* TODO сделать норм метод*/
        }

    private:
        void thrustersCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
        {
            if (!msg->data.empty())
            {
                const void *data = msg->data.data();
                unsigned length = static_cast<unsigned>(msg->data.size());
                _master.Write(data, 0, length);
                RCLCPP_INFO(this->get_logger(), "Sent %zu bytes to serial", length);
            }
        }

        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr thrusters_sub_;

        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_pub_;

        std::vector<uint8_t> createPacket(const void *data, unsigned lenght)
        {
            std::vector<uint8_t> packet;
            auto value = static_cast<const uint8_t *>(data);
            for (int i = 0; i < lenght; i++)
            {
                packet.push_back(value[i]);
            }

            return packet;
        }

        hydrolib::bus::datalink::StreamManager<ThrustersDriverNode,
                                               hydrolib::logger::LogDistributor<TestLogStream>>
            _stream_manager;

        hydrolib::bus::datalink::Stream<ThrustersDriverNode,
                                        hydrolib::logger::LogDistributor<TestLogStream>>
            _stream;

        hydrolib::bus::application::Master<hydrolib::bus::datalink::Stream<ThrustersDriverNode,
                                                                           hydrolib::logger::LogDistributor<TestLogStream>>,
                                           hydrolib::logger::LogDistributor<TestLogStream>>
            _master;

    }; // class ThrustersDriverNode

    int write(ThrustersDriverNode &stream, const void *data, unsigned lenght)
    {
        return stream.write(data, lenght);
    }

    int read(ThrustersDriverNode &stream, void *data, unsigned lenght)
    {
        return stream.read(data, lenght);
    }

} // namespace stingray_core

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stingray_core::ThrustersDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}