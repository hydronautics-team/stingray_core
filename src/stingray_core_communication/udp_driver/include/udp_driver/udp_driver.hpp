#ifndef UDP_DRIVER__UDP_DRIVER_HPP_
#define UDP_DRIVER__UDP_DRIVER_HPP_

#include <iostream>
#include <memory>
#include <string>

#include "io_context/io_context.hpp"
#include "udp_socket.hpp"

namespace drivers
{
namespace udp_driver
{

class UdpDriver
{
public:
  explicit UdpDriver(const IoContext & ctx);

  void init_sender(const std::string & ip, uint16_t port);
  void init_sender(
    const std::string & remote_ip, uint16_t remote_port,
    const std::string & host_ip, uint16_t host_port);
  void init_receiver(const std::string & ip, uint16_t port);

  std::shared_ptr<UdpSocket> sender() const;
  std::shared_ptr<UdpSocket> receiver() const;

private:
  const IoContext & m_ctx;
  std::shared_ptr<UdpSocket> m_sender;
  std::shared_ptr<UdpSocket> m_receiver;
};

}  // namespace udp_driver
}  // namespace drivers

#endif  // UDP_DRIVER__UDP_DRIVER_HPP_
