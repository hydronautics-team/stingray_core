#include "udp_driver/udp_driver.hpp"

#include <iostream>
#include <string>
#include <memory>

namespace drivers
{
namespace udp_driver
{

UdpDriver::UdpDriver(const IoContext & ctx)
: m_ctx(ctx)
{
}

void UdpDriver::init_sender(const std::string & ip, uint16_t port)
{
  m_sender.reset(new UdpSocket(m_ctx, ip, port));
}

void UdpDriver::init_sender(
  const std::string & remote_ip, uint16_t remote_port,
  const std::string & host_ip, uint16_t host_port)
{
  m_sender.reset(new UdpSocket(m_ctx, remote_ip, remote_port, host_ip, host_port));
}

void UdpDriver::init_receiver(const std::string & ip, uint16_t port)
{
  m_receiver.reset(new UdpSocket(m_ctx, ip, port));
}

std::shared_ptr<UdpSocket> UdpDriver::sender() const
{
  return m_sender;
}

std::shared_ptr<UdpSocket> UdpDriver::receiver() const
{
  return m_receiver;
}

}  // namespace udp_driver
}  // namespace drivers
