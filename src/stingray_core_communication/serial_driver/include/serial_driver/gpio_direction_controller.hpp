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

#include <gpiod.hpp> // C++ API для libgpiod v1.x
#include <memory>
#include <string>

namespace drivers
{
namespace serial_driver
{

class GpioDirectionController
{
public:
    explicit GpioDirectionController(int gpio_number = -1, const std::string &chip_name = "gpiochip0");

    ~GpioDirectionController();

    bool is_enabled() const;
    int gpio_number() const;

    void initialize();
    void set_tx();
    void set_rx();
    void delay_us(int microseconds);

private:
    int m_gpio_number;
    std::string m_chip_name;
    bool m_initialized{false};

    std::unique_ptr<gpiod::chip> m_chip;
    // В v1.x работа идет напрямую с объектом линии, а не с line_request
    std::unique_ptr<gpiod::line> m_line;
};

} // namespace serial_driver
} // namespace drivers

#endif // SERIAL_DRIVER__GPIO_DIRECTION_CONTROLLER_HPP_
