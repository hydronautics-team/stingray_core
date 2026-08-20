/* XAZX */ // Copyright 2026 The Autoware Foundation
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

#include "serial_driver/gpio_direction_controller.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace drivers
{
namespace serial_driver
{

GpioDirectionController::GpioDirectionController(int gpio_number, const std::string &chip_name)
    : m_gpio_number(gpio_number), m_chip_name(chip_name)
{
}

GpioDirectionController::~GpioDirectionController()
{
    // Освобождаем линию, если она была захвачена
    if (m_line && m_line->is_requested())
    {
        m_line->release();
    }
}

bool GpioDirectionController::is_enabled() const { return m_gpio_number >= 0 && m_initialized; }

int GpioDirectionController::gpio_number() const { return m_gpio_number; }

void GpioDirectionController::initialize()
{
    if (m_gpio_number < 0 || m_initialized)
    {
        return;
    }

    try
    {
        // 1. Открываем чип (например, "gpiochip0")
        m_chip = std::make_unique<gpiod::chip>(m_chip_name);

        // 2. Получаем конкретную линию (GPIO 16)
        m_line = std::make_unique<gpiod::line>(m_chip->get_line(m_gpio_number));

        // 3. Запрашиваем линию как ВЫХОД с начальным значением 0 (режим RX)
        // API v1.x: request(consumer_name, request_type, default_value)
        m_line->request({"autoware_serial_driver", gpiod::line_request::DIRECTION_OUTPUT, 0});

        m_initialized = true;
    }
    catch (const std::exception &e)
    {
        throw std::runtime_error("Failed to initialize GPIO " + std::to_string(m_gpio_number) + " on " + m_chip_name + ": " +
                                 e.what());
    }
}

void GpioDirectionController::set_tx()
{
    if (is_enabled())
    {
        // Устанавливаем высокий уровень (1) для разрешения передачи
        m_line->set_value(1);
    }
}

void GpioDirectionController::set_rx()
{
    if (is_enabled())
    {
        // Устанавливаем низкий уровень (0) для разрешения приема
        m_line->set_value(0);
    }
}

void GpioDirectionController::delay_us(int microseconds)
{
    if (microseconds > 0)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
    }
}

} // namespace serial_driver
} // namespace drivers
