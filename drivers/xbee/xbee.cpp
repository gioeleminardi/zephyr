/**
 * @file xbee.cpp
 * @author Gioele Minardi \<gioelem3\@gmail.com\>
 * @date 14/04/2022
 * @copyright Copyright (c) 2022 SASA - Space Team. All rights reserved.
 */

#include <zephyr/drivers/xbee/xbee.hpp>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/xbee/detail/response/response_16_bit.hpp>

LOG_MODULE_REGISTER(XBEE, LOG_LEVEL_DBG);

namespace drivers::xbee
{

void Xbee::uart_isr(const device *dev, void *user_data)
{
	static int tx_index = 0;
	if (!uart_irq_update(dev)) {
		return;
	}

	uint8_t c;
	auto xbee_inst = static_cast<Xbee *>(user_data);

	if (uart_irq_tx_ready(dev)) {
		auto ret = uart_fifo_fill(dev, &(xbee_inst->_tx_buffer[tx_index++]), 1);
		if (ret <= 0) {
			LOG_ERR("Error on UART TX: %d", ret);
			uart_irq_tx_disable(dev);
		} else if (tx_index > xbee_inst->get_tx_buffer_size()) {
			tx_index = 0;
			uart_irq_tx_disable(dev);
		}
	}

	while (uart_irq_rx_ready(dev)) {
		uart_fifo_read(dev, &c, 1);
		xbee_inst->serial_callback(c);
	}
}

Xbee::Xbee(const device *device) : _device{ device }
{
	_response.set_cmd_data(_cmd_data);
	uart_irq_callback_user_data_set(_device, uart_isr, this);
	uart_irq_rx_enable(_device);

	_tx_buffer[0] = 'b';
	_tx_buffer_size = 1;
	uart_irq_tx_enable(_device);
}

/**
   * @brief Fills the rx_buffer
   *
   * This function fills the rx_buffer and tries to read the frame data length (MSB, LSB)
   * that will use to calculate the end of the response and call k_msgq_put
   *
   * @param c the read character
   */
void Xbee::serial_callback(uint8_t c)
{
	_response.push_rx_data(c);
	if (_response.is_complete()) {
		LOG_DBG("Response is complete. LEN: %d, ERROR: %d", _response.get_payload_length(),
			_response.has_error());

		if (!_response.has_error()) {
			auto rx = get_response();
			if (rx) {
				LOG_DBG("%s", rx->get_string().c_str());
				if (rx_callback_) {
					rx_callback_(std::move(rx));
				}
			}
		}
	}
}

uint8_t Xbee::get_frame_id() const
{
	return _frame_id;
}

void Xbee::set_frame_id(uint8_t frame_id)
{
	_frame_id = frame_id;
}

std::unique_ptr<i_response> Xbee::get_response()
{
	using namespace drivers::xbee::detail::response;
	std::unique_ptr<i_response> response;
	switch (_response.get_api_id()) {
	case 0x81: {
		response = std::make_unique<response_16bit>();
		response->from_response(_response);
		break;
	}
	default: {
		LOG_ERR("Response (0x%02X) not implemented", _response.get_api_id());
		break;
	}
	}
	return response;
}

void Xbee::send_request(const detail::request::transmit_request &tx_req)
{
	uint8_t c;
	int buffer_idx = 0;

	_tx_buffer[buffer_idx++] = tx_req.get_request_byte(frame_field::START_FRAME);

	for (int i = frame_field::START_FRAME + 1; i < tx_req.get_length() + frame_field::PAYLOAD;
	     ++i) {
		c = tx_req.get_request_byte(i);

		if (need_escape(c)) {
			c = escape_char(c);
			_tx_buffer[buffer_idx++] = common::ESCAPE;
		}
		_tx_buffer[buffer_idx++] = c;
	}

	_tx_buffer[buffer_idx] = tx_req.get_checksum();

	_tx_buffer_size = buffer_idx;

	uart_irq_tx_enable(_device);
}

uint16_t Xbee::get_tx_buffer_size() const
{
	return _tx_buffer_size;
}
void Xbee::set_rx_callback(const Xbee::handler_t &rx_callback)
{
	rx_callback_ = rx_callback;
}
} // namespace drivers::xbee