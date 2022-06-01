/**
 * @file xbee.hpp
 * @author Gioele Minardi \<gioelem3\@gmail.com\>
 * @date 14/04/2022
 * @copyright Copyright (c) 2022 SASA - Space Team. All rights reserved.
 */

#ifndef CSFW_XBEE_HPP
#define CSFW_XBEE_HPP

#include <functional>

#include <zephyr/device.h>

#include <zephyr/drivers/xbee/detail/response/i_response.hpp>
#include <zephyr/drivers/xbee/detail/request/transmit_request.hpp>
#include <zephyr/drivers/xbee/response.hpp>

namespace drivers::xbee
{
class Xbee {
	using handler_t = std::function<void(std::unique_ptr<i_response>)>;

    public:
	static void uart_isr(const device *dev, void *user_data);

	explicit Xbee(const device *device);

	[[nodiscard]] uint8_t get_frame_id() const;
	void set_frame_id(uint8_t frame_id);
	std::unique_ptr<i_response> get_response();
	[[nodiscard]] uint16_t get_tx_buffer_size() const;

	void serial_callback(uint8_t c);

	void send_request(const detail::request::transmit_request &tx_req);

	void set_rx_callback(const handler_t &rx_callback);

    private:
	const device *_device;
	uint8_t _frame_id{ 0 };
	response _response{};
	uint8_t _cmd_data[common::MAX_RAW_SIZE];
	uint8_t _tx_buffer[common::MAX_RAW_SIZE];
	uint16_t _tx_buffer_size{};
	handler_t rx_callback_;
};
} // namespace drivers::xbee
#endif // CSFW_XBEE_HPP
