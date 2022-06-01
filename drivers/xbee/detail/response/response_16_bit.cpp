/**
 * @file response_16_bit.cpp
 * @author Gioele Minardi \<gioelem3\@gmail.com\>
 * @date 15/04/2022
 * @copyright Copyright (c) 2022 SASA - Space Team. All rights reserved.
 */

#include <zephyr/drivers/xbee/detail/response/response_16_bit.hpp>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rx_16bit, LOG_LEVEL_DBG);

namespace drivers::xbee::detail::response
{
void response_16bit::from_response(const drivers::xbee::response &rx)
{
	auto data_ptr = rx.get_cmd_data();
	set_length(rx.get_payload_length());
	set_frame_type(rx.get_api_id());
	uint16_t source_address =
		(data_ptr[fields::SRC_ADDR_LSB] << 8) + data_ptr[fields::SRC_ADDR_LSB];
	set_source_address(source_address);
	set_rssi(data_ptr[fields::RSSI]);
	set_options(data_ptr[fields::OPTIONS]);
	uint16_t i;
	for (i = 0; i < get_length() - 5; ++i) {
		_rf_payload[i] = data_ptr[fields::RF_DATA + i];
	}
	_rf_payload[i] = '\0';
}
} // namespace drivers::xbee::detail::response