/**
 * @file response.cpp
 * @author Gioele Minardi \<gioelem3\@gmail.com\>
 * @date 14/04/2022
 * @copyright Copyright (c) 2022 SASA - Space Team. All rights reserved.
 */

#include <drivers/xbee/response.hpp>

#include <logging/log.h>

LOG_MODULE_REGISTER(XBEE_RESPONSE, LOG_LEVEL_DBG);

namespace drivers::xbee
{

void response::reset()
{
	_msb = 0;
	_lsb = 0;
	_api_id = 0xFF;
	_checksum = 0;
	_complete = false;
	_error = error_code::NO_ERROR;
	_cmd_data_pos = 0;
	_data_field_pos = frame_field::START_FRAME;
	_escape = false;
	_complete = false;
}

void response::set_error(error_code error)
{
	_error = error;
}

void response::push_rx_data(char c)
{
	if (c == common::FRAME_DELIMITER) {
		reset();
		return;
	} else if (c == common::ESCAPE) {
		_escape = true;
		return;
	}

	if (_escape) {
		_escape = false;
		c ^= common::XOR_VALUE;
	}

	_data_field_pos += 1;

	if (_data_field_pos == frame_field::MSB) {
		_msb = c;
		return;
	}

	if (_data_field_pos == frame_field::LSB) {
		_lsb = c;
		return;
	}

	if (_data_field_pos == frame_field::PAYLOAD) {
		_api_id = c;
		return;
	}

	if (_cmd_data_pos == get_payload_length() - 1) {
		// CHECKSUM and END OF RESPONSE
		_checksum = c;
		if (!verify_checksum()) {
			_error = error_code::CHECKSUM_FAILURE;
		}
		_complete = true;
		return;
	} else {
		_cmd_data[_cmd_data_pos++] = c;
	}
}

bool response::is_complete() const
{
	return _complete;
}

bool response::has_error()
{
	return _error != error_code::NO_ERROR;
}

[[nodiscard]] uint16_t response::get_payload_length() const
{
	return (_msb << 8) + _lsb;
}

void response::calculate_checksum()
{
	auto payload_length = get_payload_length();
	uint8_t sum{};

	for (auto i = 0; i < payload_length; ++i) {
		sum += _cmd_data[i];
	}

	_checksum = 0xFF - (sum & 0xFF);
}

bool response::verify_checksum()
{
	auto data_length = get_payload_length() - 1;
	uint8_t checksum{ _api_id };
	for (auto i = 0; i < data_length; ++i) {
		checksum += _cmd_data[i];
	}
	checksum += get_checksum();

	return ((checksum & 0xFF) == 0xFF);
}

uint8_t response::get_msb() const
{
	return _msb;
}

void response::set_msb(uint8_t msb)
{
	_msb = msb;
}

uint8_t response::get_lsb() const
{
	return _lsb;
}

void response::set_lsb(uint8_t lsb)
{
	_lsb = lsb;
}

const uint8_t *response::get_cmd_data() const
{
	return _cmd_data;
}

uint8_t response::get_checksum() const
{
	return _checksum;
}

void response::set_checksum(uint8_t checksum)
{
	_checksum = checksum;
}

void response::set_cmd_data(uint8_t *cmd_data)
{
	_cmd_data = cmd_data;
}

uint8_t response::get_api_id() const
{
	return _api_id;
}

} // namespace drivers::xbee