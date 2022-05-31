/**
 * @file i_response.hpp
 * @author Gioele Minardi \<gioelem3\@gmail.com\>
 * @date 15/04/2022
 * @copyright Copyright (c) 2022 SASA - Space Team. All rights reserved.
 */

#ifndef CSFW_I_RESPONSE_HPP
#define CSFW_I_RESPONSE_HPP
#include <string>

#include "drivers/xbee/response.hpp"
namespace drivers::xbee
{
class i_response {
    public:
	virtual ~i_response() = default;

	[[nodiscard]] uint16_t get_length() const
	{
		return _length;
	}

	void set_length(uint16_t length)
	{
		_length = length;
	}

	[[nodiscard]] uint8_t get_frame_type() const
	{
		return _frame_type;
	}

	void set_frame_type(uint8_t frame_type)
	{
		_frame_type = frame_type;
	}

	[[nodiscard]] uint64_t get_source_address() const
	{
		return _source_address;
	}

	void set_source_address(uint64_t source_address)
	{
		_source_address = source_address;
	}

	[[nodiscard]] uint8_t get_rssi() const
	{
		return _rssi;
	}

	void set_rssi(uint8_t rssi)
	{
		_rssi = rssi;
	}

	[[nodiscard]] uint8_t get_options() const
	{
		return _options;
	}

	void set_options(uint8_t options)
	{
		_options = options;
	}

	[[nodiscard]] const uint8_t *get_rf_payload() const
	{
		return _rf_payload;
	}

	virtual void from_response(const drivers::xbee::response &rx) = 0;

	[[nodiscard]] std::string get_string()
	{
		return std::string{ (char *)_rf_payload };
	}

    protected:
	uint8_t _rf_payload[common::MAX_PAYLOAD_SIZE]{};

    private:
	uint16_t _length{};
	uint8_t _frame_type{};
	uint64_t _source_address{};
	uint8_t _rssi{};
	uint8_t _options{};
};
} // namespace drivers::xbee

#endif // CSFW_I_RESPONSE_HPP
