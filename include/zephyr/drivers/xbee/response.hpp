/**
 * @file response.hpp
 * @author Gioele Minardi \<gioelem3\@gmail.com\>
 * @date 14/04/2022
 * @copyright Copyright (c) 2022 SASA - Space Team. All rights reserved.
 */
#ifndef CSFW_RESPONSE_HPP
#define CSFW_RESPONSE_HPP

#include <memory>

#include <zephyr/drivers/xbee/common.hpp>

namespace drivers::xbee
{
class response {
    public:
	response() = default;

	[[nodiscard]] uint8_t get_msb() const;
	void set_msb(uint8_t msb);
	[[nodiscard]] uint8_t get_lsb() const;
	void set_lsb(uint8_t lsb);
	[[nodiscard]] uint8_t get_api_id() const;
	[[nodiscard]] const uint8_t *get_cmd_data() const;
	void set_cmd_data(uint8_t *cmd_data);
	[[nodiscard]] uint8_t get_checksum() const;
	void set_checksum(uint8_t checksum);
	bool has_error();
	void set_error(error_code error);
	[[nodiscard]] bool is_complete() const;
	[[nodiscard]] uint16_t get_payload_length() const;
	void calculate_checksum();
	bool verify_checksum();
	void push_rx_data(char c);
	void reset();

    protected:
	uint8_t *_cmd_data{};

    private:
	uint16_t _cmd_data_pos{ 0 };
	uint16_t _data_field_pos{ 0 };
	bool _complete{ false };
	bool _escape{ false };
	error_code _error{ error_code::NO_ERROR };
	uint8_t _msb{};
	uint8_t _lsb{};
	uint8_t _api_id{};
	uint8_t _checksum{};
};
} // namespace drivers::xbee
#endif // CSFW_RESPONSE_HPP
