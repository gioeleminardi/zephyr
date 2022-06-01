/**
 * @file transmit_request.hpp
 * @author Gioele Minardi \<gioelem3\@gmail.com\>
 * @date 25/04/2022
 * @copyright Copyright (c) 2022 SASA - Space Team. All rights reserved.
 */

#ifndef CSFW_TRANSMIT_REQUEST_HPP
#define CSFW_TRANSMIT_REQUEST_HPP

#include <cstdint>
#include <string>

#include <zephyr/drivers/xbee/common.hpp>

namespace drivers::xbee::detail::request
{
class transmit_request {
    public:
	transmit_request() = default;

	[[nodiscard]] uint16_t get_length() const;
	[[nodiscard]] frame_type get_frame_type() const;
	[[nodiscard]] uint8_t get_frame_id() const;
	void set_frame_id(uint8_t frame_id);
	[[nodiscard]] uint64_t get_dst_address() const;
	void set_dst_address(uint64_t dst_address);
	[[nodiscard]] uint8_t get_options() const;
	void set_options(uint8_t options);
	[[nodiscard]] const std::string &get_payload() const;
	void set_payload(const std::string &payload);
	[[nodiscard]] uint8_t get_checksum() const;
	void calculate_checksum();
	void calculate_length();
	[[nodiscard]] uint8_t get_request_byte(uint16_t pos) const;

    private:
	struct fields_offset {
		static inline constexpr uint8_t FRAME_TYPE = 3;
		static inline constexpr uint8_t FRAME_ID = 4;
		static inline constexpr uint8_t DST_ADDRESS = 5;
		static inline constexpr uint8_t OPTIONS = 13;
		static inline constexpr uint8_t DATA = 14;
	};
	uint16_t length_{};
	const frame_type frame_type_{ frame_type::TRANSMIT_REQUEST_64BIT };
	uint8_t frame_id_{};
	uint64_t dst_address_{};
	uint8_t options_{ 0 };
	std::string payload_{};
	uint8_t checksum_{};
};
} // namespace drivers::xbee::detail::request

#endif // CSFW_TRANSMIT_REQUEST_HPP
