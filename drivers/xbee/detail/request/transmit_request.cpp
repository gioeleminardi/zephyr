/**
 * @file transmit_request.cpp
 * @author Gioele Minardi \<gioelem3\@gmail.com\>
 * @date 25/04/2022
 * @copyright Copyright (c) 2022 SASA - Space Team. All rights reserved.
 */

#include <drivers/xbee/detail/request/transmit_request.hpp>

namespace drivers::xbee::detail::request
{

frame_type transmit_request::get_frame_type() const
{
	return frame_type_;
}
uint8_t transmit_request::get_frame_id() const
{
	return frame_id_;
}
void transmit_request::set_frame_id(uint8_t frame_id)
{
	frame_id_ = frame_id;
}
uint64_t transmit_request::get_dst_address() const
{
	return dst_address_;
}
void transmit_request::set_dst_address(uint64_t dst_address)
{
	dst_address_ = dst_address;
}
uint8_t transmit_request::get_options() const
{
	return options_;
}
void transmit_request::set_options(uint8_t options)
{
	options_ = options;
}
uint8_t transmit_request::get_checksum() const
{
	return checksum_;
}
void transmit_request::calculate_checksum()
{
	checksum_ = static_cast<uint8_t>(frame_type_);
	checksum_ += frame_id_;
	for (auto i = 8; i <= 64; i += 8) {
		checksum_ += (dst_address_ >> (64 - i)) & 0xFF;
	}
	checksum_ += options_;
	for (uint32_t i = 0; i < payload_.length(); ++i) {
		checksum_ += payload_.c_str()[i] & 0xFF;
	}
	checksum_ = 0xFF - checksum_;
}

uint16_t transmit_request::get_length() const
{
	return length_;
}

void transmit_request::calculate_length()
{
	length_ = 11;
	length_ += payload_.length();
}

const std::string &transmit_request::get_payload() const
{
	return payload_;
}

void transmit_request::set_payload(const std::string &payload)
{
	payload_ = payload;
}

uint8_t transmit_request::get_request_byte(uint16_t pos) const
{
	uint8_t ret = 0;
	if (pos == frame_field::START_FRAME)
		ret = common::FRAME_DELIMITER;
	if (pos == frame_field::MSB)
		ret = (length_ >> 8) & 0xFF;
	if (pos == frame_field::LSB)
		ret = length_ & 0xFF;

	if (pos >= frame_field::PAYLOAD) {
		// Inside particular frame
		if (pos == fields_offset::FRAME_TYPE)
			ret = static_cast<uint8_t>(frame_type_);
		if (pos == fields_offset::FRAME_ID)
			ret = frame_id_;
		if (pos == fields_offset::DST_ADDRESS)
			ret = (dst_address_ >> 56) & 0xFF;
		if (pos == fields_offset::DST_ADDRESS + 1)
			ret = (dst_address_ >> 48) & 0xFF;
		if (pos == fields_offset::DST_ADDRESS + 2)
			ret = (dst_address_ >> 40) & 0xFF;
		if (pos == fields_offset::DST_ADDRESS + 3)
			ret = (dst_address_ >> 32) & 0xFF;
		if (pos == fields_offset::DST_ADDRESS + 4)
			ret = (dst_address_ >> 24) & 0xFF;
		if (pos == fields_offset::DST_ADDRESS + 5)
			ret = (dst_address_ >> 16) & 0xFF;
		if (pos == fields_offset::DST_ADDRESS + 6)
			ret = (dst_address_ >> 8) & 0xFF;
		if (pos == fields_offset::DST_ADDRESS + 7)
			ret = dst_address_ & 0xFF;
		if (pos == fields_offset::OPTIONS)
			ret = options_;
		if (pos >= fields_offset::DATA)
			ret = payload_.c_str()[pos - fields_offset::DATA] & 0xFF;
	}
	return ret;
}
} // namespace drivers::xbee::detail::request