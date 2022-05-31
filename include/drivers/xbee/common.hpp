/**
 * @file common.hpp
 * @author Gioele Minardi \<gioelem3\@gmail.com\>
 * @date 14/04/2022
 * @copyright Copyright (c) 2022 SASA - Space Team. All rights reserved.
 */

#ifndef CSFW_XBEE_COMMON_HPP
#define CSFW_XBEE_COMMON_HPP

#include <cstdint>

namespace drivers::xbee
{
struct common {
	static inline constexpr uint8_t FRAME_DELIMITER = 0x7E;
	static inline constexpr uint8_t ESCAPE = 0x7D;
	static inline constexpr uint8_t XON = 0x11;
	static inline constexpr uint8_t XOFF = 0x13;
	static inline constexpr uint8_t XOR_VALUE = 0x20;
	static inline constexpr uint8_t MAX_PAYLOAD_SIZE = 0x6C; // 108 bytes
	static inline constexpr uint16_t MAX_RAW_SIZE = 256;
};

static inline bool need_escape(uint8_t c)
{
	return (c == common::FRAME_DELIMITER) || (c == common::ESCAPE) || (c == common::XON) ||
	       (c == common::XOFF);
}

static inline uint8_t escape_char(uint8_t c)
{
	return c ^ common::XOR_VALUE;
}

enum class frame_type : uint8_t {
	TRANSMIT_REQUEST_64BIT = 0x00,
	AT_COMMAND = 0x08,
	AT_COMMAND_QUEUE_PARAMETER_VALUE = 0x09,
	ZIGBEE_TRANSMIT_REQUEST = 0x10,
	ZIGBEE_EXPLICIT_TRANSMIT_REQUEST = 0x11,
	REMOTE_COMMAND_REQUEST = 0x17,
	CREATE_SOURCE_ROUTE = 0x21,
	LEGACY_RECEIVE_PACKET = 0x81,
	AT_COMMAND_RESPONSE = 0x88,
	MODEM_STATUS = 0x8A,
	ZIGBEE_TRANSMIT_STATUS = 0x8B,
	ZIGBEE_RECEIVE_PACKET = 0x90,
	ZIGBEE_EXPLICIT_RX_INDICATOR = 0x91,
	ZIGBEE_IO_DATA_SAMPLE_RX_INDICATOR = 0x92,
	XBEE_SENSOR_READ_INDICATOR = 0x94,
	NODE_IDENTIFICATION_INDICATOR = 0x95,
	REMOTE_COMMAND_RESPONSE = 0x97,
	EXTENDED_MODEM_STATUS = 0x98,
	OVER_THE_AIR_FIRMWARE_UPDATE_STATUS = 0xA0,
	ROUTE_RECORD_INDICATOR = 0xA1,
	MANY_TO_ONE_ROUTE_REQUEST_INDICATOR = 0xA3,
	NONE_TYPE = 0xFF
};

struct frame_field {
	static inline constexpr uint8_t START_FRAME = 0;
	static inline constexpr uint8_t MSB = 1;
	static inline constexpr uint8_t LSB = 2;
	static inline constexpr uint8_t PAYLOAD = 3;
};

enum class error_code {
	NO_ERROR = 0,
	CHECKSUM_FAILURE,
	PACKET_EXCEEDS_BYTE_ARRAY_LENGTH,
	UNEXPECTED_START_BYTE
};
} // namespace drivers::xbee
#endif // CSFW_XBEE_COMMON_HPP
