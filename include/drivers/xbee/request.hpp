/**
 * @file request.hpp
 * @author Gioele Minardi \<gioelem3\@gmail.com\>
 * @date 14/04/2022
 * @copyright Copyright (c) 2022 SASA - Space Team. All rights reserved.
 */

#ifndef CSFW_REQUEST_HPP
#define CSFW_REQUEST_HPP

#include <cstdint>

namespace drivers::xbee
{
class request {
    public:
	request(uint8_t api_id, uint8_t frame_id, uint8_t *payload, uint8_t payload_length);

	[[nodiscard]] uint8_t get_api_id() const;
	void set_api_id(uint8_t api_id);
	[[nodiscard]] uint8_t get_frame_id() const;
	void set_frame_id(uint8_t frame_id);
	[[nodiscard]] uint8_t *get_payload() const;
	void set_payload(uint8_t *payload);

	virtual uint8_t get_frame_data();

    protected:
	uint8_t *_payload;
	uint16_t _payload_length;

    private:
	uint8_t _api_id;
	uint8_t _frame_id;
};
} // namespace drivers::xbee
#endif // CSFW_REQUEST_HPP
