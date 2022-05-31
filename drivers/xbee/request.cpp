/**
 * @file request.cpp
 * @author Gioele Minardi \<gioelem3\@gmail.com\>
 * @date 14/04/2022
 * @copyright Copyright (c) 2022 SASA - Space Team. All rights reserved.
 */

#include <drivers/xbee/request.hpp>

namespace drivers::xbee
{

uint8_t request::get_api_id() const
{
	return _api_id;
}
void request::set_api_id(uint8_t api_id)
{
	_api_id = api_id;
}
uint8_t request::get_frame_id() const
{
	return _frame_id;
}
void request::set_frame_id(uint8_t frame_id)
{
	_frame_id = frame_id;
}
uint8_t *request::get_payload() const
{
	return _payload;
}
void request::set_payload(uint8_t *payload)
{
	_payload = payload;
}
} // namespace drivers::xbee