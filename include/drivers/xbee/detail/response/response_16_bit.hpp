/**
 * @file response_16_bit.hpp
 * @author Gioele Minardi \<gioelem3\@gmail.com\>
 * @date 15/04/2022
 * @copyright Copyright (c) 2022 SASA - Space Team. All rights reserved.
 */

#ifndef CSFW_RESPONSE_16_BIT_HPP
#define CSFW_RESPONSE_16_BIT_HPP

#include "drivers/xbee/response.hpp"
#include "i_response.hpp"

namespace drivers::xbee::detail::response
{
class response_16bit : public i_response {
    public:
	response_16bit() = default;
	~response_16bit() override = default;
	void from_response(const drivers::xbee::response &rx) override;

    private:
	struct fields {
		static inline constexpr uint8_t SRC_ADDR_MSB = 0;
		static inline constexpr uint8_t SRC_ADDR_LSB = 1;
		static inline constexpr uint8_t RSSI = 2;
		static inline constexpr uint8_t OPTIONS = 3;
		static inline constexpr uint8_t RF_DATA = 4;
	};
};
} // namespace drivers::xbee::detail::response
#endif // CSFW_RESPONSE_16_BIT_HPP
