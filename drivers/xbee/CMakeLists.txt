zephyr_library()

zephyr_library_sources_ifdef(CONFIG_XBEE_DRIVER
        request.cpp
        response.cpp
        xbee.cpp
        detail/request/transmit_request.cpp
        detail/response/i_response.hpp
        detail/response/response_16_bit.cpp
        )