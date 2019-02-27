/*
* PackageLicenseDeclared: Apache-2.0
* Copyright (c) 2018 ARM Limited
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
#ifndef _MBED_LORAWAN_DEVCLASS_CLIENT_DEVCLASS_CLIENT
#define _MBED_LORAWAN_DEVCLASS_CLIENT_DEVCLASS_CLIENT

#include "mbed.h"
#include "LoRaWANInterface.h"
#include "ClockSyncControlPackage.h"

#include "mbed_trace.h"
#define TRACE_GROUP "GPSC"


enum LORAWAN_GPS_TIME_CLIENT_STATUS {
    LW_GPS_TIME_CLIENT_STATUS_OK = 0,
    LW_GPS_TIME_CLIENT_STATUS_ERROR = -2001,
    LW_GPS_TIME_CLIENT_STATUS_CLOCK_SYNC_ERROR = -2002,
};

// Parameters for the send structure, this is sent to user application
struct LwGpsTimeClientSendParams {

    LwGpsTimeClientSendParams(uint8_t port = 0, uint8_t *data = 0, size_t length = 0, bool confirmed = false)
        : _port(port), _data(data), _length(length), _confirmed(confirmed) {}

    /**
     * The port to send the message on
     */
    uint8_t _port;

    /**
     * Message buffer, note that this buffer is only valid during the initial callback.
     * If you need the buffer afterwards, make your own copy.
     */
    uint8_t *_data;

    /**
     * Length of the message buffer
     */
    size_t _length;

    /**
     * Whether the message needs to be sent as a confirmed message
     */
    bool _confirmed;
};


class LwGpsTimeClient {

public:

    static const uint32_t GPS_TIME_SYNC_SECONDS = 60;

    LwGpsTimeClient(Callback<void(LwGpsTimeClientSendParams &)> send_fn,
                    Callback<lorawan_time_t()>get_gps_time_cb,
                    Callback<void(lorawan_time_t)>set_gps_time_cb)
        :
        _send_fn(send_fn),
        _get_gps_time_cb(get_gps_time_cb),
        _set_gps_time_cb(set_gps_time_cb),
        _set_time(0) {}

    LORAWAN_GPS_TIME_CLIENT_STATUS initialize()
    {
        _clockSync.activate_clock_sync_package(_get_gps_time_cb,
                                               Callback<void(lorawan_time_t)>(this, &LwGpsTimeClient::setGpsTime));
        return LW_GPS_TIME_CLIENT_STATUS_OK;
    }

    LORAWAN_GPS_TIME_CLIENT_STATUS sendGpsTimeSyncReq()
    {
        clk_sync_response_t *req = _clockSync.request_clock_sync(true);
        if (!req) {
            tr_error("sendGpsTimeSyncReq: request clock sync message failed");
            return LW_GPS_TIME_CLIENT_STATUS_CLOCK_SYNC_ERROR;
        }

        LwGpsTimeClientSendParams send_params(CLOCK_SYNC_PORT, req->data, req->size, false);
        _send_fn(send_params);

        return LW_GPS_TIME_CLIENT_STATUS_OK;
    }

    bool gpsTimeSyncIsNeeded()
    {
        lorawan_time_t curr_gps_time = _get_gps_time_cb();
        lorawan_time_t elapsed_time  = _set_time - curr_gps_time;

        return (curr_gps_time == 0) || (elapsed_time > GPS_TIME_SYNC_SECONDS);
    }

    void handleClockSyncCommand(uint8_t *rx_buffer, uint16_t size)
    {
        clk_sync_response_t *resp =  _clockSync.parse(rx_buffer, size);
        if (resp) {
            LwGpsTimeClientSendParams send_params(CLOCK_SYNC_PORT, resp->data, resp->size, false);
            _send_fn(send_params);
        }
    }

    lorawan_time_t getGpsTime()
    {
        return _get_gps_time_cb();
    }


protected:

    void setGpsTime(lorawan_time_t gps_time)
    {
        _set_time = gps_time;
        _set_gps_time_cb(gps_time);
    }

    ClockSyncControlPackage                     _clockSync;
    Callback<void(LwGpsTimeClientSendParams &)> _send_fn;
    Callback<lorawan_time_t()>                  _get_gps_time_cb;
    Callback<void(lorawan_time_t)>              _set_gps_time_cb;
    lorawan_time_t                              _set_time;
};

#endif


