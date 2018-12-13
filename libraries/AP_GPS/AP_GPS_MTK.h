// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
//  DIYDrones Custom Mediatek GPS driver for ArduPilot and ArduPilotMega.
//	Code by Michael Smith, Jordi Munoz and Jose Julio, DIYDrones.com
//
//	GPS configuration : Custom protocol per "DIYDrones Custom Binary Sentence Specification V1.1"
//
// Note - see AP_GPS_MTK16.h for firmware 1.6 and later.
//
#ifndef __AP_GPS_MTK_H__
#define __AP_GPS_MTK_H__

#include "AP_GPS.h"
#include "AP_GPS_MTK_Common.h"
#include <AP_HAL/AP_HAL.h>

class AP_GPS_MTK : public AP_GPS_Backend {
public:
    AP_GPS_MTK(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    bool read(void);
    bool decodeMsg(void); // checkeSUM 、

    static bool _detect(struct MTK_detect_state &state, uint8_t data);
    static void send_init_blob(uint8_t instance, AP_GPS &gps);

private:
    struct PACKED diyd_mtk_msg {
        uint8_t header;
        float roll_rate;
        float pitch_rate;
        float yaw_rate;
        float acc_x;
        float acc_y;
        float acc_z;
        float pitch;
        float yaw;
        float roll;
        float vel_e;
        float vel_n;
        float vel_d;
        int32_t lng;      // /10^7 得到经度
        int32_t lat;      // /10^7 得到纬度
        int32_t alt;      // mm
        int32_t RelPx;      // mm
        int32_t RelPy;      // mm
        int32_t RelPz;      // mm
        float ground_course;
        uint8_t empty1;
        uint8_t empty2;
        uint8_t empty3;
        uint8_t num_satellites;
        uint8_t flag;    //16-单点定位； 34浮点rtk； 50-整数rtk
        uint8_t fix_type;    //  0-等待卫星， 1-初始对准， 2-组合导航
        uint8_t sum;     //2-87字节和的低八位
    };
    enum diyd_mtk_fix_type {
        FIX_NONE = 0,
        FIX_2D = 1,
        FIX_3D = 2
    };

    enum diyd_mtk_protocol_bytes {
        PREAMBLE1 = 0xAA,
        PREAMBLE2 = 0x62,
        MESSAGE_CLASS = 1,
        MESSAGE_ID = 5
    };

    // Packet checksum accumulators
    uint8_t         _ck_a;
    uint8_t         _ck_b;

    // State machine state
    uint8_t         _step;
    uint8_t         _payload_counter;

    // store buffer
    uint8_t temp_buffer[1024];
    uint32_t head ;
    uint32_t tail ;

    // Receive buffer
    union PACKED {
        diyd_mtk_msg msg;
        uint8_t bytes[84];
    } _buffer;

;

    // Buffer parse & GPS state update
    void        _parse_gps();



    void GPS_data_2_temp_buffer(uint16_t);

    void temp_buffer_2_packed(void);

    static const prog_char _initialisation_blob[];
};

#endif  // __AP_GPS_MTK_H__
