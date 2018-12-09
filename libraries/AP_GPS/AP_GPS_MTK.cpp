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
#include "AP_GPS.h"
#include "AP_GPS_MTK.h"
#include "math.h"

extern const AP_HAL::HAL& hal;

// initialisation blobs to send to the GPS to try to get it into the
// right mode
const prog_char AP_GPS_MTK::_initialisation_blob[] PROGMEM
= MTK_OUTPUT_5HZ SBAS_ON WAAS_ON MTK_NAVTHRES_OFF;

AP_GPS_MTK::AP_GPS_MTK(AP_GPS &_gps, AP_GPS::GPS_State &_state,
        AP_HAL::UARTDriver *_port) :
        AP_GPS_Backend(_gps, _state, _port), _step(0), _payload_counter(0) {
    gps.send_blob_start(state.instance, _initialisation_blob,
            sizeof(_initialisation_blob));
}

/*
 send an initialisation blob to configure the GPS
 */
void AP_GPS_MTK::send_init_blob(uint8_t instance, AP_GPS &gps) {
    gps.send_blob_start(instance, _initialisation_blob,
            sizeof(_initialisation_blob));
}

// Process bytes available from the stream
//
// The stream is assumed to contain only our custom message.  If it
// contains other messages, and those messages contain the preamble bytes,
// it is possible for this code to become de-synchronised.  Without
// buffering the entire message and re-processing it from the top,
// this is unavoidable.
//
// The lack of a standard header length field makes it impossible to skip
// unrecognised messages.
//
bool AP_GPS_MTK::read(void) {

    bool parsed = false;
//    bool IsAMsg = false;
//
    uint8_t numc = port->available();

//    hal.uartE->printf("\r\n numc:%d \r\n",numc);
    _ck_a = 0;

    head = 1;
    tail = 0;

    GPS_data_2_temp_buffer(numc);         //读取数据

    for (uint8_t k = 0; k <= numc; k++) {
        if (head + 84 >= numc) {
            break;
        }

        if (temp_buffer[head] != PREAMBLE1) {        //判断帧头
            head++;
//            if (hal.scheduler->millis() % 500 < 20) {
//                hal.uartE->printf("\r\n  head: %d , tail: %d  \r\n", head,
//                        tail);
//            }

        } else {
            for (int8_t i = 1; i <= 82; i++) {
                _ck_a += temp_buffer[(head + i)];     //计算校验和
            }
//            port->printf("sum_c:%d\n sum_s:%d\n", _ck_a & 0xff,
//                    temp_buffer[(head - 1)]);
            if ((_ck_a = _ck_a & 0xff) == temp_buffer[(head + 83)]) {

                temp_buffer_2_packed();                 // 保存数据
                decodeMsg();                            // 解码
                parsed = true;
            } else {
                head++;
            }
        }

        _ck_a = 0;
    }

    return parsed;
}

//     解析数据包
bool AP_GPS_MTK::decodeMsg() {

    if (_buffer.msg.fix_type == FIX_3D || _buffer.msg.flag == 50) {
        state.status = AP_GPS::GPS_OK_FIX_3D;
    } else if (_buffer.msg.fix_type == FIX_3D || _buffer.msg.flag == 34) {
        state.status = AP_GPS::GPS_OK_FIX_2D;
    } else {
        state.status = AP_GPS::NO_FIX;
    }

//    port->printf("lat:%f\n", _buffer.msg.latitude);

    state.location.lat = _buffer.msg.lat;
    state.location.lng = _buffer.msg.lng;
    state.location.alt = _buffer.msg.alt * 10;

    if (hal.scheduler->millis() % 500 < 20) {
        hal.uartE->printf("sta_lat:%d\n", state.location.lat);
    }

    state.ground_speed = sqrt(
            _buffer.msg.vel_e * _buffer.msg.vel_e
                    + _buffer.msg.vel_n * _buffer.msg.vel_n);
    state.velocity.x = _buffer.msg.vel_n;
    state.velocity.y = _buffer.msg.vel_e;
    state.velocity.z = -_buffer.msg.vel_d;
    state.ground_course_cd = wrap_360_cd(_buffer.msg.ground_course * 100);
    state.num_sats = _buffer.msg.num_satellites;
    state.have_vertical_velocity = false;
    state.last_gps_time_ms = hal.scheduler->millis();
    state.roll = _buffer.msg.roll;
    state.pitch = _buffer.msg.pitch;
    state.yaw = _buffer.msg.yaw;
    state.roll_rate = _buffer.msg.roll_rate;
    state.pitch_rate = _buffer.msg.pitch_rate;
    state.yaw_rate = _buffer.msg.yaw_rate;
    state.acc_x = _buffer.msg.acc_x;
    state.acc_y = _buffer.msg.acc_y;
    state.acc_z = _buffer.msg.acc_z;
    state.RelPx = _buffer.msg.RelPx;
    state.RelPy = _buffer.msg.RelPy;
    state.RelPz = _buffer.msg.RelPz;

    return true;

}

/*
 detect a MTK GPS
 */bool AP_GPS_MTK::_detect(struct MTK_detect_state &state, uint8_t data) {
    switch (state.step) {
    case 1:
        if (PREAMBLE2 == data) {
            state.step++;
            break;
        }
        state.step = 0;
    case 0:
        state.ck_b = state.ck_a = state.payload_counter = 0;
        if (PREAMBLE1 == data)
            state.step++;
        break;
    case 2:
        if (MESSAGE_CLASS == data) {
            state.step++;
            state.ck_b = state.ck_a = data;
        } else {
            state.step = 0;
        }
        break;
    case 3:
        if (MESSAGE_ID == data) {
            state.step++;
            state.ck_b += (state.ck_a += data);
            state.payload_counter = 0;
        } else {
            state.step = 0;
        }
        break;
    case 4:
        state.ck_b += (state.ck_a += data);
        if (++state.payload_counter == sizeof(struct diyd_mtk_msg))
            state.step++;
        break;
    case 5:
        state.step++;
        if (state.ck_a != data) {
            state.step = 0;
        }
        break;
    case 6:
        state.step = 0;
        if (state.ck_b == data) {
            return true;
        }
    }
    return false;
}

//  将GPS数据保存到临时缓存
void AP_GPS_MTK::GPS_data_2_temp_buffer(uint8_t numc) {

    // 每次保存50字节的数据
    for (uint8_t i = 0; i <= numc; i++) {

        temp_buffer[tail] = port->read();
        tail++;
    }
}

// 数据校验成功后将一帧数据保存到数据包
void AP_GPS_MTK::temp_buffer_2_packed(void) {
    for (uint8_t i = 0; i <= 83; i++) {
        _buffer.bytes[i] = temp_buffer[head];
        head++;
    }
}
