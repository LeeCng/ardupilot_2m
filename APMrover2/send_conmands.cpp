/*
 * send_conmands.cpp
 *
 *  Created on: 2017-10-10
 *      Author: zp
 */

/*
 *     由steering.cpp 调用，用于与无人船EtherCAT建立通讯
 *     通讯协议采用708定好的协议
 */

#include "Rover.h"

#define Baudrate  9600      // 设定通信波特率
#define ArraySize 100

struct camera_status {
    bool plantform_switch;bool magnification;
} camera_status;

uint8_t EtherCAT_FeedBack[ArraySize] = { 0 };
uint8_t control_feedback[4];
uint8_t camera_feedback[10];
uint32_t head = 0;
uint32_t tail = 1;

//  数据更新标志位，读取后置1，向地面站发送后置0
bool ctrl_FB_update_flag = false;
bool camera_FB_update_flag = false;

// 初始化
void Rover::setup_CWE(void) {

    // 初始状态转台收起，无缩放
    camera_status.magnification = false;
    camera_status.plantform_switch = false;

    if (hal.uartE == NULL)
        return;
    hal.uartE->begin(Baudrate);

}

// 清空数组
void Rover::clean_array(uint8_t array[], uint8_t num) {
    for (int i = 0; i <= num - 1; i++) {
        array[i] = 0;

    }
}

void Rover::send_Ctrl_CMD_to_EtherCAT(AP_HAL::UARTDriver *uart,
        int16_t throttle_sent, int16_t steering_sent) {

    uint8_t command_buffer[10] = { 0 };
    uint8_t CMD_exp[6] = {0};

    throttle_sent = constrain_int16(throttle_sent, -100, 100);
    steering_sent = constrain_int16(steering_sent, -4500, 4500);

    uart->begin(Baudrate);
//    int16_t sum = 0;
//
//    // 设置帧头
//    command_buffer[0] = 0xAA;
//    command_buffer[1] = 0x55;
//
//    // 报文长度
//    command_buffer[2] = 0x0D;

/****************************************

    //添加帧ID
    command_buffer[0] = 0x00;
    command_buffer[1] = 0x01;

// 报文类型   0x02指示当前帧的控制对象为喷泵
    command_buffer[2] = 0x02;

    // 帧编号
    command_buffer[3] = 0x00;

    // 控制命令  当前情况左右舵角倒斗一致
    //  5 ———— 左舵角 0-200 对应 -30° —— +30°
    //  6 ———— 左倒斗 0-200 对应  -5  ——  +5
    //  7 ———— 右舵角 0-200 对应 -30° —— +30
    //  8 ———— 右倒斗 0-200 对应  -5  ——  +5
    command_buffer[4] = steering_sent / 37.5 + 120;
    command_buffer[5] = throttle_sent*0.9 + 90;
    command_buffer[6] = steering_sent / 37.5 + 120;
    command_buffer[7] = throttle_sent*0.9 + 90;

//    command_buffer[7] = camera_H_sent / 5 - 202;
//    command_buffer[8] = camera_V_sent / 5 - 202;

    // 9 10 备用
    command_buffer[8] = 0x00;
    command_buffer[9] = 0x00;

//    // 3-11位的校验和低八位
//    for (int i = 2; i <= 10; i++) {
//
//        sum ^= command_buffer[i];
//    }
//    command_buffer[11] = sum;
//
//    // 报尾
//    command_buffer[12] = 0xFF;
//
//    // 发送
    uart->write(command_buffer, 10);

    ********************/

    CMD_exp[0] =0x02;
    CMD_exp[1] = steering_sent>>8;
    CMD_exp[2] = steering_sent&0xff;
    CMD_exp[3]= (throttle_sent+200)>>8;
    CMD_exp[4] = (throttle_sent+200) &0xff;
    CMD_exp[5] = 0x03;

    uart->write(CMD_exp,6);

}

void Rover::EtherCAT_data_exchange(void) {

    uint8_t camera_data[6];
    // 向EtherCAT 发送视频侦查仪除水平、俯仰速度以外的数据

    if ((RC_Channel_aux::read_ch_radio(3) > 0) ^ camera_status.magnification) {
        clean_array(camera_data, 6);
        if (RC_Channel_aux::read_ch_radio(3) > 0) {
            EtherCAT_camera_send(5, camera_data);
            camera_status.magnification = true;
        } else {
            EtherCAT_camera_send(6, camera_data);
            camera_status.magnification = false;
        }

    }
    if ((RC_Channel_aux::read_ch_radio(5) > 0)
            ^ camera_status.plantform_switch) {
        clean_array(camera_data, 6);
        if (RC_Channel_aux::read_ch_radio(5) > 0) {
            camera_data[0] = 1;
            EtherCAT_camera_send(15, camera_data);
            camera_status.plantform_switch = true;
        } else {
            EtherCAT_camera_send(15, camera_data);
            camera_status.plantform_switch = false;
        }

    }

    // 读取EtherCAT 数据
//    EtherCAT_FeedBack_read();

}

void gcs_data_exchange(void) {

    //  发送状态反馈    在向地面站发送后flag位置0
    if (ctrl_FB_update_flag == true) {

        ctrl_FB_update_flag = false;
    }
    if (camera_FB_update_flag == true) {

        camera_FB_update_flag = false;
    }

    //  读取地面站命令

}

//  读取EtherCAT反馈数据
bool Rover::EtherCAT_FeedBack_read() {

    uint8_t numP = hal.uartE->available();             // 缓存大小
//    uint8_t len;                                        // 帧数据长度（不包括帧头帧尾、校验）
//    uint8_t check = 0;                                  // 校验

    hal.uartE->begin(Baudrate);

    //  从串口缓存中读取数据
    if (head + 20 <= tail) {
        for (int i = 0; i <= numP; i++) {
            EtherCAT_FeedBack[tail % ArraySize] = hal.uartE->read();
            tail++;
        }
    }

    while (head + 20 <= tail) {
        //  0x04 代表舵角与倒斗反馈
        //  读取控制状态反馈
        if (EtherCAT_FeedBack[head % ArraySize] == 0x00
                && EtherCAT_FeedBack[(head + 1) % ArraySize] == 0x04
                && EtherCAT_FeedBack[(head + 10) == 0x00]) {

            for (int j1 = 0; j1 <= 3; j1++) {
                control_feedback[j1] = EtherCAT_FeedBack[(head + 2 + j1)
                        % ArraySize];
                ctrl_FB_update_flag = true;
            }

            head = head + 10;

        }
        // 0x15 代表视频侦查仪 反馈
        // 读取视频侦查仪状态反馈
        if (EtherCAT_FeedBack[head % ArraySize] == 0x00
                && EtherCAT_FeedBack[(head + 1) % ArraySize] == 0x15
                && EtherCAT_FeedBack[(head + 10) == 0x00]) {
            if (EtherCAT_FeedBack[(head + 2) & ArraySize] == 0x01) {

                for (int j2 = 0; j2 <= 6; j2++) {
                    camera_feedback[j2] = EtherCAT_FeedBack[(head + 3 + j2)
                            % ArraySize];
                    camera_FB_update_flag = true;
                }

                head = head + 10;

            }
            if (EtherCAT_FeedBack[(head + 2) & ArraySize] == 0x02) {

                for (int j3 = 0; j3 <= 2; j3++) {
                    camera_feedback[j3 + 7] = EtherCAT_FeedBack[(head + 3 + j3)
                            % ArraySize];
                    camera_FB_update_flag = true;
                }

                head = head + 10;

            }

        }
        head++;
    }

    return false;
}

//  向EtherCAT 发送视频侦查仪除水平、俯仰速度以外的数据
void Rover::EtherCAT_camera_send(uint8_t data_type,
        const uint8_t camera_command[]) {

    uint8_t camera_buffer[10] = { 0 };

    hal.uartE->begin(Baudrate);
//    uint8_t check = 0;
//
//    // 设置帧头
//    camera_buffer[0] = 0xAA;
//    camera_buffer[1] = 0x55;
//
//    // 报文长度
//    camera_buffer[2] = 0x0D;


    //用于can总线输出，两位帧ID
    camera_buffer[0] = 0x00;
    camera_buffer[1] = 0x01;


    // 报文类型    0x03指示当前指令控制视频侦查仪
    camera_buffer[2] = 0x03;

    // 报文标识类型
    camera_buffer[3] = data_type;

    //  报文内容
    for (int i = 0; i <= 5; i++) {
        camera_buffer[4 + i] = camera_command[i];
    }
//
//    //  计算异或校验
//    for (int j = 2; j <= 10; j++) {
//        check ^= camera_buffer[j];
//    }
//
//    camera_buffer[11] = check;
//
//    // 帧尾
//    camera_buffer[12] = 0xFF;

    hal.uartE->write(camera_buffer, 10);
//    hal.uartE->printf("camera_buffer  %d  \n", data_type);

}

