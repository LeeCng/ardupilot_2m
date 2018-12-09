/*
 * send_conmands.cpp
 *
 *  Created on: 2017-10-10
 *      Author: zp
 */

/*
 *     ��steering.cpp ���ã����������˴�EtherCAT����ͨѶ
 *     ͨѶЭ�����708���õ�Э��
 */

#include "Rover.h"

#define Baudrate  9600      // �趨ͨ�Ų�����
#define ArraySize 100

struct camera_status {
    bool plantform_switch;bool magnification;
} camera_status;

uint8_t EtherCAT_FeedBack[ArraySize] = { 0 };
uint8_t control_feedback[4];
uint8_t camera_feedback[10];
uint32_t head = 0;
uint32_t tail = 1;

//  ���ݸ��±�־λ����ȡ����1�������վ���ͺ���0
bool ctrl_FB_update_flag = false;
bool camera_FB_update_flag = false;

// ��ʼ��
void Rover::setup_CWE(void) {

    // ��ʼ״̬ת̨����������
    camera_status.magnification = false;
    camera_status.plantform_switch = false;

    if (hal.uartE == NULL)
        return;
    hal.uartE->begin(Baudrate);

}

// �������
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
//    // ����֡ͷ
//    command_buffer[0] = 0xAA;
//    command_buffer[1] = 0x55;
//
//    // ���ĳ���
//    command_buffer[2] = 0x0D;

/****************************************

    //���֡ID
    command_buffer[0] = 0x00;
    command_buffer[1] = 0x01;

// ��������   0x02ָʾ��ǰ֡�Ŀ��ƶ���Ϊ���
    command_buffer[2] = 0x02;

    // ֡���
    command_buffer[3] = 0x00;

    // ��������  ��ǰ������Ҷ�ǵ���һ��
    //  5 �������� ���� 0-200 ��Ӧ -30�� ���� +30��
    //  6 �������� �󵹶� 0-200 ��Ӧ  -5  ����  +5
    //  7 �������� �Ҷ�� 0-200 ��Ӧ -30�� ���� +30
    //  8 �������� �ҵ��� 0-200 ��Ӧ  -5  ����  +5
    command_buffer[4] = steering_sent / 37.5 + 120;
    command_buffer[5] = throttle_sent*0.9 + 90;
    command_buffer[6] = steering_sent / 37.5 + 120;
    command_buffer[7] = throttle_sent*0.9 + 90;

//    command_buffer[7] = camera_H_sent / 5 - 202;
//    command_buffer[8] = camera_V_sent / 5 - 202;

    // 9 10 ����
    command_buffer[8] = 0x00;
    command_buffer[9] = 0x00;

//    // 3-11λ��У��͵Ͱ�λ
//    for (int i = 2; i <= 10; i++) {
//
//        sum ^= command_buffer[i];
//    }
//    command_buffer[11] = sum;
//
//    // ��β
//    command_buffer[12] = 0xFF;
//
//    // ����
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
    // ��EtherCAT ������Ƶ����ǳ�ˮƽ�������ٶ����������

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

    // ��ȡEtherCAT ����
//    EtherCAT_FeedBack_read();

}

void gcs_data_exchange(void) {

    //  ����״̬����    �������վ���ͺ�flagλ��0
    if (ctrl_FB_update_flag == true) {

        ctrl_FB_update_flag = false;
    }
    if (camera_FB_update_flag == true) {

        camera_FB_update_flag = false;
    }

    //  ��ȡ����վ����

}

//  ��ȡEtherCAT��������
bool Rover::EtherCAT_FeedBack_read() {

    uint8_t numP = hal.uartE->available();             // �����С
//    uint8_t len;                                        // ֡���ݳ��ȣ�������֡ͷ֡β��У�飩
//    uint8_t check = 0;                                  // У��

    hal.uartE->begin(Baudrate);

    //  �Ӵ��ڻ����ж�ȡ����
    if (head + 20 <= tail) {
        for (int i = 0; i <= numP; i++) {
            EtherCAT_FeedBack[tail % ArraySize] = hal.uartE->read();
            tail++;
        }
    }

    while (head + 20 <= tail) {
        //  0x04 �������뵹������
        //  ��ȡ����״̬����
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
        // 0x15 ������Ƶ����� ����
        // ��ȡ��Ƶ�����״̬����
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

//  ��EtherCAT ������Ƶ����ǳ�ˮƽ�������ٶ����������
void Rover::EtherCAT_camera_send(uint8_t data_type,
        const uint8_t camera_command[]) {

    uint8_t camera_buffer[10] = { 0 };

    hal.uartE->begin(Baudrate);
//    uint8_t check = 0;
//
//    // ����֡ͷ
//    camera_buffer[0] = 0xAA;
//    camera_buffer[1] = 0x55;
//
//    // ���ĳ���
//    camera_buffer[2] = 0x0D;


    //����can�����������λ֡ID
    camera_buffer[0] = 0x00;
    camera_buffer[1] = 0x01;


    // ��������    0x03ָʾ��ǰָ�������Ƶ�����
    camera_buffer[2] = 0x03;

    // ���ı�ʶ����
    camera_buffer[3] = data_type;

    //  ��������
    for (int i = 0; i <= 5; i++) {
        camera_buffer[4 + i] = camera_command[i];
    }
//
//    //  �������У��
//    for (int j = 2; j <= 10; j++) {
//        check ^= camera_buffer[j];
//    }
//
//    camera_buffer[11] = check;
//
//    // ֡β
//    camera_buffer[12] = 0xFF;

    hal.uartE->write(camera_buffer, 10);
//    hal.uartE->printf("camera_buffer  %d  \n", data_type);

}

