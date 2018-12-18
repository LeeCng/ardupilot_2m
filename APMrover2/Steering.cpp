// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"

/*
 *              uart out start
 *
 */

static AP_HAL::UARTDriver* uarts[] = { hal.uartA, // console
        };
#define NUM_UARTS (sizeof(uarts)/sizeof(uarts[0]))
#define output_freq  400
#define pulse2angle  0.156*3.141519/180
#define delta_time   20  // setservo 执行时间间隔  单位ms
#define pos_range  45*3.14/180   // 舵角范围  ±45°
uint32_t time_count = 0;
uint32_t last_time_count = 0;
uint16_t last_dir = 0;

#define turn_left   2500
#define turn_right  0

// uart

static void setup_uart(AP_HAL::UARTDriver *uart, const char *name) {
    if (uart == NULL) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(9600);
}

static void test_uart(AP_HAL::UARTDriver *uart, const char *name, int16_t yaw,
        int16_t throttle_radio_output) {
    if (uart == NULL) {
        // that UART doesn't exist on this platform
        return;
    }

    yaw = (yaw + 5000) / 10;
    throttle_radio_output = throttle_radio_output + 200;

    uint8_t data[6] = { 0x02, 0xff, 0xff, 0xff, 0xff, 0x03 };

    data[1] = yaw >> 8;
    data[2] = yaw & 0x00ff;

    data[3] = throttle_radio_output >> 8;
    data[4] = throttle_radio_output & 0x00ff;

    uart->write(data, 6);

}

// print output for check state
//static void print_uart(AP_HAL::UARTDriver *uart, const char *name, int16_t yaw, int16_t throttle_radio_output)
//{
//    if (uart == NULL)
//    {
//        // that UART doesn't exist on this platform
//        return;
//    }
//
//        yaw = (yaw + 5000)/10;
//        throttle_radio_output = throttle_radio_output + 200;
//
//        uint8_t data[6]={0x02,0xff,0xff,0xff,0xff,0x03};
//
//        data[1] = yaw >> 8;
//        data[2] = yaw & 0x00ff;
//
//        data[3] = throttle_radio_output >> 8;
//        data[4] = throttle_radio_output & 0x00ff;
//
//        uart->printf("Output by uart %s.at %.3f seconds.\n",name, hal.scheduler->millis()*0.001f);
//        uart->printf("Yaw is %d    %d\n",data[1],data[2]);
//        uart->printf("Throttle is %d    %d\n",data[3],data[4]);
//
//}

/* ***********************
 * uart out end
 *
 *********************************/

/*****************************************
 * Throttle slew limit
 *****************************************/
void Rover::throttle_slew_limit(int16_t last_throttle) {
    // if slew limit rate is set to zero then do not slew limit
    if (g.throttle_slewrate && last_throttle != 0) {
        // limit throttle change by the given percentage per second
        float temp = g.throttle_slewrate * G_Dt * 0.01f
                * fabsf(
                        channel_throttle->radio_max
                                - channel_throttle->radio_min);
        // allow a minimum change of 1 PWM per cycle
        if (temp < 1) {
            temp = 1;
        }
        channel_throttle->radio_out = constrain_int16(
                channel_throttle->radio_out, last_throttle - temp,
                last_throttle + temp);
    }
}

/*
 check for triggering of start of auto mode
 */bool Rover::auto_check_trigger(void) {
    // only applies to AUTO mode
    if (control_mode != AUTO) {
        return true;
    }

    // check for user pressing the auto trigger to off
    if (auto_triggered && g.auto_trigger_pin != -1
            && check_digital_pin(g.auto_trigger_pin) == 1) {
        gcs_send_text_P(MAV_SEVERITY_WARNING, PSTR("AUTO triggered off"));
        auto_triggered = false;
        return false;
    }

    // if already triggered, then return true, so you don't
    // need to hold the switch down
    if (auto_triggered) {
        return true;
    }

    if (g.auto_trigger_pin == -1 && is_zero(g.auto_kickstart)) {
        // no trigger configured - let's go!
        auto_triggered = true;
        return true;
    }

    if (g.auto_trigger_pin != -1
            && check_digital_pin(g.auto_trigger_pin) == 0) {
        gcs_send_text_P(MAV_SEVERITY_WARNING, PSTR("Triggered AUTO with pin"));
        auto_triggered = true;
        return true;
    }

    if (!is_zero(g.auto_kickstart)) {
        float xaccel = ins.get_accel().x;
        if (xaccel >= g.auto_kickstart) {
            gcs_send_text_fmt(PSTR("Triggered AUTO xaccel=%.1f"),
                    (double) xaccel);
            auto_triggered = true;
            return true;
        }
    }

    return false;
}

/*
 work out if we are going to use pivot steering
 */bool Rover::use_pivot_steering(void) {
    if (control_mode >= AUTO && g.skid_steer_out && g.pivot_turn_angle != 0) {
        int16_t bearing_error = wrap_180_cd(
                nav_controller->target_bearing_cd() - ahrs.yaw_sensor) / 100;
        if (abs(bearing_error) > g.pivot_turn_angle) {
            return true;
        }
    }
    return false;
}

/*
 calculate the throtte for auto-throttle modes
 */
void Rover::calc_throttle(float target_speed) {
    // If not autostarting OR we are loitering at a waypoint
    // then set the throttle to minimum
    if (!auto_check_trigger()
            || ((loiter_time > 0) && (control_mode == AUTO))) {
        channel_throttle->servo_out = g.throttle_min.get();
        return;
    }

    float throttle_base = (fabsf(target_speed) / g.speed_cruise)
            * g.throttle_cruise;
    int throttle_target = throttle_base + throttle_nudge;

    /*
     reduce target speed in proportion to turning rate, up to the
     SPEED_TURN_GAIN percentage.
     */
    float steer_rate = fabsf(
            lateral_acceleration / (g.turn_max_g * GRAVITY_MSS));
    steer_rate = constrain_float(steer_rate, 0.0f, 1.0f);

    // use g.speed_turn_gain for a 90 degree turn, and in proportion
    // for other turn angles
    int32_t turn_angle = wrap_180_cd(next_navigation_leg_cd - ahrs.yaw_sensor);
    float speed_turn_ratio = constrain_float(fabsf(turn_angle / 9000.0f), 0, 1);
    float speed_turn_reduction = (100 - g.speed_turn_gain) * speed_turn_ratio
            * 0.01f;

    float reduction = 1.0f - steer_rate * speed_turn_reduction;

    if (control_mode >= AUTO && wp_distance <= g.speed_turn_dist) {
        // in auto-modes we reduce speed when approaching waypoints
        float reduction2 = 1.0f - speed_turn_reduction;
        if (reduction2 < reduction) {
            reduction = reduction2;
        }
    }

    // reduce the target speed by the reduction factor
    target_speed *= reduction;

    groundspeed_error = fabsf(target_speed) - ground_speed;

    throttle = throttle_target
            + (g.pidSpeedThrottle.get_pid(groundspeed_error * 100) / 100);

    // also reduce the throttle by the reduction factor. This gives a
    // much faster response in turns
    throttle *= reduction;

    if (in_reverse) {
        channel_throttle->servo_out = constrain_int16(-throttle,
                -g.throttle_max, -g.throttle_min);
    } else {
        channel_throttle->servo_out = constrain_int16(throttle, g.throttle_min,
                g.throttle_max);
    }

    if (!in_reverse && g.braking_percent != 0
            && groundspeed_error < -g.braking_speederr) {
        // the user has asked to use reverse throttle to brake. Apply
        // it in proportion to the ground speed error, but only when
        // our ground speed error is more than BRAKING_SPEEDERR.
        //
        // We use a linear gain, with 0 gain at a ground speed error
        // of braking_speederr, and 100% gain when groundspeed_error
        // is 2*braking_speederr
        float brake_gain = constrain_float(
                ((-groundspeed_error) - g.braking_speederr)
                        / g.braking_speederr, 0, 1);
        int16_t braking_throttle = g.throttle_max * (g.braking_percent * 0.01f)
                * brake_gain;
        channel_throttle->servo_out = constrain_int16(-braking_throttle,
                -g.throttle_max, -g.throttle_min);

        // temporarily set us in reverse to allow the PWM setting to
        // go negative
        set_reverse(true);
    }

    if (use_pivot_steering()) {
        channel_throttle->servo_out = 0;
    }
}

/*****************************************
 * Calculate desired turn angles (in medium freq loop)
 *****************************************/

void Rover::calc_lateral_acceleration() {
//    hal.uartD->begin(9600);

    switch (control_mode) {
    case AUTO:
        method = nav_controller->update_waypoint(prev_WP, next_WP);
        break;

    case RTL:
    case GUIDED:
    case STEERING:
        method = nav_controller->update_waypoint(current_loc, next_WP);
        break;
    default:
        return;
    }

    // Calculate the required turn of the wheels

    // negative error = left turn
    // positive error = right turn
    lateral_acceleration = nav_controller->lateral_acceleration();

//    if ((hal.scheduler->millis() % 500) <= 10) {
//        hal.uartE->printf("la_acc:  %f  \n", lateral_acceleration);
//    }

    if (use_pivot_steering()) {
        int16_t bearing_error = wrap_180_cd(
                nav_controller->target_bearing_cd() - ahrs.yaw_sensor) / 100;

        if (bearing_error > 0) {
            lateral_acceleration = g.turn_max_g * GRAVITY_MSS;
        } else {
            lateral_acceleration = -g.turn_max_g * GRAVITY_MSS;
        }
    }
}

/*
 calculate steering angle given lateral_acceleration
 */
void Rover::calc_nav_steer() {
    // check to see if the rover is loitering
    if ((loiter_time > 0) && (control_mode == AUTO)) {
        channel_steer->servo_out = 0;
        return;
    }

    // add in obstacle avoidance
    lateral_acceleration += (obstacle.turn_angle / 45.0f) * g.turn_max_g;

    // constrain to max G force
    lateral_acceleration = constrain_float(lateral_acceleration,
            -g.turn_max_g * GRAVITY_MSS, g.turn_max_g * GRAVITY_MSS);

    channel_steer->servo_out = steerController.get_steering_out_lat_accel(
            lateral_acceleration);
}

/*****************************************
 * Set the flight control servos based on the current calculated values
 *****************************************/
void Rover::set_servos(void) {
    static int16_t last_throttle;
    uint32_t pulse;
    float dir = 0;
    double tar_pos = 0;
//    int16_t camera_H_sent;
//    int16_t camera_V_sent;
//    uint8_t camera_sent[6] = { 0 };

    channel_throttle->radio_mid = (channel_throttle->radio_max
            + channel_throttle->radio_min) / 2;
    channel_steer->radio_mid = (channel_steer->radio_max
            + channel_steer->radio_min) / 2;

    // support a separate steering channel
    RC_Channel_aux::set_servo_out(RC_Channel_aux::k_steering,
            channel_steer->pwm_to_angle_dz(0));

    if (control_mode == MANUAL || control_mode == LEARNING) {

        hal.rcout->set_freq(0xff, output_freq);
        // do a direct pass through of radio values
        //  throttle control
        channel_throttle->radio_in = channel_throttle->read();
        channel_steer->radio_in = channel_steer->read();

        /****************************************
         * 计算油门输出，正转时channel_camera_H通道3.3v，反转输出0
         **************************************/

        if (channel_throttle->radio_in >= channel_throttle->radio_mid + 100) {
            channel_throttle->radio_out =
                    (int16_t) (((float) (channel_throttle->radio_in - 100
                            - channel_throttle->radio_mid))
                            / (channel_throttle->radio_max
                                    - channel_throttle->radio_mid - 100) * 1500);
            channel_throttle_dir->radio_out = 2500;

        } else if (channel_throttle->radio_in
                <= channel_throttle->radio_mid - 100) {
            channel_throttle->radio_out =
                    (int16_t) (((float) (channel_throttle->radio_mid - 100
                            - channel_throttle->radio_in))
                            / (channel_throttle->radio_mid - 100
                                    - channel_throttle->radio_min) * 1500);
            channel_throttle_dir->radio_out = 0;
        } else {
            channel_throttle->radio_out = 0;
        }

        /*******************************
         *  计算偏航输出，摇杆推向左，船向左转
         *  ****************************/

        tar_pos = pos_range
                * ((double) (channel_steer->radio_in - channel_steer->radio_mid)
                        / (double) (channel_steer->radio_max
                                - channel_steer->radio_mid));
//        tar_pos = pos_range
//                * ((double) (1800 - channel_steer->radio_mid)
//                        / (double) (channel_steer->radio_max
//                                - channel_steer->radio_mid));

        steer_pos_output(tar_pos);

//        channel_steer->radio_out = channel_steer->read();
//        channel_throttle->radio_out = channel_throttle->read();
        if (failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
            // suppress throttle if in failsafe and manual
            channel_throttle->radio_out = channel_throttle->radio_trim;
        }
    } else if (control_mode != HOLD) {
        hal.rcout->set_freq(0xff, output_freq);

        //  auto rtl  mode etc

//        tar_pos = pos_range * (double) channel_steer->servo_out / 4500;

        if (method) {     // method == 1 ; 汤老师 方法
            tar_pos = ((double) nav_controller->bearing_error_cd())
                    / 5729.57795f;

            if (tar_pos > 0.8 * pos_range) {
                tar_pos = 0.8 * pos_range;
            } else if (tar_pos < -0.8 * pos_range) {
                tar_pos = -0.8 * pos_range;
            }

            channel_steer->servo_out = (int16_t) (4500 * tar_pos / (pos_range));

            steer_pos_output(-tar_pos);
        } else {      // method == 0 原方法
            if (channel_steer->servo_out > 100 && steer_pos < 25 * PI / 180) { // turn right,less than +25 degree
                channel_steer->radio_out = 2000;
                channel_steer_dir->radio_out = turn_right;
            } else if (channel_steer->servo_out < -100
                    && steer_pos > -25 * PI / 180) { // turn left, more than -25 degree
                channel_steer->radio_out = 2000;
                channel_steer_dir->radio_out = turn_left;
            } else {
                channel_steer->radio_out = 0;
            }
        }

//        channel_steer->calc_pwm();
        // calc channel steer radio out
//        channel_steer->radio_out = 2000;
//        //calc steer pwm out according to lateral acceleration
//        if (lateral_acceleration >= 0.5) {
//            channel_steer_dir->radio_out = turn_right
//            ;
//        } else if (lateral_acceleration <= -0.5) {
//            channel_steer_dir->radio_out = turn_left
//            ;
//        } else {
//            channel_steer->radio_out = 0;
//        }

//        if (in_reverse) {
//            channel_throttle->servo_out = constrain_int16(
//                    channel_throttle->servo_out, -g.throttle_max,
//                    -g.throttle_min);
//        } else {
//            channel_throttle->servo_out = constrain_int16(
//                    channel_throttle->servo_out, g.throttle_min.get(),
//                    g.throttle_max.get());
//        }

        if ((failsafe.bits & FAILSAFE_EVENT_THROTTLE)&& control_mode < AUTO) {
            // suppress throttle if in failsafe
            channel_throttle->servo_out = 0;
        }

        // convert 0 to 100% into PWM
        channel_throttle->calc_pwm();

        // limit throttle movement speed
        throttle_slew_limit(last_throttle);
        if (channel_throttle->radio_out > channel_throttle->radio_max) {
            channel_throttle->radio_out = channel_throttle->radio_max;
        } else if (channel_throttle->radio_out < channel_throttle->radio_min) {
            channel_throttle->radio_out = channel_throttle->radio_min;
        }
        channel_throttle->radio_out =
                (int16_t) (((float) (channel_throttle->radio_out
                        - channel_throttle->radio_min))
                        / (channel_throttle->radio_max
                                - channel_throttle->radio_min) * 700);
        channel_throttle_dir->radio_out = 2500;
    } else {
        channel_throttle->radio_out = 0;
        mid_adjust();
    }

    // record last throttle before we apply skid steering
    last_throttle = channel_throttle->radio_out;

    if (g.skid_steer_out) {
        // convert the two radio_out values to skid steering values

        float steering_scaled = channel_steer->norm_output();
        float throttle_scaled = channel_throttle->norm_output();
        float motor1 = throttle_scaled + 0.5f * steering_scaled;
        float motor2 = throttle_scaled - 0.5f * steering_scaled;
        channel_steer->servo_out = 4500 * motor1;
        channel_throttle->servo_out = 100 * motor2;
        channel_steer->calc_pwm();
        channel_throttle->calc_pwm();
    }

#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
    // send values to the PWM timers for output
    // ----------------------------------------
    channel_steer->output();
    channel_throttle->output();
    channel_steer_dir->output();
    channel_throttle_dir->output();
    RC_Channel_aux::output_ch_all();
#endif

    /*********************************
     * 计算舵机位置
     *******************/

    // 计算下一个控制周期中脉冲数
    pulse = delta_time * output_freq / 1000;
    // 转向方向
    if (channel_steer->radio_out
            > 100&& channel_steer_dir->radio_out == turn_left) {
        dir = -1;
    } else if (channel_steer->radio_out
            > 100&& channel_steer_dir->radio_out == turn_right) {
        dir = 1;
    } else {
        dir = 0;
    }

    steer_pos += dir * pulse * pulse2angle;

//    uint8_t message = 0;
//    message = 0xff & ((int8_t) (steer_pos * 100));
//    hal.uartE->write(message);

//    if (hal.scheduler->millis() % 500 <= 10) {
//        hal.uartE->printf("\r\n p:%f tp:%f \r\n", (float)steer_pos, (float)tar_pos);
//    }

}

/****************************
 * 调整舵角中位位置
 */
void Rover::mid_adjust(void) {

    hal.rcout->set_freq(0xff, output_freq / 4);
    if (channel_steer->radio_in >= channel_steer->radio_mid + 100) {
        channel_steer->radio_out = 2000;
        channel_steer_dir->radio_out = turn_right
        ;
        // 附加通道 radio_out = 2500;
    } else {
        if (channel_steer->radio_in <= channel_steer->radio_mid - 100) {
            channel_steer->radio_out = 2000;
            channel_steer_dir->radio_out = turn_left
            ;
            //附加通道 radio_out = 0;
        } else {
            channel_steer->radio_out = 0;
        }
    }

    steer_pos = 0;
    // 对中完成，标志位置1
    mid_adjust_flag = 1;

}

/*************************************
 * 舵偏角控制程序，输入目标舵偏角，计算后更改脉冲输出
 */

void Rover::steer_pos_output(double steer_pos_tar) {

    if (steer_pos_tar > pos_range) {
        steer_pos_tar = pos_range;
    } else if (steer_pos_tar < -pos_range) {
        steer_pos_tar = -pos_range;
    }

    double delta_pos = 0;

    uint32_t time = hal.scheduler->millis();
    delta_pos = steer_pos_tar - steer_pos;
//    hal.uartE->printf("/r/n dp:%f",(float)delta_pos);
    if (delta_pos > 0.025) {
        channel_steer->radio_out = 2000;
        channel_steer_dir->radio_out = turn_right;
    } else if (delta_pos < -0.025) {
        channel_steer->radio_out = 2000;
        channel_steer_dir->radio_out = turn_left;
    } else {
        channel_steer->radio_out = 0;
    }

//    }

}

