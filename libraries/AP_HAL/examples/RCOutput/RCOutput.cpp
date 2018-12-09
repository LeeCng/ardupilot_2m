/*
 simple test of RC output interface
 */

#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup(void) {
    hal.console->println("Starting AP_HAL::RCOutput test");
    for (uint8_t i = 0; i < 14; i++) {
        hal.rcout->enable_ch(i);
    }
    hal.rcout->set_freq(0xffff, 400);
}

static uint16_t pwm = 1700;
static int8_t delta = 1;

void loop(void) {
    uint8_t i;

//        pwm += delta;
//        if (delta > 0 && pwm >= i*10+500) {
//            delta = -1;
//            hal.console->printf("reversing\n");
//        } else if (delta < 0 && pwm <= i*20) {
//            delta = 1;
//            hal.console->printf("reversing\n");
//        }

    for (i = 0; i < 8; i++) {
        hal.rcout->write(i, pwm+i*100);
    }
//    hal.scheduler->delay(400);
//    for (i = 0; i < 8; i++) {
//        hal.rcout->write(i, 0);
//    }
//    hal.scheduler->delay(2000);

}

AP_HAL_MAIN()
;
