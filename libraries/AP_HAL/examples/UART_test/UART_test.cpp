/*
 simple test of UART interfaces
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

struct lidar_state {
    int16_t R_D;
    int16_t R_V;
    uint8_t credit;
    int16_t R_D_S;
    int16_t R_V_S;
    uint8_t counter;

} _state_data;

uint16_t numc;

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_HAL::UARTDriver* uarts[] = { hal.uartA, // console
        };
#define NUM_UARTS (sizeof(uarts)/sizeof(uarts[0]))

/*
 setup one UART at 57600
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name) {
    if (uart == NULL) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(115200);
    uart->printf("%s OK", name);
}

void setup(void) {
    /*
     start all UARTs at 57600 with default buffer sizes
     */
    setup_uart(hal.uartA, "uartA"); // console
    setup_uart(hal.uartB, "uartB"); // 1st GPS
    setup_uart(hal.uartC, "uartC"); // telemetry 1
    setup_uart(hal.uartD, "uartD"); // telemetry 2
    setup_uart(hal.uartE, "uartE"); // 2nd GPS
}

static void test_uart(AP_HAL::UARTDriver *uart, const char *name) {
    if (uart == NULL) {
        // that UART doesn't exist on this platform
        return;
    }

//    hal.uartE->printf("curret uart is %s and j is %d",name,j);

    uart->printf("Hello on UART %s at %.3f seconds\n", name,
            hal.scheduler->millis() * 0.001f);

}

static void data_output(AP_HAL::UARTDriver *uart, uint8_t *buffer, uint8_t n) {

    if (n >= 9) {

        _state_data.R_D = buffer[0] | (buffer[1] << 8);
        _state_data.R_V = buffer[2] | (buffer[3] << 8);
        _state_data.credit = buffer[4];
        _state_data.R_D_S = buffer[5] | (buffer[6] << 8);
        _state_data.R_V_S = buffer[7] | (buffer[8] << 8);
        _state_data.counter = buffer[9];

        uart->printf(
                "\n\t credit: %d\t distance: %d\t velocity: %d\t smooth dis: %d\t smooth vel: %d\n",
                _state_data.credit, _state_data.R_D, _state_data.R_V,
                _state_data.R_D_S, _state_data.R_V_S);

    }

}

static void data_input(AP_HAL::UARTDriver *uart_in, const char *name) {
    uint8_t temp_buffer[50];
    uint8_t _payload_counter = 0;
    uint8_t data;
    uint8_t step = 0;
    // struct lidar_state *data = new lidar_state();

    if (uart_in == NULL) {
        // that UART doesn't exist on this platform
        return;
    }

    numc = uart_in->available();

    for (uint8_t i = 0; i <= numc; i++) {

        data = uart_in->read();

        switch (step) {
        case 0:
            if (data == 0x55) {
                step++;
                _payload_counter = 0;
            }
            break;

        case 1:
            if (data == 0xAA) {
                step = 0;
                data_output(hal.uartE, temp_buffer, _payload_counter);
            } else {
                temp_buffer[_payload_counter++] = data;
            }
            break;

        }

    }

}

void loop(void) {
//    test_uart(hal.uartA, "uartA");
//    test_uart(hal.uartB, "uartB");
//    test_uart(hal.uartC, "uartC");
//    test_uart(hal.uartD, "uartD");
//    test_uart1(hal.uartE, "uartE");

    data_input(hal.uartD, "uartD");

    // also do a raw printf() on some platforms, which prints to the
    // debug console
#if HAL_OS_POSIX_IO
    ::printf("Hello on debug console at %.3f seconds\n", hal.scheduler->millis()*0.001f);
#endif

    hal.scheduler->delay(100);
}

AP_HAL_MAIN()
;
