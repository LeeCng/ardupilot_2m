/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  simple hello world sketch
  Andrew Tridgell September 2011
*/

#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup() {
	hal.console->println_P(PSTR("hello world"));
/*
 * hal.uartD->begin(map_baudrate(g.serial0_baud), 512, 128);，  
      
    //gcs[0].init(hal.uartA);  
      
    hal.uartD->set_blocking_writes(false);  

 *
 */

}

void loop()
{
	hal.scheduler->delay(1000);
	hal.console->println("*");

	while (hal.uartD->available())  
	    {  
	         hal.console->println("D-OK");
	         uint8_t data = (uint8_t)hal.uartD->read();  
	         hal.uartD->write(data);  
	    }  

}

AP_HAL_MAIN();
