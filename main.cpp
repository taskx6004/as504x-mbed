#include "mbed.h"
#include <as5048spi.h>
 
// The sensors connection are attached to pins 5-8
As5048Spi sensor(p5, p6, p7, p8);
Serial pc(USBTX, USBRX); // tx, rx

int main() {
    while(1) {
        // 
        const int* angles = sensor.read_angle();
        int angle = angles[0];
        
        // The read angle returns the value returned over the SPI bus, including parity bit
        pc.printf("Read result: %x\r\n", angle);
        
        if( As5048Spi::parity_check(angle) )
        {
            // Convert range from 0 to 2^14-1 to 0 - 360 degrees
            int degrees = As5048Spi::degrees(angle)/100;
            pc.printf("Parity check succesfull.\r\n");
            pc.printf("Angle: %i degrees\r\n", degrees );
        }
        else
        {
            pc.printf("Parity check failed.\r\n");
        }
            
        wait_ms(500);
    }
}
