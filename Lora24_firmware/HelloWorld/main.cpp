#include <stdio.h>
#include <stdlib.h>
#include "mbed.h"

int main()

{
    BufferedSerial s( PA_9, PA_10, 115200 );
    // BufferedSerial s(PA_12, PA_13, 115200);
    // DigitalOut p1(PA_9);
    // DigitalOut p2(PA_10);
    // DigitalOut p3(PA_12);
    // DigitalOut p4(PA_14);

    // DigitalOut PIN2(PA_13);
    // DigitalOut PIN3(PB_14);

    char buf[20] = "Hello World\r\n";
    s.set_format(8, BufferedSerial::None, 1);
    while (1)
    {
        // printf("Hello World !\r\n");
        s.write(buf, 13);
        thread_sleep_for(1000);
        // p1 = !p1;
        // // p2 = !p2;
        // p3 = !p3;
        // p4 = !p4;
    }
    return (0);
}