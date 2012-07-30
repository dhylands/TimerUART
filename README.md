# TimerUART

TimerUART is an implementation of a UART using a Timer interrupt for each bit
rather than using the hardware UART built into the chip.

To use:

Add the following to your .pde file:

  #include <Timer-UART.h>

Create an instance of TimerUART, passing it the Tx pin number:

  TimerUART  serial(A1);

Call the init method to set the baud rate:

  serial.init(38400);

Use print statments to write data to the serial port:

  serial.println("This is a test");

The baud rates suppported depend on the frequency your AVR is running at. Dividing
the CPU frequency by the baudrate and by 8, should yield a number which will fit in
an 8-bit value.

8000000 / 38400 / 8 = 26

So using 34800 baud on an 8MHz part should be fine.

