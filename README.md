# RTOS
Embedded C code on real time operating system which can handle multiple tasks at once based on the priority of the tasks.
Works on pseudo-parallelism concept that is the threads think they are all running at the same time but Kernel makes sure that each thread is given 1ms before its switched saving its context along with its SP.
Works on ARM TM4C123GH6PM microcontroller
Very efficient to test and learn multi process handling capability
Simple hardware with few leds hooked up on the controller board
