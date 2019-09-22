# Simple-Stopwatch
## Describtion
This school project is codded in Assembly created on K22 MCU. I create a simple stopwatch with ability to . Using the "bit-banging" method on a GPIO port generates the square wave with a certain frequency. However, to create relatively precise timing, a routine to delay the whole system is needed, which means the understanding of the frequency of the MCU and the period of each assembly command is needed. To improve the performance of the function generator, I can use the peripheral clock on the MCU to generator signals with more accurate frequency.
