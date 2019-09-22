/******************************************************************************
* Assembly EE 244 Lab5 Assembly Code
* This code creates a stop watch that changes the counting rates according to
* the switch inputs. There is a button that can pause the counting. Once the button
* is pressed again, the timer restart to count.
*
* Shao-Peng Yang, 03/12/2019
******************************************************************************/
                .syntax unified        /* define Syntax */
                .cpu cortex-m4
                .fpu fpv4-sp-d16

                .globl main            /* make main() global so outside file */
                                       // Lower global Port control register addr for C
                .equ PORTC_GPCLR, 0x4004B080
				// Port control register addr for D
				.equ PORTD_PCR2, 0x4004C008
				.equ PORTD_PCR3, 0x4004C00C
				.equ PORTD_PCR4, 0x4004C010
				.equ PORTD_PCR7, 0x4004C01C
				// Port control register addr for B
				.equ PORTB_PCR0, 0x4004A000
				.equ PORTB_PCR1, 0x4004A004
				.equ PORTB_PCR16, 0x4004A040
				.equ PORTB_PCR18, 0x4004A048
				.equ PORTB_PCR19, 0x4004A04C


				.equ SIM_SCGC5, 0x40048038 // addr for System Clock Gating Control Register 5

				// Addr for PDOR,PDDR,PORI of B,C,D
				.equ GPIOB_PDOR, 0x400FF040
				.equ GPIOB_PDDR, 0x400FF054
				.equ GPIOD_PDOR, 0x400FF0C0
				.equ GPIOD_PDDR, 0x400FF0D4
				.equ GPIOC_PDIR, 0x400FF090
				.equ GPIOC_PDDR, 0x400FF094
				.equ GPIOD_PDIR, 0x400FF0D0

				// Addr for LPT registers
			    .equ LPTMR0_CSR,0x40040000
				// Constatnts for making masks
				.equ BIT0, 1<<0
				.equ BIT_SP, BIT0<<18|BIT0<<19|BIT0<<20|BIT0<<21|BIT0<<22|BIT0<<23|BIT0<<24|BIT0<<25
                .equ ALL_ONES, 0xFFFFFFFF
                // masks for clocks, mux, pullup resister, and PDDR
				.equ CLOCK_MASK, BIT0<<10|BIT0<<11|BIT0<<12|BIT0
				.equ CLOCK_MASK_NEG, ~CLOCK_MASK
				.equ MUX_MASK, BIT0<<8|BIT0<<9|BIT0<<10
				.equ MUX_MASK_NEG, ~MUX_MASK
				.equ MUX_MASK_SECOND , BIT0<<8
				.equ SW_MASK, MUX_MASK|BIT0|BIT0<<1
				.equ SW_MASK_NEG, ~SW_MASK
				.equ SW_MASK_SECOND, BIT0<<1|BIT0|BIT0<<8
				.equ MASK_PDDRB, BIT0|BIT0<<1|BIT0<<16|BIT0<<18|BIT0<<19
				.equ MASK_PDDRB_NEG, ~MASK_PDDRB
				.equ MASK_GC, BIT_SP|MUX_MASK|BIT0|BIT0<<1
				.equ MASK_GC_NEG, ~MASK_GC
				.equ MASK_GC_SECOND, BIT0<<1|BIT0|BIT0<<8|BIT_SP
				.equ MASK_D_DIR, BIT0<<2|BIT0<<3|BIT0<<4
				.equ MASK_D_DIR_N, ~(MASK_D_DIR|BIT0<<7)
				.equ MASK_C_DIR, ~(BIT0<<2|BIT0<<3|BIT0<<4|BIT0<<5|BIT0<<6|BIT0<<7|BIT0<<8|BIT0<<9)

				.equ MASK_CMR_INIT, 0x00000C35
				.equ MASK_CLEAR, 0x00000000
                .equ MASK_CLEAR_BYTE, 0x000000FF
				                 /* can see it. Required for startup   */
                .section .text         /* the following is program code      */
                
main:           bl  K22FRDM_BootClock // initialize the clock @20.9Mhz
                bl  IOShieldInit
                bl  InitLPT
                // initialize the last switchvalue to make sure there is a first time check
                ldr r0, = ALL_ONES
                ldr r1, = LastSW
                str r0, [r1]
                // r7 holds the count for the timer, it is initializzed here
                mov r7, #0
                // check whether the switch value is changed to decide whether to change the clock rate
readata:        bl  SWArrayRead
                ldr r1, = LastSW
                ldr r1, [r1]
                cmp r0, r1
                bne changerate
                //check whether the button is pressed to decide whether to hold the timer
                bl DebNCheck
                cmp r0, #1
                beq holdstate

                // check whether the TCM flag is set to determine whether to increment the counter
nothold:        ldr r2, =LPTMR0_CSR
                ldr r1, [r2]
                tst r1, BIT0<<7
                bne flagset
                b   readata

flagset:        // write a one to TCM flag to reset it for the next count
                ldr r4, = LPTMR0_CSR
                ldr r5, [r4]
                orr r5, BIT0<<7
                str r5, [r4]

                add r7, r7, #1// increment the counter
                // this rolls over the count if needed
                cmp r7, #9
                it gt
                movgt r7, #0
                mov r0,r7 // pass the count by r0 to SevSegDis
                bl  SevSegDis
                b   readata



changerate:    //update last switch
               ldr r1, =LastSW
               str r0, [r1]
               // saturate the switch input value to 15
               usat r0, #4, r0
               lsl r0, #1 // multiplied by two for making it an offset to load proper compare value to store into CMR
               // disable the timer to make change to CMR
               ldr r4, =LPTMR0_CSR
               ldr r2, [r4]
               and r2, ~(BIT0)
               str r2, [r4]
               // load the new compare value from LUT
               // update the CMR with proper count value
               ldr r1, =CountNum
               ldrh r2, [r1, r0]
               ldr r5, [r4, #8]
               and r5, MASK_CLEAR
               str r5, [r4, #8]
               str r2, [r4, #8]
               // enable the timer again
               ldr r2, [r4]
               orr r2, BIT0
               str r2, [r4]
               b readata

holdstate:     //check whether the button is pressed again to exit the state
               // if not, the prgram will be hold in the while loop till the button pressed
               bl DebNCheck
               tst r0, #1
               beq holdstate
               //reset the TCM flag to restart the counter
               ldr r4, = LPTMR0_CSR
               ldr r5, [r4]
               orr r5, BIT0<<7
               str r5, [r4]
               b readata
/**************************************************************************
* BOOL DebNCheck(void) – This subroutine reads the input from the button
* and debounce the input if needed. It will return a bool value if the
* button is pressed(debounced). This includes a switch debouncer.
*
* Params: none
* Returns: bool value to determine whether to pause the clock (returned by r0)
* MCU: K22F
* Shao-Peng Yang, 02/12/2019
**************************************************************************/
DebNCheck:      push {lr}
readbutt:       ldr r0, =GPIOD_PDIR
                ldr r1, [r0]
                //check whether the button is pressed
                //the button is active low, so the program exit the subroutine if the input value is set
                tst r1, BIT0<<7
                itt ne
                movne r0, #0 // clear r0 to return 0 to tell the main code to not hold the timer
                bne exit
                // delay for a few cycle to do the next read
                mov r3, #0x02
delay2:         subs r3, #1
                bne delay2
                // check if the input is two 1's in a row to debounce
                // if not, it goes back to read again to check whether the signal is just noise
                ldr r1, [r0]
                tst r1, BIT0<<7
                bne readbutt
                // the code wait until the button is not pressed(closed), then it will return a 1 to main program to hold the timer
readbutt2:      ldr r1, [r0]
                tst r1, BIT0<<7
                beq readbutt2
                mov r0, #1
exit:           pop {pc}


/**************************************************************************
* void SevSegDis(INT8U) – This subroutine gets the 8 bit integer for the
* count, and it output the corresponding decimal on the display.
*
* Params: 8bit Integer passed by r0
* Returns: none
* MCU: K22F
* Shao-Peng Yang, 03/12/2019
**************************************************************************/
SevSegDis:      push {lr}
                push {r4}

                // loading the proper 7seg code to display
                ldr r1, =SevenSegTable
                ldr r2, [r1, r0]

                ldr r4, =GPIOD_PDOR
				ldr r3,= #0x0
				bfi r3, r2,#2,#3// insert the output bits from the 8 bit integer to the proper bit position
				                // corresponding to the data output register
				ldr r1,= ALL_ONES
				eor r3, r1 // invert the output value to active low since the led is active low
				str r3, [r4]

				ldr r4, =GPIOB_PDOR
				// put the bits ready to be inserted to the far right
				// so we can insert it to the proper position
				lsr r2,#3
				ldr r3,= #0x0
				bfi r3, r2,#0,#2 // insert the outputs for bit 0-1(led d-e)

				lsr r2,#2
				bfi r3, r2,#16, #1 // insert the output for bit 16(led f)

				lsr r2,#1
				bfi r3, r2,#18, #2// insert the output for bit 18-19(led f-dp)

				ldr r1,= ALL_ONES
				eor r3, r1 // invert the output value to active low since the led is active low
				str r3, [r4]
				pop {r4}
                pop {pc}
/**************************************************************************
* INT8U SwArrayRead(void) – This subroutine reads the input from the switch
* and store the value as an 8bits integer into R0.
*
* Params: none
* Returns: an 8 bit integer
* MCU: K22F
* Shao-Peng Yang, 02/12/2019
**************************************************************************/
SWArrayRead:
                push {lr}
                ldr r1, =GPIOC_PDIR
                ldr r1, [r1]

                lsr r1, #2// put inputs to the far right
                ldr r2,=MASK_CLEAR_BYTE
                eor r0, r1, r2// xor the inputs value with 1s, to invert them to active high
                pop {pc}
/**************************************************************************
* void InitLPT(void) – This subroutine initialize the Low Power Timer.
* This initialize the prescaler and enable the clock.
*
* Params: none
* Returns: none
* MCU: K22F
* Shao-Peng Yang, 03/12/2019
**************************************************************************/
InitLPT:        push {lr}
                //disable the timer so I can initialize the timer
                ldr r0, =LPTMR0_CSR
                ldr r1, [r0]
                and r1, ~(BIT0)
                str r1, [r0]
                //this configures the prescale register
                ldr r1, [r0, #4]
                // this selects the "OSCERCLK_UNDIV" clock, and 256 prescale value
                orr r1, BIT0 | BIT0<<1 | BIT0 <<3 | BIT0 <<4 | BIT0 <<5
                str r1, [r0, #4]
                // this initializes the delay to 100msecond
                ldr r1, [r0, #8]
                orr r1, MASK_CMR_INIT
                str r1, [r0, #8]
                //loop for waiting for the glitch filter to recognize change
                mov r3, #32
delay:          subs r3, #1
                bne delay
                // this enables the timer
                ldr r2, [r0]
                mov r1, #1
                orr r1, r2
                str r1,[r0]
                pop {pc}
/**************************************************************************
* void IOShieldInit(void); – This subroutine initializes the ports.
* The sevenseg display, the switch, and the button 5 is configured by the
* subroutine.
*
* Params: none
* Returns: none
* MCU: K22F
* Shao-Peng Yang, 03/12/2019
**************************************************************************/
IOShieldInit:
                // turn on the clock for port B,C,D
                push {lr}
                ldr r0, =SIM_SCGC5
				ldr r1, [r0]
				// using the masks to assign '1' to bit 12-10
				ldr r2, =CLOCK_MASK_NEG
				and r1, r2
				ldr r2, =CLOCK_MASK
				orr r1, r2
                str r1, [r0]
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //set the mux bits to 001(GPIO) for GPIO port D2,D3,D4
                ldr r0, =PORTD_PCR2
				ldr r1, [r0]
				// using the masks to assign '001' to bit 10-8
				ldr r2, =MUX_MASK_NEG
				and r1, r2
				ldr r2, =MUX_MASK_SECOND
				orr r1, r2
                str r1, [r0]
                // The following is doing the same thing for other port
                ldr r0, =PORTD_PCR3
				ldr r1, [r0]
				ldr r2, =MUX_MASK_NEG
				and r1, r2
				ldr r2, =MUX_MASK_SECOND
				orr r1, r2
                str r1, [r0]

                ldr r0, =PORTD_PCR4
				ldr r1, [r0]
				ldr r2, =MUX_MASK_NEG
				and r1, r2
				ldr r2, =MUX_MASK_SECOND
				orr r1, r2
                str r1, [r0]
                // this is the configuration for button 5
                ldr r0, =PORTD_PCR7
				ldr r1, [r0]
				ldr r2, =~(MUX_MASK|BIT0<<1|BIT0)
				and r1, r2
				ldr r2, =BIT0<<8|BIT0<<1|BIT0
				orr r1, r2
                str r1, [r0]
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //set the mux bits to 001(GPIO) for GPIO port B0,B1,B16, B18, B19
                ldr r0, =PORTB_PCR0
				ldr r1, [r0]
				// using the masks to assign '001' to bit 10-8
				ldr r2, =MUX_MASK_NEG
				and r1, r2
				ldr r2, =MUX_MASK_SECOND
				orr r1, r2
                str r1, [r0]
                // The following is doing the same thing for other port
                ldr r0, =PORTB_PCR1
				ldr r1, [r0]
				ldr r2, =MUX_MASK_NEG
				and r1, r2
				ldr r2, =MUX_MASK_SECOND
				orr r1, r2
                str r1, [r0]

                ldr r0, =PORTB_PCR16
				ldr r1, [r0]
				ldr r2, =MUX_MASK_NEG
				and r1, r2
				ldr r2, =MUX_MASK_SECOND
				orr r1, r2
                str r1, [r0]

                ldr r0, =PORTB_PCR18
				ldr r1, [r0]
				ldr r2, =MUX_MASK_NEG
				and r1, r2
				ldr r2, =MUX_MASK_SECOND
				orr r1, r2
                str r1, [r0]

                ldr r0, =PORTB_PCR19
				ldr r1, [r0]
				ldr r2, =MUX_MASK_NEG
				and r1, r2
				ldr r2, =MUX_MASK_SECOND
				orr r1, r2
                str r1, [r0]
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//set the mux bits to 001(GPIO) and pullup resisters bits to '11'(enable) for GPIO Port C9-0(SW9-0) by using global Pin control*/
				ldr r0, =PORTC_GPCLR
				ldr r1, [r0]
				// using the masks to assign '001' to bit 10-8 and '11' to bit 1-0
				ldr r2, =MASK_GC_NEG
				and r1, r2
				ldr r2, =MASK_GC_SECOND
				orr r1, r2
                str r1, [r0]
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//define direction for GPIO of PORT B(output)
                ldr r0, =GPIOB_PDDR
				ldr r1, [r0]
				ldr r2, =MASK_PDDRB_NEG
				and r1, r2
				ldr r2, =MASK_PDDRB
				orr r1, r2
                str r1, [r0]

				//define direction for GPIO of PORT D(output)
                ldr r0, =GPIOD_PDDR
				ldr r1, [r0]
				and r1, MASK_D_DIR_N
				orr r1, MASK_D_DIR
                str r1, [r0]
                //display zero initially on the sevenseg
                ldr r0, =GPIOB_PDOR
                ldr r1, =BIT0<<18|BIT0<<19
                str r1, [r0]

                pop {pc}
                .section .rodata
/* place defined constants here */
SevenSegTable:  .byte 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F // the LUT for the output of 7 segment display

CountNum:       .2byte 0x0C35, 0x186A, 0x249F, 0x30D4,0x3D09, 0x493E, 0x5573, 0x61A8,0x6DDD, 0x7A12, 0x8647, 0x927C,0x9EB1, 0xAAE6, 0xB71B, 0xC350
                //LUT for the compare value to adjust the counting rate
                .section .bss
/* place RAM variables here */
                .comm LastSW, 1 // variable holds the last state of the switches
/*****************************************************************************/
