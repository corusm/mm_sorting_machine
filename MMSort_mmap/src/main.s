@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@@@     main.s
@@@ ---------------------------------------------------------------------------
@@@     target:  Raspberry Pi Zero W
@@@     project: MM-Sorting-Machine
@@@ ---------------------------------------------------------------------------
@@@ This program controls the MM-Sorting-Machine by reading two inputs,
@@@ controlling the motors(, serving the 7-segment display) and interacting
@@@ with the co-processor.
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

/* constants for assembler */
        @ The following are defined in /usr/include/asm-generic/fcntl.h:
        @ Note that the values are specified in octal.
                .equ      O_RDWR,00000002             @ open for read/write
                .equ      O_DSYNC,00010000            @ synchronize virtual memory
                .equ      __O_SYNC,04000000           @ programming changes with
                .equ      O_SYNC,__O_SYNC|O_DSYNC     @ I/O memory
        @ The following are defined in /usr/include/asm-generic/mman-common.h:
                .equ      PROT_READ,0x1               @ page can be read
                .equ      PROT_WRITE,0x2              @ page can be written
                .equ      MAP_SHARED,0x01             @ share changes
        @ The following are defined by me:
        @        .equ      PERIPH,0x3f000000           @ RPi 2 & 3 peripherals
                .equ      PERIPH,0x20000000           @ RPi zero & 1 peripherals

                .equ      GPIO_OFFSET,0x200000        @ GPIO Function Selecet Register (GPFSEL)
                .equ      GPIO_OFFSET_SET,0x1C        @ GPIO Output Set Register (GPSET)
                .equ      GPIO_OFFSET_LEVEL,0x34      @ GPIO Pin Level Register (GPLEV)
                .equ      GPIO_OFFSET_CLEAR,0x28      @ GPIO Pin Output Clear Register (GPCLR)

                .equ      IND_RED,0b001               @ define color red for COLREG
                .equ      POS_RED,66                  @ define the stepcount for red for TURREG
                .equ      COLOR_RED,0xFF0000          @ define color red for LEDs

                .equ      IND_GREEN,0b010             @ define color green for COLREG
                .equ      POS_GREEN,0                 @ define the stepcount for green for TURREG
                .equ      COLOR_GREEN,0x00FF00        @ define color green for LEDs

                .equ      IND_BLUE,0b011              @ define color blue for COLREG
                .equ      POS_BLUE,134                @ define the stepcount for blue for TURREG
                .equ      COLOR_BLUE,0x0000FF         @ define color blue for LEDs

                .equ      IND_BROWN,0b100             @ define color brown for COLREG
                .equ      POS_BROWN,334               @ define the stepcount for brown for TURREG
                .equ      COLOR_BROWN,0xD2691E        @ define color brown for LEDs

                .equ      IND_ORANGE,0b101            @ define color orange for COLREG
                .equ      POS_ORANGE,200              @ define the stepcount for orange for TURREG
                .equ      COLOR_ORANGE,0xFF8C00       @ define color orange for LEDs

                .equ      IND_YELLOW,0b110            @ define color yellow for COLREG
                .equ      POS_YELLOW,266              @ define the stepcount for yellow for TURREG
                .equ      COLOR_YELLOW,0xFFD700       @ define color yellow for LEDs

                .equ      IND_NA,0b000                @ define color NA for COLREG

                .equ      TIMERIR_OFFSET,0xB000       @ start f´of IR and timer
                .equ      TIMERIR_OFFSET_LOAD,0x400   @ Timer Load Register
                .equ      TIMERIR_OFFSET_VALUE,0x404  @ Timer Value Register
                .equ      TIMERIR_OFFSET_DIVIDER,0x41C@ Timer Divider Register

                .equ      TIME_0_01_MS,1250000        @ Time constant for 0.01 ms

                .equ      O_FLAGS,O_RDWR|O_SYNC       @ open file flags
                .equ      PROT_RDWR,PROT_READ|PROT_WRITE
                .equ      NO_PREF,0
                .equ      PAGE_SIZE,4096              @ Raspbian memory page
                .equ      FILE_DESCRP_ARG,0           @ file descriptor
                .equ      DEVICE_ARG,4                @ device address
                .equ      STACK_ARGS,8                @ sp already 8-byte aligned



/* specified Registers */
        @ SLIREG -> save the position of the Outlet-Slide (0...399)
        SLIREG  .req      r4
        @ TMPREG -> save the address of Timer -> access via [] and other registers with offsets (i.e. Load)
        TMPREG  .req      r5
        @ TURREG -> save the step position of the bucket corresponding to the current M&Ms color
        TURREG  .req      r6
        @ WAITREG -> save the time to wait in this register befor calling doWait
        WAITREG .req      r8
        @CNTREG -> save the amount of sorted M&Ms
        CNTREG  .req      r9
        @ GPIOREG -> save the address of GPIO Function Selecet Register (GPFSEL) -> access via [] and other registers with offsets (i.e. Set)
        GPIOREG .req      r10
        @COLREG -> save the color of the current M&M
        COLREG  .req      r11



@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ - START OF DATA SECTION @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .data
        .balign   4

/* paths to gpiomem and mem */
        gpiomem:
                .asciz    "/dev/gpiomem"
        mem:
                .asciz    "/dev/mem"



/* display messages */
        fdMsg:
                .asciz    "File descriptor = %i\n"
        memMsgGpio:
                .asciz    "(GPIO) Using memory at %p\n"
        memMsgTimerIR:
                .asciz    "(Timer + IR) Using memory at %p\n"

        IntroMsg:
                .asciz    "Welcome to the MM-Sorting-Machine!\n"
        CounterMsg:
                .asciz    "current counter: %i\n"
        SegmentMsg:
                .asciz    "digit of the %i. segment: %i\n"

/* addresse for accessing GPIO and Timer registers */
        .balign   4
        gpio_mmap_adr:
                .word     0
        gpio_mmap_fd:
                .word     0
        timerir_mmap_adr:
                .word     0
        timerir_mmap_fd:
                .word     0

@ - END OF DATA SECTION @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@



@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ - START OF TEXT SECTION @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .text

/* "imports" */
        @ externals for making use of std-functions
        .extern printf

        @ externals for RGB LEDs
        .extern WS2812RPi_Init
        .extern WS2812RPi_DeInit
        .extern WS2812RPi_SetBrightness       @ provide (uint8_t brightness);
        .extern WS2812RPi_SetSingle           @ provide (uint8_t pos, uint32_t color);
        .extern WS2812RPi_SetOthersOff        @ provide (uint8_t pos);
        .extern WS2812RPi_AllOff              @ provide (void);
        .extern WS2812RPi_AnimDo              @ provide (uint32_t cntCycles);
        .extern WS2812RPi_Show

        .balign   4
        .global   main
        .type     main, %function



/* addresses */
        .balign   4
        openMode:
                .word     O_FLAGS
        gpio:
                .word     PERIPH+GPIO_OFFSET
        timerIR:
                .word     PERIPH+TIMERIR_OFFSET



/* general functions */
        @ -----------------------------------------------------------------------------
        @ wait the given time - SW timer
        @   param:     WAITREG
        @   return:    none
        @ -----------------------------------------------------------------------------
        doWait:
                subs WAITREG, #1                      @ WAITREG-- -> update cpsr flags
                bne doWait                            @ if not equal -> keep counting down
                bx lr                                 @ return from function


        @ -----------------------------------------------------------------------------
        @ get the color of the current M&M
        @   param:     none
        @   return:    COLREG
        @ -----------------------------------------------------------------------------
        getColor:
        /*  COLOR     colorBit0   colorBit1  colorBit2       GlobalValue
           --------------------------------------------------------------
            Red           1           0          0           IND_RED
            Green         0           1          0           IND_GREEN
            Blue          1           1          0           IND_BLUE
            Brown         0           0          1           IND_BROWN
            Orange        1           0          1           IND_ORANGE
            Yellow        0           1          1           IND_YELLOW
            -------------------------------------------------------------
            NA            0           0          0           IND_NA
        */
                ldr r0,[GPIOREG, #GPIO_OFFSET_LEVEL]  @ read GPIO Pin Level Register (GPLEV)
                and r0, r0, #0x01C00000               @ AND operation to extract only Bits 22 (colorbit 0), 23 (colorbit 1), 24 (colorbit 2)
                lsr r0, r0, #22                       @ shift to get colorBits directly -> format: xxx

                cmp r0, #0b111                        @ (r1 == undefinedColor) ?
                beq colorUndefinded                   @ r1 == undefinedColor -> set color to IND_NA
                b setColor
        colorUndefinded:
                mov r0, #0b000                        @ r1 = IND_NA
        setColor:
                mov COLREG, r0                        @ set color in COLREG

                teq COLREG, #IND_RED
                beq posRed
                teq COLREG, #IND_GREEN
                beq posGreen
                teq COLREG, #IND_BLUE
                beq posBlue
                teq COLREG, #IND_BROWN
                beq posBrown
                teq COLREG, #IND_ORANGE
                beq posOrange
                teq COLREG, #IND_YELLOW
                beq posYellow
        posRed:
                ldr TURREG, =POS_RED
                b ColorExit
        posGreen:
                ldr TURREG, =POS_GREEN
                b ColorExit
        posBlue:
                ldr TURREG, =POS_BLUE
                b ColorExit
        posBrown:
                ldr TURREG, =POS_BROWN
                b ColorExit
        posOrange:
                ldr TURREG, =POS_ORANGE
                b ColorExit
        posYellow:
                ldr TURREG, =POS_YELLOW
                b ColorExit

        ColorExit:
                bx lr                                 @ return from function


        @ -----------------------------------------------------------------------------
        @ move the Outlet the specified number of steps and update SLIREG
        @   param:     r0 -> steps to move
        @   return:    none
        @ -----------------------------------------------------------------------------
        moveOutlet:
                push {r1, r2, lr}                     @ for (int i = 0; i < steps; i++)
                mov r1, #0                            @ r0 = steps; r1 = i
                mov r2, #0x00001000                   @ set Outlet-StepOut Bit to 1

        /* implement for-loop */
        moveOutletLoop:
                cmp r1, r0                            @ (i < steps) ?
                bge moveOutletLoopExit                @ i >= steps -> branch to exit
                str r2, [GPIOREG, #GPIO_OFFSET_SET]   @ write to GPIO Output Set Register (GPSET) -> set Outlet-StepOut to 1 for next rising edge
                ldr WAITREG, =TIME_0_01_MS
                bl doWait                             @ wait for ~ 0.01ms
                str r2, [GPIOREG, #GPIO_OFFSET_CLEAR] @ write to GPIO Output Clear Register (GPCLR) -> set Outlet-StepOut to 0 for next falling edge
                ldr WAITREG, =TIME_0_01_MS
                bl doWait                             @ wait for ~ 0.01ms
                add r1, r1, #1                        @ i++
                b moveOutletLoop                      @ loop

        moveOutletLoopExit:
                add SLIREG, SLIREG, r0                @ add turned steps (SLIREG)
                cmp SLIREG, #400                      @ (SLIREG < 400) ?
                subge SLIREG, SLIREG, #400            @ SLIREG >= 400 -> full rotation -> start from 0 (modulo)
                pop {r1, r2, lr}
                bx lr                                 @ return from function
                

        @ -----------------------------------------------------------------------------
        @ only set led of bucket corresponding to the current M&Ms color
        @   param:     COLREG -> index of color
        @   return:    none
        @ -----------------------------------------------------------------------------
        setLed:
                push {lr}

                bl WS2812RPi_AllOff                   @ set all LEDs off

                mov r0, COLREG                        @ switch to current color LED on

                cmp r0, #IND_RED
                ldreq r1, =COLOR_RED
                bleq WS2812RPi_SetSingle

                cmp r0, #IND_GREEN
                ldreq r1, =COLOR_GREEN
                bleq WS2812RPi_SetSingle

                cmp r0, #IND_BLUE
                ldreq r1, =COLOR_BLUE
                bleq WS2812RPi_SetSingle

                cmp r0, #IND_BROWN
                ldreq r1, =COLOR_BROWN
                bleq WS2812RPi_SetSingle

                cmp r0, #IND_ORANGE
                ldreq r1, =COLOR_ORANGE
                bleq WS2812RPi_SetSingle

                cmp r0, #IND_YELLOW
                ldreq r1, =COLOR_YELLOW
                bleq WS2812RPi_SetSingle

                bl WS2812RPi_Show                     @ update the leds
                pop {lr}
                bx lr                                 @ return from function


        @ -----------------------------------------------------------------------------
        @ move the slide to the desired position
        @   param:     TURREG -> desired position
        @   return:    None
        @ -----------------------------------------------------------------------------
        moveSlide:
                push {lr}
                cmp SLIREG, TURREG
                beq moveSlideExit                     @ slide at wanted position -> no need to change
                bgt moveSlideBack                     @ slide must move back to get to color_position (do "full" rotation)
                blt moveSlideForward                  @ slide must move forward to get to color_position

        moveSlideBack:
                add TURREG, #400                      @ add "full" rotation
        moveSlideForward:
                sub r0, TURREG, SLIREG                @ r0 = (wanted_pos - current_position) => steps to go
                bl moveOutlet                         @ move Outlet r0 Steps Forward

        moveSlideExit:
                pop {lr}
                bx lr                                 @ return from function


        @ -----------------------------------------------------------------------------
        @ move the colorwheel the specified number of steps
        @   param:     r0 -> steps to move
        @   return:    None
        @ -----------------------------------------------------------------------------
        moveColorwheel:
                push {r1, r2, lr}                     @ for (int i = 0; i < steps; i++)
                mov r1, #0                            @ r1 = i
                mov r2, #0x00002000                   @ set Colorwheel-StepCW Bit to 1

        /* implement for-loop */
        moveColorwheelLoop:
                cmp r1, r0                            @ (i < steps) ?
                bge moveColorwheelLoopExit            @ i >= steps -> branch to test
                str r2, [GPIOREG, #GPIO_OFFSET_SET]   @ write to GPIO Output Set Register (GPSET) -> set Colorwheel-StepCW to 1 for next rising edge
                ldr WAITREG, =TIME_0_01_MS
                bl doWait                             @ wait for ~ 0.01ms
                str r2, [GPIOREG, #GPIO_OFFSET_CLEAR] @ write to GPIO Output Clear Register (GPCLR) -> set Colorwheel-StepCW to 0 for next falling edge
                ldr WAITREG, =TIME_0_01_MS
                bl doWait                             @ wait for ~ 0.01ms
                add r1, r1, #1                        @ i++
                b moveColorwheelLoop                  @ loop

        moveColorwheelLoopExit:
                pop {r1, r2, lr}
                bx lr                                 @ return from function


        @ -----------------------------------------------------------------------------
        @ return pattern for number
        @   param:     r0 -> number to convert
        @   return:    r0 -> pattern of number
        @ -----------------------------------------------------------------------------
        getPattern:
                mov r1, #0b0                          @ if no number from 0-9 -> print nothing

                /* set pattern corresponding to number */
                cmp r0, #0
                moveq r1, #0b11111100
                cmp r0, #1
                moveq r1, #0b01100000
                cmp r0, #2
                moveq r1, #0b11011010
                cmp r0, #3
                moveq r1, #0b11110010
                cmp r0, #4
                moveq r1, #0b01100110
                cmp r0, #5
                moveq r1, #0b10110110
                cmp r0, #6
                moveq r1, #0b10111110
                cmp r0, #7
                moveq r1, #0b11100000
                cmp r0, #8
                moveq r1, #0b11111110
                cmp r0, #9
                moveq r1, #0b11110110

                mov r0, r1
                bx lr                                 @ return from function

        @ -----------------------------------------------------------------------------
        @ print pattern to register
        @   param:     r0 -> pattern to write
        @   return:    None
        @ -----------------------------------------------------------------------------
        printPattern:
                push {r1, r2, lr}
                mov r1, #0x10
                str r1, [GPIOREG, #GPIO_OFFSET_SET]   @ start writing to register
                mov r1, #8
        printPatternLoop:
                cmp r1, #0
                beq printPatternExit                  @ after 8 Bits -> end loop
                lsrs r0, r0, #1                       @ shift Bit to right and load shifted bit into carry Bit
                blcs shiftHigh                        @ if carry Bit is set -> shift a 1 into the register
                blcc shiftLow                         @ if not set -> shift a 0 into the register
                sub r1, r1, #1                        @ i--
                b printPatternLoop                    @ and repeat (loop)
        printPatternExit:
                mov r2, #0x20
                str r2, [GPIOREG, #GPIO_OFFSET_SET]   @ set RCLK to return full register
                ldr WAITREG, =TIME_0_01_MS
                bl doWait
                str r2, [GPIOREG, #GPIO_OFFSET_CLEAR] @ reset RCLK
                ldr WAITREG, =TIME_0_01_MS
                bl doWait
                mov r2, #0x10   
                str r2, [GPIOREG, #GPIO_OFFSET_CLEAR] @ reset register
                pop {r1, r2, lr}
                bx lr                                 @ return from function
        shiftHigh:
                push {r1, r2, lr}
                mov r1, #0x4
                str r1, [GPIOREG, #GPIO_OFFSET_SET]   @ set Bit to 1 for register
                mov r2, #0x8
                str r2, [GPIOREG, #GPIO_OFFSET_SET]   @ set rising edge for clk
                ldr WAITREG, =TIME_0_01_MS
                bl doWait
                str r2, [GPIOREG, #GPIO_OFFSET_CLEAR] @ set falling edge for clk
                ldr WAITREG, =TIME_0_01_MS
                bl doWait
                str r1, [GPIOREG, #GPIO_OFFSET_CLEAR] @ clear Bit for register
                pop {r1, r2, lr}
                bx lr                                 @ return from function
        shiftLow:
                push {r1, r2, lr}
                mov r1, #0x4
                str r1, [GPIOREG, #GPIO_OFFSET_CLEAR] @ set Bit to 0 for register
                mov r2, #0x8
                str r2, [GPIOREG, #GPIO_OFFSET_SET]   @ set rising edge for clk
                ldr WAITREG, =TIME_0_01_MS
                bl doWait
                str r2, [GPIOREG, #GPIO_OFFSET_CLEAR] @ set falling edge for clk
                ldr WAITREG, =TIME_0_01_MS
                bl doWait
                pop {r1, r2, lr}
                bx lr                                 @ return from function

        @ -----------------------------------------------------------------------------
        @ get the number of the given digit
        @   param:     r0 -> current digit, r1 -> next higher digit, CNTREG -> counter
        @   return:    r0 -> number at digit
        @ -----------------------------------------------------------------------------
        getDigit:
                push {r1, r2}
                mov r2, r1                            @ higher digit in r2
                mov r1, r0                            @ current digit in r1
                mov r0, CNTREG                        @ count number in r0

        digitLoop:
                cmp r0, r2                            @ compare number to next higher digit
                subge r0, r0, r2                      @ subtract higher digit until number lower than next higher digit
                bge digitLoop

                mov r2, #0                            @ r2 -> counter of number at digit
        digitCountLoop:
                cmp r0, r1                            @ compare number to current digit
                subge r0, r0, r1                      @ if number >= current digit -> number = number - current digit
                addge r2, r2, #1                      @ AND counter++
                bge digitCountLoop

                mov r0, r2                            @ store counter to r0
                pop {r1, r2}
                bx lr                                 @ return from function with number at digit

        @ -----------------------------------------------------------------------------
        @ display the number of CNTREG
        @   param:     CNTREG
        @   return:    None
        @ -----------------------------------------------------------------------------
        showCounter:
                push {r0, r1, r2, r3, lr}
                ldr r0, =CounterMsg
                mov r1, CNTREG
                bl  printf                            @ print counter to console

                /* print ones */
                ldr r0, =1                            @ current digit = 1 -> ones
                ldr r1, =10                           @ next higher digit = 10 -> tens
                bl getDigit                           @ r0 -> count of ones

                push {r0, r1}
                mov r2, r0
                ldr r0, =SegmentMsg
                mov r1, #4
                bl  printf                            @ print the count and the selected segment to console
                pop {r0, r1}

                mov r1, #0xC0                         @ if next higher digit = 10 -> first Seg from right
                str r1, [GPIOREG, #GPIO_OFFSET_SET]   @ set chanel for given Seg

                bl getPattern                         @ get pattern of number to display
                bl printPattern                       @ print returned pattern to given Seg
        /* Abbruch -> nur Einerstelle wird angezeigt */
        CounterExit:
                pop {r0, r1, r2, r3, lr}
                bx lr                                 @ return from function



/* init functions */
        @ -----------------------------------------------------------------------------
        @ initialize GPIO with output functionality and deactivate low active Pins
        @   param:     none
        @   return:    none
        @ -----------------------------------------------------------------------------
        initGPIOs:
        /*  PIN     GPIO-MODE   Bauteil                 Low Acitve Signal
        ------------------------------------------------------------------
        2 - 7       OUTPUT      7-Segment
        11          OUTPUT      Outlet nRST             x
        12          OUTPUT      Outlet Step
        13          OUTPUT      Colour Wheel Step
        16          OUTPUT      Color Wheel Direction
        17          OUTPUT      Color Wheel nRST        x
        19          OUTPUT      Feeder
        26          OUTPUT      Outlet Direction
        27          OUTPUT      Co-Processor nSLP       x
        */

                push {lr}

        /* GPIO Set 1 (0-9) */
                @         P7  P6  P5  P4  P3  P2  P1  P0
                @ Value: 001 001 001 001 001 001 000 000 = 0x00249240
                ldr r1, =0x00249240
                str r1, [GPIOREG]                     @ write to GPIO Function Select Register (GPFSEL)

        /* GPIO Set 2 (10-19) */
                @        P19 P18 P17 P16 P15 P14 P13 P12 P11 P10
                @ Value: 001 000 001 001 000 000 001 001 001 000 = 0x08240248
                ldr r1, =0x08240248
                str r1, [GPIOREG, #4]                 @ write to GPIO Function Select Register (GPFSEL)


        /* GPIO Set 3 (20-29) */
                @        P27 P26 P25 P24 P23 P22 P21 P20
                @ Value: 001 001 000 000 000 000 000 000 = 0x00240000
                ldr r1, =0x00240000
                str r1, [GPIOREG, #8]                 @ write to GPIO Function Select Register (GPFSEL)

        /* deactivate low active Pins */
                @        25-30 20-24 15-19 10-14  5-9   0-4
                @ Value: 00100 00000 00100 00010 00000 00000 = 0x08020800
                ldr r1, =0x08000000                   @ set Co-Processor nSLP Bit to 1 -> so it wakes up
                orr r1, #0x00020000                   @ set Colorwheel nRST Bit to 1 -> remove RESET instruction
                orr r1, #0x00000800                   @ set Outlet nRST Bit to 1 -> remove RESET instruction
                str r1, [GPIOREG, #GPIO_OFFSET_SET]   @ write to GPIO Output Set Register (GPSET)

                pop {lr}
                bx lr                                 @ return from function


        @ -----------------------------------------------------------------------------
        @ initialize Outlet to position the Slide to startpoint
        @   param:     none
        @   return:    none
        @ -----------------------------------------------------------------------------
        initOutlet:
                push {r0, r1, lr}
                mov r1, #0x4000000
                str r1, [GPIOREG, #GPIO_OFFSET_CLEAR] @ clear Outlet-Dir-Pin to do clockwise rotation

                /* check if Outlet starts in hallsensor -> move outside to calibrate correctly */
                ldr r1, [GPIOREG, #GPIO_OFFSET_LEVEL] @ read GPIO Pin Level Register (GPLEV)
                tst r1, #0x00200000                   @ test if Hallsensor level==1 --> not detected
                moveq r0, #200
                bleq moveOutlet                       @ move Outlet a half rotation to get outside the hallsensor range

                /* continue with initialization */
                mov r0, #1                            @ set stepsize to 1 for moveOutlet-call
                mov r2, #0                            @ set counter to 0 for Hallsensor-steps

        /* while Outlet-Hallsensor is not detected */
        initOutletLoopWhileNotDetected:
                ldr r1, [GPIOREG, #GPIO_OFFSET_LEVEL] @ read GPIO Pin Level Register (GPLEV)
                tst r1, #0x00200000                   @ test if Hallsensor level==1 --> not detected
                beq initOutletLoopWhileDetected       @ if detected -> move to next loop
                bl moveOutlet                         @ else -> move Slide
                b initOutletLoopWhileNotDetected      @ and repeat (loop)

        /* while Outlet-Hallsensor is detected */
        initOutletLoopWhileDetected:                  @ (turn until on edge of detection)
                add r2, r2, #1
                ldr r1, [GPIOREG, #GPIO_OFFSET_LEVEL] @ read GPIO Pin Level Register (GPLEV)
                tst r1, #0x00200000                   @ test if Hallsensor level==1 --> not detected
                bne initOutletExit                    @ if not detected -> move to exit
                bl moveOutlet                         @ else -> move Slide
                b initOutletLoopWhileDetected         @ and repeat (loop)

        initOutletExit:
                mov r1, #0x4000000
                str r1, [GPIOREG, #GPIO_OFFSET_SET]   @ set Outlet-Dir-Pin to do counter-clockwise rotation
                lsr r0, r2, #1
                bl moveOutlet                         @ move Outlet backwards half the counted steps from detecting the hall sensor
                str r1, [GPIOREG, #GPIO_OFFSET_CLEAR] @ clear Outlet-Dir-Pin to do clockwise rotation
                mov SLIREG, #0                        @ set position to 0
                pop {r0, r1, lr}
                bx lr                                 @ return from function


        @ -----------------------------------------------------------------------------
        @ initialize Colorwheel to position free pocket for M&M
        @   param:     none
        @   return:    none
        @ -----------------------------------------------------------------------------
        initColorwheel:
                push {r0, r1, r2, lr}
                mov r1, #0x10000
                str r1, [GPIOREG, #GPIO_OFFSET_CLEAR] @ clear Colorwheel-Dir-Pin to do clockwise rotation

                /* check if Colorwheel starts in hallsensor -> move outside to calibrate correctly */
                ldr r1, [GPIOREG, #GPIO_OFFSET_LEVEL] @ read GPIO Pin Level Register (GPLEV)
                tst r1, #0x00100000                   @ test if Hallsensor level==1 --> not detected
                moveq r0, #200
                bleq moveColorwheel                   @ move Colorwheel to get out of Hallsensor range

                mov r0, #1                            @ set stepsize to 1 for moveColorwheel-call
                mov r2, #0                            @ set counter to 0 for Hallsensor-stpes

        /* while Colorwheel-Hallsensor is not detected */
        initColorwheelLoopWhileNotDetected:
                ldr r1, [GPIOREG, #GPIO_OFFSET_LEVEL] @ read GPIO Pin Level Register (GPLEV)
                tst r1, #0x00100000                   @ test if Hallsensor level==1 --> not detected
                beq initColorwheelLoopWhileDetected   @ if detected -> move to next loop
                bl moveColorwheel                     @ else -> continue to turn
                b initColorwheelLoopWhileNotDetected  @ and repeat (loop)

        /* while Colorwheel-Hallsensor is detected */
        initColorwheelLoopWhileDetected:
                add r2, r2, #1
                ldr r1, [GPIOREG, #GPIO_OFFSET_LEVEL] @ read GPIO Pin Level Register (GPLEV)
                tst r1, #0x00100000                   @ test if Hallsensor level==1 --> not detected#
                bne initColorwheelExit                @ if not detected -> move to exit
                bl moveColorwheel                     @ else -> continue to turn
                b initColorwheelLoopWhileDetected     @ and repeat (loop)

        initColorwheelExit:
                mov r1, #0x10000
                str r1, [GPIOREG, #GPIO_OFFSET_SET]   @ set Colorwheel-Dir-Pin to do counter-clockwise rotation
                lsr r0, r2, #1
                bl moveColorwheel                     @ move Colorwheel backwards half the counted steps from detecting the hall sensor
                str r1, [GPIOREG, #GPIO_OFFSET_CLEAR] @ clear Colorwheel-Dir-Pin to do clockwise rotation
                pop {r0, r1, r2, lr}
                bx lr                                 @ return from function


        @ -----------------------------------------------------------------------------
        @ initialize the LED's with the correct colors
        @   param:     none
        @   return:    none
        @ -----------------------------------------------------------------------------
        initLEDs:
        /*  LED     COLOR     HEX          
           ----------------------------
            1       red       0xFF0000
            2       green     0x00FF00
            3       blue      0x2F9FD7
            4       brown     0x603A34
            5       orange    0xF26F22
            6       yellow    0xFFF200
        */
                push {r0, r1, lr}

                bl WS2812RPi_Init                     @ initialize the LEDs

                mov r0, #50
                bl WS2812RPi_SetBrightness

                mov r0, #IND_RED                      @ set LED 1 (IND_RED) to red
                ldr r1, =COLOR_RED
                bl WS2812RPi_SetSingle

                mov r0, #IND_GREEN                    @ set LED 2 (IND_GREEN) to green
                ldr r1, =COLOR_GREEN
                bl WS2812RPi_SetSingle
                bl WS2812RPi_Show                     @ show updated color values

                mov r0, #IND_BLUE                     @ set LED 3 (IND_BLUE) to blue
                ldr r1, =COLOR_BLUE
                bl WS2812RPi_SetSingle

                mov r0, #IND_BROWN                    @ set LED 4 (IND_BROWN) to brown
                ldr r1, =COLOR_BROWN
                bl WS2812RPi_SetSingle
                bl WS2812RPi_Show                     @ show updated color values

                mov r0, #IND_ORANGE                   @ set LED 5 (IND_ORANGE) to orange
                ldr r1, =COLOR_ORANGE
                bl WS2812RPi_SetSingle

                mov r0, #IND_YELLOW                   @ set LED 6 (IND_YELLOW) to yellow
                ldr r1, =COLOR_YELLOW
                bl WS2812RPi_SetSingle
                bl WS2812RPi_Show                     @ show updated color values

                pop {r0, r1, lr}
                bx lr                                 @ return from function


        @ -----------------------------------------------------------------------------
        @ initialize the counter and show it on the display
        @   param:     none
        @   return:    none
        @ -----------------------------------------------------------------------------
        initCounter:
                push {lr}
                mov CNTREG, #0
                bl showCounter
                pop {lr}
                bx lr                                 @ return from function

        @ -----------------------------------------------------------------------------
        @ loop that waites for the start button to be pressed
        @   param:     none
        @   return:    none
        @ -----------------------------------------------------------------------------
        programStart:
                ldr r0, [GPIOREG, #GPIO_OFFSET_LEVEL] @ read GPIO Pin Level Register (GPLEV)
                tst r0, #0x100                        @ test if Taster (nBTN1) level==1 --> not pressed
                bne programStart
                bx lr                                 @ return from function

        @ -----------------------------------------------------------------------------
        @ initialize the hardware and jumps into the main loop
        @   param:     none
        @   return:    none
        @ -----------------------------------------------------------------------------
        hwInit:
                ldr       r1, =gpio_mmap_adr          @ load the address for accessing the GPIOs into the GPIOREG
                ldr       GPIOREG, [r1]

                ldr       r1, =timerir_mmap_adr       @ load the address for accessing the Timer into the TMPREG
                ldr       TMPREG, [r1]
                
                bl initGPIOs
                
                bl initCounter

                bl initOutlet

                bl initColorwheel

                bl initLEDs

                bl programStart                    

                bl program

                bl shutdown

                b endOfApp

                @ TODO: PLEASE INIT HW HERE
                @ HINT:
                @   configuration of inputs is not necessary cause the pins are
                @   configured as inputs after reset

                @ TODO: BRANCH HERE TO YOUR APPLICATION CODE
                @ b         ...

                @ WARNING:
                @   call "endOfApp" if you're done with your application



/* program functions */
        @ -----------------------------------------------------------------------------
        @ program loop
        @   param:     none
        @   return:    none
        @ -----------------------------------------------------------------------------
        program:
                push {r0, r1, lr}
                mov r0, #0x00080000                   @ set Feeder (GoStop) Bit to 1
                str r0, [GPIOREG, #GPIO_OFFSET_SET]   @ write to GPIO Output Set Register (GPSET) -> set Feeder (GoStop) Bit to 1
        
        /* detect the color of the current M&M */
        programLoopGetColor:
                ldr r1, [GPIOREG, #GPIO_OFFSET_LEVEL] @ read GPIO Pin Level Register (GPLEV)
                tst r1, #0x200                        @ test if Taster (nBTN1) level==1 --> not pressed
                beq programLoopExit                   @ not pressed -> loop

                bl getColor                           @ get current M&Ms color in COLREG
                cmp COLREG, #IND_NA                   @ (COLREG == IND_NA) ?
                bne programLoopProcessMM              @ COLREG != IND_NA -> processMM
                mov r0, #400                          @ set to rotate quarter turn
                bl moveColorwheel                     @ COLREG == IND_NA -> moveColorwheel
                b programLoopGetColor                 @ and repeat (loop)

        /* process the detected M&M to fall into the correct bucket */
        programLoopProcessMM:
                bl setLed                             @ light up the bucket with corresponding M&M color

                bl moveSlide                          @ position the slide over the bucket with the corresponding M&M color
                
                mov r0, #400                          @ set to rotate quarter turn
                bl moveColorwheel                     @ turn colorwheel to let the current M&M fall out and get a new one in front of the color-detection

                add CNTREG, CNTREG, #1
                bl showCounter
                b programLoopGetColor                 @ and repeat (loop)

        programLoopExit:
                pop {r0, r1, lr}
                bx lr                                 @ return from function


        @ -----------------------------------------------------------------------------
        @ shutdown method to call before ending the program
        @   param:     none
        @   return:    none
        @ -----------------------------------------------------------------------------
        shutdown:
                push {lr}

                /* turn off Feeder */
                ldr r0, =0x00080000                   @ set Feeder (GoStop) Bit to 1
                str r0, [GPIOREG, #GPIO_OFFSET_CLEAR] @ write to GPIO Output Clear Register (GPCLR)

                /* turn LEDs off and deinitalize */
                bl WS2812RPi_AllOff
                bl WS2812RPi_Show                     @ turn off LED's
                bl WS2812RPi_DeInit                   @ deinitialize the LED's

                /* clear Colorwheel */
                ldr r0, =3200                         @ set to rotate two full rotations
                bl moveColorwheel                     @ turn colorwheel to let all M&M left in the colorwheel fall out

                /* clear 7-Seg */
                mov r1, #0xC0                         @ if next higher digit = 10 -> first Seg from right
                str r1, [GPIOREG, #GPIO_OFFSET_SET]   @ set chanel for given Seg
                mov r0, #0
                bl printPattern                       @ print nothing to the segment

                /* clear Output Register */
                ldr r0, =0x08000000                   @ set Co-Processor nSLP Bit to 1 -> so it shuts down
                orr r0, #0x00020000                   @ set Colorwheel nRST Bit to 1 -> set to RESET instruction
                orr r0, #0x00000800                   @ set Outlet nRST Bit to 1 -> set to RESET instruction
                str r0, [GPIOREG, #GPIO_OFFSET_CLEAR] @ write to GPIO Output Clear Register (GPCLR)

                pop {lr}
                bx lr                                 @ return from function


/* main functions */
        @ -----------------------------------------------------------------------------
        @ main function to call from outside
        @   param:     none
        @   return:    none
        @ -----------------------------------------------------------------------------
        main:
                ldr r0, =IntroMsg
                bl  printf

                @ GET GPIO VIRTUAL MEMORY ---------------------------------------------
                @ create backup and reserve stack space
                sub       sp, sp, #16                 @ space for saving regs
                str       r4, [sp, #0]                @ save r4
                str       r5, [sp, #4]                @      r5
                str       fp, [sp, #8]                @      fp
                str       lr, [sp, #12]               @      lr
                add       fp, sp, #12                 @ set our frame pointer
                sub       sp, sp, #STACK_ARGS         @ sp on 8-byte boundary

                @ open /dev/gpiomem for read/write and syncing
                ldr       r0, =gpiomem                @ address of /dev/gpiomem
                ldr       r1, openMode                @ flags for accessing device
                bl        open
                mov       r4, r0                      @ use r4 for file descriptor

                @ display file descriptor
                ldr       r0, =fdMsg                  @ format for printf
                mov       r1, r4                      @ file descriptor
                bl        printf

                @ map the GPIO registers to a virtual memory location so we can access them
                str       r4, [sp, #FILE_DESCRP_ARG]  @ /dev/gpiomem file descriptor
                ldr       r0, gpio                    @ address of GPIO
                str       r0, [sp, #DEVICE_ARG]       @ location of GPIO
                mov       r0, #NO_PREF                @ let kernel pick memory
                mov       r1, #PAGE_SIZE              @ get 1 page of memory
                mov       r2, #PROT_RDWR              @ read/write this memory
                mov       r3, #MAP_SHARED             @ share with other processes
                bl        mmap

                @ save virtual memory address
                ldr       r1, =gpio_mmap_adr          @ store gpio mmap (virtual address)
                str       r0, [r1]
                ldr       r1, =gpio_mmap_fd           @ store the file descriptor
                str       r4, [r1]

                ldr       r6, [r1]
                mov       r1, r0                      @ display virtual address
                ldr       r0, =memMsgGpio
                bl        printf
                mov       r1, r6
                ldr       r0, =memMsgGpio
                bl        printf

                @ restore sp and free stack
                add       sp, sp, #STACK_ARGS         @ fix sp
                ldr       r4, [sp, #0]                @ restore r4
                ldr       r5, [sp, #4]                @      r5
                ldr       fp, [sp, #8]                @         fp
                ldr       lr, [sp, #12]               @         lr
                add       sp, sp, #16                 @ restore sp

                @ GET TIMER + IR VIRTUAL MEMORY ---------------------------------------
                @ create backup and reserve stack space
                sub       sp, sp, #16                 @ space for saving regs
                str       r4, [sp, #0]                @ save r4
                str       r5, [sp, #4]                @      r5
                str       fp, [sp, #8]                @      fp
                str       lr, [sp, #12]               @      lr
                add       fp, sp, #12                 @ set our frame pointer
                sub       sp, sp, #STACK_ARGS         @ sp on 8-byte boundary

                @ open /dev/gpiomem for read/write and syncing
                ldr       r0, =mem                    @ address of /dev/mem
                ldr       r1, openMode                @ flags for accessing device
                bl        open
                mov       r4, r0                      @ use r4 for file descriptor

                @ display file descriptor
                ldr       r0, =fdMsg                  @ format for printf
                mov       r1, r4                      @ file descriptor
                bl        printf

                @ map the GPIO registers to a virtual memory location so we can access them
                str       r4, [sp, #FILE_DESCRP_ARG]  @ /dev/mem file descriptor
                ldr       r0, timerIR                 @ address of timer + IR
                str       r0, [sp, #DEVICE_ARG]       @ location of timer +IR
                mov       r0, #NO_PREF                @ let kernel pick memory
                mov       r1, #PAGE_SIZE              @ get 1 page of memory
                mov       r2, #PROT_RDWR              @ read/write this memory
                mov       r3, #MAP_SHARED             @ share with other processes
                bl        mmap

                @ save virtual memory address
                ldr       r1, =timerir_mmap_adr       @ store timer + IR mmap (virtual address)
                str       r0, [r1]
                ldr       r1, =timerir_mmap_fd        @ store the file descriptor
                str       r4, [r1]

                ldr       r6, [r1]
                mov       r1, r0                      @ display virtual address
                ldr       r0, =memMsgTimerIR
                bl        printf
                mov       r1, r6
                ldr       r0, =memMsgTimerIR
                bl        printf

                @ restore sp and free stack
                add       sp, sp, #STACK_ARGS         @ fix sp
                ldr       r4, [sp, #0]                @ restore r4
                ldr       r5, [sp, #4]                @      r5
                ldr       fp, [sp, #8]                @         fp
                ldr       lr, [sp, #12]               @         lr
                add       sp, sp, #16                 @ restore sp

                @ initialize all other hardware
                b         hwInit


        @ --------------------------------------------------------------------------------------------------------------------
        @
        @ END OF APPLICATION
        @
        @ --------------------------------------------------------------------------------------------------------------------
        endOfApp:
                ldr       r1, =gpio_mmap_adr          @ reload the addr for accessing the GPIOs
                ldr       r0, [r1]                    @ memory to unmap
                mov       r1, #PAGE_SIZE              @ amount we mapped
                bl        munmap                      @ unmap it
                ldr       r1, =gpio_mmap_fd           @ reload the addr for accessing the GPIOs
                ldr       r0, [r1]                    @ memory to unmap
                bl        close                       @ close the file

                ldr       r1, =timerir_mmap_adr       @ reload the addr for accessing the Timer + IR
                ldr       r0, [r1]                    @ memory to unmap
                mov       r1, #PAGE_SIZE              @ amount we mapped
                bl        munmap                      @ unmap it
                ldr       r1, =timerir_mmap_fd        @ reload the addr for accessing the Timer + IR
                ldr       r0, [r1]                    @ memory to unmap
                bl        close                       @ close the file

                mov       r0, #0                      @ return code 0
                mov       r7, #1                      @ exit app
                svc       0
                .end

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
