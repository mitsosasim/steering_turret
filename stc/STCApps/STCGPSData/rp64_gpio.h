/* =======================================================================================================
  rp64_gpio.h
  Created on: Apr 28, 2025

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
    R. Filippone
    A. Kamal Eddine

 ------------------------------------------------------------------------------
  v. 1.0 - Apr 28, 2025 - Short description
 ------------------------------------------------------------------------------
  Reference:
    R1 - https://gist.github.com/CounterPillow/fe066655bf2d929148fe6eb3f15b1dd5#file-rp64_gpio-h
 ------------------------------------------------------------------------------
  Functions and logics

======================================================================================================= */
#ifndef RP64_GPIO_H_
#define RP64_GPIO_H_
/*
 * Permission to use, copy, modify, and/or distribute this software for any purpose with or without
 * fee is hereby granted.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS
 * SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */

#ifndef RP64_GPIO_H
#define RP64_GPIO_H

#define RP64_GPIO_CHIP_3   1
#define RP64_GPIO_LINE_3  20

#define RP64_GPIO_CHIP_5   1
#define RP64_GPIO_LINE_5  21

#define RP64_GPIO_CHIP_7   4
#define RP64_GPIO_LINE_7  24

#define RP64_GPIO_CHIP_8   4
#define RP64_GPIO_LINE_8  20

#define RP64_GPIO_CHIP_10  4
#define RP64_GPIO_LINE_10 19

#define RP64_GPIO_CHIP_11  1
#define RP64_GPIO_LINE_11 22

#define RP64_GPIO_CHIP_12  3
#define RP64_GPIO_LINE_12 24

#define RP64_GPIO_CHIP_13  1
#define RP64_GPIO_LINE_13 18

#define RP64_GPIO_CHIP_15  1
#define RP64_GPIO_LINE_15  1

#define RP64_GPIO_CHIP_16  1
#define RP64_GPIO_LINE_16  4

#define RP64_GPIO_CHIP_18  4
#define RP64_GPIO_LINE_18 21

#define RP64_GPIO_CHIP_19  1
#define RP64_GPIO_LINE_19  8

#define RP64_GPIO_CHIP_21  1
#define RP64_GPIO_LINE_21  7

#define RP64_GPIO_CHIP_22  4
#define RP64_GPIO_LINE_22 25

#define RP64_GPIO_CHIP_23  1
#define RP64_GPIO_LINE_23  9

#define RP64_GPIO_CHIP_24  1
#define RP64_GPIO_LINE_24 10

#define RP64_GPIO_CHIP_26  1
#define RP64_GPIO_LINE_26 13

#define RP64_GPIO_CHIP_27  1
#define RP64_GPIO_LINE_27 11

#define RP64_GPIO_CHIP_28  1
#define RP64_GPIO_LINE_28 12

#define RP64_GPIO_CHIP_29  4
#define RP64_GPIO_LINE_29 27

#define RP64_GPIO_CHIP_31  4
#define RP64_GPIO_LINE_31 28

#define RP64_GPIO_CHIP_32  3
#define RP64_GPIO_LINE_32 28

#define RP64_GPIO_CHIP_33  3
#define RP64_GPIO_LINE_33 29

#define RP64_GPIO_CHIP_35  3
#define RP64_GPIO_LINE_35 26

#define RP64_GPIO_CHIP_36  3
#define RP64_GPIO_LINE_36 30

#define RP64_GPIO_CHIP_37  3
#define RP64_GPIO_LINE_37 25

#define RP64_GPIO_CHIP_38  3
#define RP64_GPIO_LINE_38 27

#define RP64_GPIO_CHIP_40  3
#define RP64_GPIO_LINE_40 31

const unsigned int rp64_gpio_line[41] = {
    0,
    0,
    0,
    RP64_GPIO_LINE_3,
    0,
    RP64_GPIO_LINE_5,
    0,
    RP64_GPIO_LINE_7,
    RP64_GPIO_LINE_8,
    0,
    RP64_GPIO_LINE_10,
    RP64_GPIO_LINE_11,
    RP64_GPIO_LINE_12,
    RP64_GPIO_LINE_13,
    0,
    RP64_GPIO_LINE_15,
    RP64_GPIO_LINE_16,
    0,
    RP64_GPIO_LINE_18,
    RP64_GPIO_LINE_19,
    0,
    RP64_GPIO_LINE_21,
    RP64_GPIO_LINE_22,
    RP64_GPIO_LINE_23,
    RP64_GPIO_LINE_24,
    0,
    RP64_GPIO_LINE_26,
    RP64_GPIO_LINE_27,
    RP64_GPIO_LINE_28,
    RP64_GPIO_LINE_29,
    0,
    RP64_GPIO_LINE_31,
    RP64_GPIO_LINE_32,
    RP64_GPIO_LINE_33,
    0,
    RP64_GPIO_LINE_35,
    RP64_GPIO_LINE_36,
    RP64_GPIO_LINE_37,
    RP64_GPIO_LINE_38,
    0,
    RP64_GPIO_LINE_40,
};

const int rp64_gpio_chip[41] = {
    -1,
    -1,
    -1,
    RP64_GPIO_CHIP_3,
    -1,
    RP64_GPIO_CHIP_5,
    -1,
    RP64_GPIO_CHIP_7,
    RP64_GPIO_CHIP_8,
    -1,
    RP64_GPIO_CHIP_10,
    RP64_GPIO_CHIP_11,
    RP64_GPIO_CHIP_12,
    RP64_GPIO_CHIP_13,
    -1,
    RP64_GPIO_CHIP_15,
    RP64_GPIO_CHIP_16,
    -1,
    RP64_GPIO_CHIP_18,
    RP64_GPIO_CHIP_19,
    -1,
    RP64_GPIO_CHIP_21,
    RP64_GPIO_CHIP_22,
    RP64_GPIO_CHIP_23,
    RP64_GPIO_CHIP_24,
    -1,
    RP64_GPIO_CHIP_26,
    RP64_GPIO_CHIP_27,
    RP64_GPIO_CHIP_28,
    RP64_GPIO_CHIP_29,
    -1,
    RP64_GPIO_CHIP_31,
    RP64_GPIO_CHIP_32,
    RP64_GPIO_CHIP_33,
    -1,
    RP64_GPIO_CHIP_35,
    RP64_GPIO_CHIP_36,
    RP64_GPIO_CHIP_37,
    RP64_GPIO_CHIP_38,
    -1,
    RP64_GPIO_CHIP_40,
};

#define RP64_NUM_GPIO 40 // == preceding array length - 1

#endif



#endif /* RP64_GPIO_H_ */
