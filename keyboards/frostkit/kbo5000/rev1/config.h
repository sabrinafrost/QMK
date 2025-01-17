/*
Copyright 2012 Jun Wako <wakojun@gmail.com>
Copyright 2015 Jack Humbert

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

/* USB Device descriptor parameter */
#define VENDOR_ID       0xCB10
#define PRODUCT_ID      0x126A
#define DEVICE_VER      0x0100
#define MANUFACTURER    Frostrid of Bebbanburg
#define PRODUCT         Splits Montgomery Beauregard I

/* Matrix definitions (rows are doubled-up) */
#define MATRIX_ROWS 12
#define MATRIX_COLS 10
#define DIODE_DIRECTION COL2ROW

/* Pin assignments for each half */
#define MATRIX_ROW_PINS { B1, B2, C7, B4, D7, B0 }
#define MATRIX_COL_PINS { F4, NO_PIN, F0, B7, B3, D2, D3, D5, D4, NO_PIN }
#define MATRIX_ROW_PINS_RIGHT { B3, B2, B6, B4, D7, B0 }
#define MATRIX_COL_PINS_RIGHT { F1, F0, F4, F5, F6, D5, D2, D3, B7, B1 }
#define SPLIT_HAND_PIN F7
#define ENCODERS_PAD_A { F5, NO_PIN }
#define ENCODERS_PAD_B { F6, NO_PIN }
#define ENCODERS_PAD_A_RIGHT { D6, C7 }
#define ENCODERS_PAD_B_RIGHT { D4, C6 }

/* Other pin assignments */
#define SOFT_SERIAL_PIN D0 // serial.c configuration for split keyboard
#define BACKLIGHT_PIN B5
#define CAPS_LOCK_LED_PIN B6

/* RGB LED (WS2812) Config */
#define RGB_DI_PIN E6
#define RGBLIGHT_ANIMATIONS
#define RGBLED_NUM 16 // Number of LEDs
#define RGBLED_SPLIT { 8, 8 }
#define RGBLIGHT_LED_MAP { 1, 2, 3, 12, 13, 14, 15, 0, 7, 6, 5, 4, 11, 10, 9, 8 }

/* Settings */
#define DEBOUNCE 5 // Set 0 if debouncing isn't needed
#define TAPPING_TERM 150
#define TAP_CODE_DELAY 150
#define LOCKING_SUPPORT_ENABLE // Mechanical locking support. Use KC_LCAP, KC_LNUM or KC_LSCR instead in keymap
#define LOCKING_RESYNC_ENABLE // Locking resynchronize hack
