#include QMK_KEYBOARD_H

#include "rev1.h"
#include "keymap.h"
#include "action_tapping.h"
#include "kbo5000.h"
// #include "flash_stm32.h"

unsigned char carrierFreq = 0; // default
unsigned char period = 0; // calculated once for each signal sent in initSoftPWM
unsigned char periodHigh = 0; // calculated once for each signal sent in initSoftPWM
unsigned char periodLow = 0; // calculated once for each signal sent in initSoftPWM
unsigned long sigTime = 0; // used in mark & space functions to keep track of time
unsigned long sigStart = 0; // used to calculate correct length of existing signal, to handle some repeats

// static uint16_t ir_timer;
// static bool ir_is_transmitting;

// ir_timer = timer_read();

// if (timer_elapsed(key_timer) < 100) {
//   // do something if less than 100ms have passed
// } else {
//   // do something if 100ms or more have passed
// }



// void loop() {
//   // First send the NEC RAW signal defined above
//   sendRawBuf(NEC_RAW, sizeof(NEC_RAW) / sizeof(NEC_RAW[0]), 56);
//   delay(5000); // wait 5 seconds between each signal (change to suit)

//   // Next send the NEC_HEX_VALUE signal defined above
//   sendHexNEC(NEC_HEX_VALUE, NEC_BIT_COUNT, 1, 38);
//   delay(5000); // wait 5 seconds between each signal (change to suit)

//   // Next send the NEC_HEX_VALUE signal defined above
//   sendHexNEC(NEC_HEX_VALUE, NEC_BIT_COUNT, 1, 33);
//   delay(5000); // wait 5 seconds between each signal (change to suit)
// }
void delayMicroseconds(unsigned int us)
{
	// calling avrlib's delay_us() function with low values (e.g. 1 or
	// 2 microseconds) gives delays longer than desired.
	//delay_us(us);
#if F_CPU >= 20000000L
	// for the 20 MHz clock on rare Arduino boards

	// for a one-microsecond delay, simply wait 2 cycle and return. The overhead
	// of the function call yields a delay of exactly a one microsecond.
	__asm__ __volatile__ (
		"nop" "\n\t"
		"nop"); //just waiting 2 cycle
	if (--us == 0)
		return;

	// the following loop takes a 1/5 of a microsecond (4 cycles)
	// per iteration, so execute it five times for each microsecond of
	// delay requested.
	us = (us<<2) + us; // x5 us

	// account for the time taken in the preceeding commands.
	us -= 2;

#elif F_CPU >= 16000000L
	// for the 16 MHz clock on most Arduino boards

	// for a one-microsecond delay, simply return.  the overhead
	// of the function call yields a delay of approximately 1 1/8 us.
	if (--us == 0)
		return;

	// the following loop takes a quarter of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	us <<= 2;

	// account for the time taken in the preceeding commands.
	us -= 2;
#else
	// for the 8 MHz internal clock on the ATmega168

	// for a one- or two-microsecond delay, simply return.  the overhead of
	// the function calls takes more than two microseconds.  can't just
	// subtract two, since us is unsigned; we'd overflow.
	if (--us == 0)
		return;
	if (--us == 0)
		return;

	// the following loop takes half of a microsecond (4 cycles)
	// per iteration, so execute it twice for each microsecond of
	// delay requested.
	us <<= 1;

	// partially compensate for the time taken by the preceeding commands.
	// we can't subtract any more than this or we'd overflow w/ small delays.
	us--;
#endif

	// busy wait
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
}

// static uint16_t ir_timer;
// static uint16_t next_ir_transmission;

// LEADER_EXTERNS();

// void matrix_scan_user(void) {
//     LEADER_DICTIONARY() {
//         leading = false;
//         leader_end();

//         SEQ_ONE_KEY(KC_LEAD) {
//             tap_code(KC_CAPS);
//         }

//         SEQ_FOUR_KEYS(KC_I, KC_D, KC_L, KC_E) {
//             ir_is_transmitting = !ir_is_transmitting;
//             ir_timer = timer_read();
//         }

//         SEQ_TWO_KEYS(KC_O, KC_K) {
//             send_unicode_string("👍");
//         }

//         SEQ_THREE_KEYS(KC_S, KC_A, KC_D) {
//             send_unicode_string("😞");
//         }

//         SEQ_FIVE_KEYS(KC_C, KC_H, KC_E, KC_C, KC_K) {
//             send_unicode_string("✅");
//         }

//         SEQ_FIVE_KEYS(KC_C, KC_R, KC_O, KC_S, KC_S) {
//             send_unicode_string("❎");
//         }

//         SEQ_FIVE_KEYS(KC_T, KC_H, KC_A, KC_N, KC_K) {
//             send_unicode_string("🙏");
//         }

//         SEQ_FIVE_KEYS(KC_S, KC_M, KC_I, KC_L, KC_E) {
//             send_unicode_string("😊");
//         }

//         SEQ_FIVE_KEYS(KC_P, KC_A, KC_R, KC_T, KC_Y) {
//             send_unicode_string("🎉");
//         }

//         SEQ_FOUR_KEYS(KC_E, KC_Y, KC_E, KC_S) {
//             send_unicode_string("(ಠ_ಠ)");
//         }

//         SEQ_FIVE_KEYS(KC_M, KC_A, KC_G, KC_I, KC_C) {
//             send_unicode_string("(ಠ_ಠ) 🪄 ⠁⭒*.✫.*⭒⠁");
//         }

//         SEQ_FIVE_KEYS(KC_T, KC_A, KC_B, KC_L, KC_E) {
//             send_unicode_string("(ノಠ痊ಠ)ノ彡┻━┻");
//         }

//         SEQ_FIVE_KEYS(KC_S, KC_H, KC_R, KC_U, KC_G) {
//             send_unicode_string("¯\\_(ツ)_/¯");
//         }
//     }

//     if (ir_is_transmitting && timer_elapsed(ir_timer) > next_ir_transmission) {
//         ir_timer = timer_read();

//         uint8_t rval = TCNT0 + TCNT1 + TCNT3 + TCNT4;
//         next_ir_transmission = rval % 1000;

//         if (rval & 1) tap_code(KC_MS_UP);
//         if (rval & 2) tap_code(KC_MS_DOWN);
//         if (rval & 4) tap_code(KC_MS_LEFT);
//         if (rval & 8) tap_code(KC_MS_RIGHT);
//     }
// }


// void sendRawBuf(unsigned int *sigArray, unsigned int sizeArray, unsigned char kHz) {
//   if (carrierFreq != kHz)  initSoftPWM(kHz); // we only need to re-initialise if it has changed from last signal sent
//   sigTime = timer_read(); // keeps rolling track of signal time to avoid impact of loop & code execution delays
//   for (int i = 0; i < sizeArray; i++) {
//     mark(sigArray[i++]); // also move pointer to next position
//     if (i < sizeArray) { // check we have a space remaining before sending it
//       space(sigArray[i]); // pointer will be moved by for loop
//     }
//   }
// }

// void matrix_scan_user(void) {
void matrix_post_init_user(void) {
    setPinOutput(IR_TRANSMIT_PIN);

    // writePinHigh(IR_TRANSMIT_PIN);
    // delayMicroseconds((unsigned int)500000);
    // writePinLow(IR_TRANSMIT_PIN);
    // delayMicroseconds((unsigned int)5000000);
};

void sendHexNEC(unsigned long sigCode, char numBits, unsigned char repeats, unsigned char kHz) {

    /*  A basic 32 bit NEC signal is made up of:
    *  1 x 9000 uSec Header Mark, followed by
    *  1 x 4500 uSec Header Space, followed by
    *  32 x bits uSec ( 1- bit 560 uSec Mark followed by 1690 uSec space; 0 - bit 560 uSec Mark follwed by 560 uSec Space)
    *  1 x 560 uSec Trailer Mark
    *  There can also be a generic repeat signal, which is usually not neccessary & can be replaced by sending multiple signals
    */

    unsigned long bitMask = (unsigned long) 1 << (numBits - 1); // allows for signal from 1 bit up to 32 bits
    unsigned long start = 108000;

    if (carrierFreq != kHz)  initSoftPWM(kHz); // we only need to re-initialise if it has changed from last signal sent

    sigTime = timer_read(); // keeps rolling track of signal time to avoid impact of loop & code execution delays
    sigStart = sigTime; // remember for calculating first repeat gap (space), must end 108ms after signal starts
    // First send header Mark & Space
    mark(NEC_HEADER_MARK);
    space(NEC_HEADER_SPACE);

    while (bitMask) {
        if (bitMask & sigCode) { // its a One bit
            mark(NEC_ONE_MARK);
            space(NEC_ONE_SPACE);
        }
        else { // its a Zero bit
            mark(NEC_ZERO_MARK);
            space(NEC_ZERO_SPACE);
        }

        bitMask = (unsigned long) bitMask >> 1; // shift the mask bit along until it reaches zero & we exit the while loop
    }
    // Last send NEC Trailer MArk
    mark(NEC_TRAILER_MARK);

    // now send the requested number of NEC repeat signals. Repeats can be useful for certain functions like Vol+, Vol- etc
    /*  A repeat signal consists of
    *   A space which ends 108ms after the start of the last signal in this sequence
    *  1 x 9000 uSec Repeat Header Mark, followed by
    *  1 x 2250 uSec Repeat Header Space, followed by
    *  32 x bits uSec ( 1- bit 560 uSec Mark followed by 1690 uSec space; 0 - bit 560 uSec Mark follwed by 560 uSec Space)
    *  1 x 560 uSec repeat Trailer Mark
    */
    // First calcualte length of space for first repeat
    // by getting length of signal to date and subtracting from 108ms

    if (repeats == 0) return; // finished - no repeats
    else if (repeats > 0) { // first repeat must start 108ms after first signal
        space(start - (sigTime - sigStart)); // first repeat Header should start 108ms after first signal
        mark(NEC_HEADER_MARK);
        space(NEC_HEADER_SPACE / 2); // half the length for repeats
        mark(NEC_TRAILER_MARK);
    }

    while (--repeats > 0) { // now send any remaining repeats
        space(start - NEC_HEADER_MARK - NEC_HEADER_SPACE / 2 - NEC_TRAILER_MARK); // subsequent repeat Header must start 108ms after previous repeat signal
        mark(NEC_HEADER_MARK);
        space(NEC_HEADER_SPACE / 2); // half the length for repeats
        mark(NEC_TRAILER_MARK);
    }

}

void initSoftPWM(unsigned char carrierFreq) { // Assumes standard 8-bit Arduino, running at 16Mhz
  // supported values are 30, 33, 36, 38, 40, 56 kHz, any other value defaults to 38kHz
  // we will aim for a duty cycle of circa 33%

  period = (1000 + carrierFreq / 2) / carrierFreq;
  periodHigh = (period + 1) / 3;
  periodLow = period - periodHigh;

  switch (carrierFreq) {
    case 30  : // delivers a carrier frequency of 29.8kHz & duty cycle of 34.52%
      periodHigh -= 6; // Trim it based on measurementt from Oscilloscope
      periodLow  -= 10; // Trim it based on measurementt from Oscilloscope
      break;

    case 33  : // delivers a carrier frequency of 32.7kHz & duty cycle of 34.64%
      periodHigh -= 6; // Trim it based on measurementt from Oscilloscope
      periodLow  -= 10; // Trim it based on measurementt from Oscilloscope
      break;

    case 36  : // delivers a carrier frequency of 36.2kHz & duty cycle of 35.14%
      periodHigh -= 6; // Trim it based on measurementt from Oscilloscope
      periodLow  -= 11; // Trim it based on measurementt from Oscilloscope
      break;

    case 40  : // delivers a carrier frequency of 40.6kHz & duty cycle of 34.96%
      periodHigh -= 6; // Trim it based on measurementt from Oscilloscope
      periodLow  -= 11; // Trim it based on measurementt from Oscilloscope
      break;

    case 56  : // delivers a carrier frequency of 53.8kHz & duty cycle of 40.86%
      periodHigh -= 6; // Trim it based on measurementt from Oscilloscope
      periodLow  -= 12; // Trim it based on measurementt from Oscilloscope
      break;

    case 38  : // delivers a carrier frequency of 37.6kHz & duty cycle of 36.47%
    default :
      periodHigh -= 6; // Trim it based on measurementt from Oscilloscope
      periodLow  -= 11; // Trim it based on measurementt from Oscilloscope
      break;
  }
}

void mark(unsigned int mLen) { // uses sigTime as end parameter
  sigTime += mLen; // mark ends at new sigTime
  unsigned long now = timer_read();
  unsigned long dur = sigTime - now; // allows for rolling time adjustment due to code execution delays
  if (dur == 0) return;
  while ((timer_read() - now) < dur) { // just wait here until time is up
    writePinHigh(IR_TRANSMIT_PIN);
    if (periodHigh) delayMicroseconds(periodHigh);
    writePinLow(IR_TRANSMIT_PIN);
    if (periodLow) delayMicroseconds(periodLow);
  }
}

void space(unsigned int sLen) { // uses sigTime as end parameter
  sigTime += sLen; // space ends at new sigTime
  unsigned long now = timer_read();
  unsigned long dur = sigTime - now; // allows for rolling time adjustment due to code execution delays
  if (dur == 0) return;
  while ((timer_read() - now) < dur) ; // just wait here until time is up
}

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_ansi(
    MO(1),    KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,                     KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,           MD_TAPD, KC_MUTE, HA_TOG,
    KC_ESC,   KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,             KC_7,    KC_8,    KC_9,    KC_0,    KC_LBRC, KC_RBRC, KC_DEL,  KC_BSPC, KC_DEL,  KC_PGUP,
    CC_BTAB,  KC_TAB,  KC_QUOT, KC_COMM, KC_DOT,  KC_P,    KC_Y,                      KC_F,    KC_G,    KC_C,    KC_R,    KC_L,    KC_SLSH, KC_EQL,  KC_BSLS, KC_DEL,  KC_PGDN,
    CC_SCRN,  KC_CAPS, KC_A,    KC_O,    KC_E,    KC_U,    KC_I,                      KC_D,    KC_H,    KC_T,    KC_N,    KC_S,    KC_MINS,          KC_ENT,  MC_LSPC, MC_RSPC,
    KC_MPLY,  KC_LSFT,          KC_SCLN, KC_Q,    KC_J,    KC_K,    KC_X,             KC_B,    KC_M,    KC_W,    KC_V,    KC_Z,    KC_RSFT,                   KC_UP,
    CC_LOCK,  KC_LCTL, KC_LALT, KC_LGUI, KC_NO,   KC_SPC,  KC_SPC,                    LS_TAPD, MC_TAPD, KC_RGUI, KC_RALT, KC_RCTL,                   KC_LEFT, KC_DOWN, KC_RGHT
  ),
  [1] = LAYOUT_ansi(
    _______, _______, _______, _______, _______, _______, _______,                   _______, _______, _______, _______, _______, _______,          _______, RGB_MOD, RGB_TOG,
    RESET,   _______, _______, _______, _______, _______, _______, _______,          _______, _______, _______, _______, _______, _______, _______, _______, RGB_HUD, RGB_HUI,
    _______, _______, _______, _______, _______, _______, _______,                   _______, _______, _______, _______, _______, _______, _______, _______, RGB_SAD, RGB_SAI,
    _______, _______, _______, _______, _______, _______, _______,                   _______, _______, _______, _______, _______, _______,          _______, RGB_VAD, RGB_VAI,
    _______, _______,          _______, _______, _______, _______, _______,          _______, _______, _______, _______, _______,          _______,          _______,
    _______, _______, _______, _______, _______, _______, _______,                   _______, _______, _______, _______,                   _______, _______, _______, _______
  )
};

bool encoder_update_user(uint8_t index, bool clockwise) {
    switch (index) {
        case ENC_LEFT: // Encoder on the left half of the keyboard
            if (clockwise) sendHexNEC(NEC_HEX_VALUE, NEC_BIT_COUNT, 1, 38);
            else sendHexNEC(NEC_HEX_VALUE, NEC_BIT_COUNT, 1, 37);
            break;
        case ENC_RIGHT1: // First encoder on the right half of the keyboard
            if (clockwise) tap_code(KC_VOLU);
            else tap_code(KC_VOLD);
            break;
        case ENC_RIGHT2: // Second encoder on the right half of the keyboard
            if (clockwise) tap_code16(HA_BRUP);
            else tap_code16(HA_BRDN);
            break;
        default:
            break;
    }

    return true;
}

void tap_dance_handler(qk_tap_dance_state_t *state, qk_tap_dance_custom *keycodes, void action(uint16_t)) {
    switch (state->count) {
        case 1: // Single tap or hold
            if (state->interrupted || !state->pressed) action(keycodes->tap1);
            else action(keycodes->hold);
            break;
        case 2: // Double tap
            action(keycodes->tap2);
            break;
        case 3: // Triple tap
            action(keycodes->tap3);
            break;
        default:
            break;
    }
}

void tap_register(qk_tap_dance_state_t *state, void *user_data) {
    qk_tap_dance_custom *keycodes = (qk_tap_dance_custom *)user_data;
    tap_dance_handler(state, keycodes, register_code16);
}

void tap_reset(qk_tap_dance_state_t *state, void *user_data) {
    qk_tap_dance_custom *keycodes = (qk_tap_dance_custom *)user_data;
    tap_dance_handler(state, keycodes, unregister_code16);
}

qk_tap_dance_action_t tap_dance_actions[] = {
    [TD_MEDIA_CONTROL] = CUSTOM_TAP_DANCE(
        KC_NO,                  // Do nothing
        KC_MPLY,                // Play / Pause music
        KC_MEDIA_NEXT_TRACK,    // Next track
        KC_MEDIA_PREV_TRACK     // Previous track
    ),
    [TD_KEYBOARD_FUNCTIONS] = CUSTOM_TAP_DANCE(
        RESET,                  // Put keyboard into DFU mode
        KC_NO,                  // Do nothing
        KC_NO,                  // Do nothing
        EEP_RST                 // Reset the keyboard's persistent memory
    ),
    [TD_LAUNCHER] = CUSTOM_TAP_DANCE(
        KC_NO,                  // Do nothing
        LCMD(KC_SPC),           // Alfred
        LSFT(LCMD(KC_SPC)),     // 1Password Quick Access
        LOPT(KC_SPC)            // Spotlight
    ),
    [TD_MISSION_CONTROL] = CUSTOM_TAP_DANCE(
        KC_NO,                  // Do nothing
        LCTL(KC_UP),            // Exposé all windows (A for all)
        LCTL(KC_DOWN),          // Exposé app windows (L for local)
        KC_F11                  // Show desktop
    )
};
