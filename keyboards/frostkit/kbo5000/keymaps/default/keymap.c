#include QMK_KEYBOARD_H
#include "rev1.h"
#include "keymap.h"
#include "action_tapping.h"

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
            if (clockwise) rgblight_increase_hue(); // tap_code16(RGB_MODE_FORWARD);
            else rgblight_decrease_hue(); // tap_code16(RGB_MODE_REVERSE);
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
