#pragma once

#include QMK_KEYBOARD_H
#include "kbo5000.h"
#include "quantum.h"

enum encoder_names {
  ENC_LEFT = 0,
  ENC_RIGHT1 = 2,
  ENC_RIGHT2
};

enum td_keycodes {
    TD_MEDIA_CONTROL,
    TD_KEYBOARD_FUNCTIONS,
    TD_LAUNCHER,
    TD_MISSION_CONTROL
};

typedef struct {
    uint16_t hold;
    uint16_t tap1;
    uint16_t tap2;
    uint16_t tap3;
} qk_tap_dance_custom;

#define CC_BTAB LSFT(KC_TAB) // Reverse tab for outdent
#define CC_LOCK LCTL(LCMD(KC_Q)) // Lock macOS
#define CC_SCRN LSFT(LCMD(KC_4)) // Screenshot

// Tap Dance Aliases
#define KB_TAPD TD(TD_KEYBOARD_FUNCTIONS)
#define LS_TAPD TD(TD_LAUNCHER)
#define MC_TAPD TD(TD_MISSION_CONTROL) // Tap: Show app windows, double tap: show all windows, triple tap: show desktop
#define MD_TAPD TD(TD_MEDIA_CONTROL) // Tap: Show app windows, double tap: show all windows, triple tap: show desktop

// Mission Control Shortcuts
#define MC_LSPC LCTL(KC_LEFT) // Switch to space on the left
#define MC_RSPC LCTL(KC_RIGHT) // Switch to space on the right

// Home Automation Shortcuts
#define HA_BRUP LCTL(LOPT(LCMD(KC_EQL))) // Increase office lighting
#define HA_BRDN LCTL(LOPT(LCMD(KC_MINS))) // Decrease office lighting
#define HA_TOG LCTL(LOPT(LCMD(KC_ENT))) // Toggle office lighting

#define CUSTOM_TAP_DANCE(hold, tap1, tap2, tap3) \
{ .fn = {NULL, tap_register, tap_reset}, .user_data = (void *)&((qk_tap_dance_custom){hold, tap1, tap2, tap3}), }

#define LAYOUT( \
    LF1,      LF4, LF5, LF6, LF7, LF8, LF9, \
    LA1, LA3, LA4, LA5, LA6, LA7, LA8, LA9, \
    LB1, LB3, LB4, LB5, LB6, LB7, LB8,      \
    LC1, LC3, LC4, LC5, LC6, LC7, LC8,      \
    LD1, LD3,      LD5, LD6, LD7, LD8, LD9, \
    LE1, LE3, LE4, LE5, LE6, LE7, LE8,      \
    \
    RF1, RF2,      RF4, RF5, RF6, RF7, RF8, RF9, RF10, \
    RA1, RA2, RA3, RA4, RA5, RA6, RA7, RA8, RA9, RA10, \
    RB1, RB2, RB3, RB4, RB5, RB6, RB7, RB8, RB9, RB10, \
    RC1, RC2, RC3, RC4, RC5, RC6,      RC8, RC9, RC10, \
    RD1, RD2, RD3, RD4, RD5,      RD7,      RD9,       \
    RE1, RE2, RE3, RE4,           RE7, RE8, RE9, RE10  \
) \
{ \
    { LA1, KC_NO, LA3, LA4, LA5, LA6, LA7, LA8, LA9, KC_NO },   \
    { LB1, KC_NO, LB3, LB4, LB5, LB6, LB7, LB8, KC_NO, KC_NO }, \
    { LC1, KC_NO, LC3, LC4, LC5, LC6, LC7, LC8, KC_NO, KC_NO }, \
    { LD1, KC_NO, LD3, KC_NO, LD5, LD6, LD7, LD8, LD9, KC_NO }, \
    { LE1, KC_NO, LE3, LE4, LE5, LE6, LE7, LE8, KC_NO, KC_NO }, \
    { LF1, KC_NO, KC_NO, LF4, LF5, LF6, LF7, LF8, LF9, KC_NO }, \
    \
    { RA1, RA2, RA3, RA4, RA5, RA6, RA7, RA8, RA9, RA10 },      \
    { RB1, RB2, RB3, RB4, RB5, RB6, RB7, RB8, RB9, RB10 },      \
    { RC1, RC2, RC3, RC4, RC5, RC6, KC_NO, RC8, RC9, RC10 },    \
    { RD1, RD2, RD3, RD4, RD5, KC_NO, RD7, KC_NO, RD9, KC_NO }, \
    { RE1, RE2, RE3, RE4, KC_NO, KC_NO, RE7, RE8, RE9, RE10 },  \
    { RF1, RF2, KC_NO, RF4, RF5, RF6, RF7, RF8, RF9, RF10 }     \
}
