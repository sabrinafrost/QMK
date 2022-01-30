# MCU name
MCU = atmega32u4

# Bootloader selection
BOOTLOADER = atmel-dfu

# Build Options
# Change yes to no to disable
BOOTMAGIC_ENABLE = yes      # Enable Bootmagic Lite
MOUSEKEY_ENABLE = no        # Mouse keys
EXTRAKEY_ENABLE = yes       # Audio control and System control
CONSOLE_ENABLE = no         # Console for debug
COMMAND_ENABLE = no         # Commands for debug and configuration
NKRO_ENABLE = no            # Nkey Rollover - if this doesn't work, see here: https://github.com/tmk/tmk_keyboard/wiki/FAQ#nkro-doesnt-work
BACKLIGHT_ENABLE = no       # Enable keyboard backlight functionality
AUDIO_ENABLE = no           # Audio output
RGBLIGHT_ENABLE = yes       # Enable WS2812 RGB underlight.
LTO_ENABLE = yes
SPLIT_KEYBOARD = yes

# Do not enable SLEEP_LED_ENABLE
# It uses the same timer as BACKLIGHT_ENABLE
SLEEP_LED_ENABLE = no    	# Breathing sleep LED during USB suspend

DEFAULT_FOLDER = frostkit/kbo5000/rev2-ir
