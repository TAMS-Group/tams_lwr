#!/usr/bin/env python


# the transport-section buttons
CC_REWIND = 0x2B
CC_FORWARD = 0x2C
CC_STOP = 0x2A
CC_PLAY = 0x29
CC_PAUSE = 0x2D
CC_DOT = 0x2D

CC_CYCLE = 0x2E
CC_PREV_TRACK = 0x3A
CC_NEXT_TRACK = 0x3B
CC_SET_MARKER = 0x3C
CC_PREV_MARKER = 0x3D
CC_NEXT_MARKER = 0x3E

# faders and knobs
CC_FADER_0 = 0x00
CC_FADER_1 = 0x01
CC_FADER_2 = 0x02
CC_FADER_3 = 0x03
CC_FADER_4 = 0x04
CC_FADER_5 = 0x05
CC_FADER_6 = 0x06
CC_FADER_7 = 0x07

CC_KNOB_0 = 0x10
CC_KNOB_1 = 0x11
CC_KNOB_2 = 0x12
CC_KNOB_3 = 0x13
CC_KNOB_4 = 0x14
CC_KNOB_5 = 0x15
CC_KNOB_6 = 0x16
CC_KNOB_7 = 0x17

# add channel index (0..7) to access the per-channel buttons
CC_SOLO_BASE = 0x20
CC_MUTE_BASE = 0x30
CC_RECORD_BASE = 0x40

# the channel section buttons
CC_SOLO_0 = 0x20
CC_SOLO_1 = 0x21
CC_SOLO_2 = 0x22
CC_SOLO_3 = 0x23
CC_SOLO_4 = 0x24
CC_SOLO_5 = 0x25
CC_SOLO_6 = 0x26
CC_SOLO_7 = 0x27

CC_MUTE_0 = 0x30
CC_MUTE_1 = 0x31
CC_MUTE_2 = 0x32
CC_MUTE_3 = 0x33
CC_MUTE_4 = 0x34
CC_MUTE_5 = 0x35
CC_MUTE_6 = 0x36
CC_MUTE_7 = 0x37

CC_RECORD_0 = 0x40
CC_RECORD_1 = 0x41
CC_RECORD_2 = 0x42
CC_RECORD_3 = 0x43
CC_RECORD_4 = 0x44
CC_RECORD_5 = 0x45
CC_RECORD_6 = 0x46
CC_RECORD_7 = 0x47

LED_OFF = 0x00
LED_ON = 0x7F

axis_mapping = {
    CC_FADER_0: 0,
    CC_FADER_1: 1,
    CC_FADER_2: 2,
    CC_FADER_3: 3,
    CC_FADER_4: 4,
    CC_FADER_5: 5,
    CC_FADER_6: 6,
    CC_FADER_7: 7,
    CC_KNOB_0: 8,
    CC_KNOB_1: 9,
    CC_KNOB_2: 10,
    CC_KNOB_3: 11,
    CC_KNOB_4: 12,
    CC_KNOB_5: 13,
    CC_KNOB_6: 14,
    CC_KNOB_7: 15,
}

button_mapping = {
    CC_SOLO_0: 0,
    CC_SOLO_1: 1,
    CC_SOLO_2: 2,
    CC_SOLO_3: 3,
    CC_SOLO_4: 4,
    CC_SOLO_5: 5,
    CC_SOLO_6: 6,
    CC_SOLO_7: 7,
    CC_MUTE_0: 8 + 0,
    CC_MUTE_1: 8 + 1,
    CC_MUTE_2: 8 + 2,
    CC_MUTE_3: 8 + 3,
    CC_MUTE_4: 8 + 4,
    CC_MUTE_5: 8 + 5,
    CC_MUTE_6: 8 + 6,
    CC_MUTE_7: 8 + 7,
    CC_RECORD_0: 16 + 0,
    CC_RECORD_1: 16 + 1,
    CC_RECORD_2: 16 + 2,
    CC_RECORD_3: 16 + 3,
    CC_RECORD_4: 16 + 4,
    CC_RECORD_5: 16 + 5,
    CC_RECORD_6: 16 + 6,
    CC_RECORD_7: 16 + 7,
    CC_REWIND: 24 + 0,
    CC_FORWARD: 24 + 1,
    CC_STOP: 24 + 2,
    CC_PLAY: 24 + 3,
    CC_DOT: 24 + 4,
    CC_CYCLE: 24 + 5,
    CC_PREV_TRACK: 24 + 6,
    CC_NEXT_TRACK: 24 + 7,
    CC_SET_MARKER: 24 + 8,
    CC_PREV_MARKER: 24 + 9,
    CC_NEXT_MARKER: 24 + 10,
}

inverse_button_mapping = [
    CC_SOLO_0,
    CC_SOLO_1,
    CC_SOLO_2,
    CC_SOLO_3,
    CC_SOLO_4,
    CC_SOLO_5,
    CC_SOLO_6,
    CC_SOLO_7,
    CC_MUTE_0,
    CC_MUTE_1,
    CC_MUTE_2,
    CC_MUTE_3,
    CC_MUTE_4,
    CC_MUTE_5,
    CC_MUTE_6,
    CC_MUTE_7,
    CC_RECORD_0,
    CC_RECORD_1,
    CC_RECORD_2,
    CC_RECORD_3,
    CC_RECORD_4,
    CC_RECORD_5,
    CC_RECORD_6,
    CC_RECORD_7,
    CC_REWIND,
    CC_FORWARD,
    CC_STOP,
    CC_PLAY,
    CC_DOT,
    CC_CYCLE,
    CC_PREV_TRACK,
    CC_NEXT_TRACK,
    CC_SET_MARKER,
    CC_PREV_MARKER,
    CC_NEXT_MARKER,
]