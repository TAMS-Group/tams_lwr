#!/usr/bin/env python
#
# joystick input driver for the KORG nanokontrol MIDI controller.
# Connects to the MIDI controller and forwards all incoming
# MIDI events to the /joy topic, where the faders and knobs
# are mapped to joystick-axes, while the channel and transport
# buttons are mapped to joystick button events.
#
# Slightly modified version of korg_nanokontrol/kontrol.py
# Author: Austin Hendrix
#
# Author: Norman Hendrich, hendrich@informatik.uni-hamburg.de
#
# Note that the device defaults to "internal" LED control,
# where the button LEDs are active while the button is pressed,
# or depending on the selected workstation mode (Cubase, etc.).
# Also note that control of the track and marker LEDs is still
# up to the selected DAW workstation mode (and might not work).
#
# To enable "external" LED control, install and run the
# Korg Kontrol Editor program (e.g., using Wine). Then:
# (Control drop-down list) -> Common -> LED Mode -> External
# Write Scene.
# Quit Kontrol Editor
#
# 2017.02.13 - update for LWR / tams_f329
# 2015.12.25 - add JointState subscriber for single-message LED feedback
# 2015.12.24 - catkinize

import roslib

import rospy

import pygame
import pygame.midi
import sys
from sensor_msgs.msg import *

roslib.load_manifest("tams_lwr")

# global variable to handle the setLED calls
leds = None

# the transport-section buttons
#
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
#
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
#
CC_SOLO_BASE = 0x20
CC_MUTE_BASE = 0x30
CC_RECORD_BASE = 0x40

# the channel section buttons
#
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


# set the LED selected by cc to the given value, where 0..0x40=off,
# and 0x41..0x7F = on. Note that this function only works if the
# nanokontrol device is operating in "external-LED-control" mode.
# You may have to use their Windows tool to select this mode once...
#
def setLED(cc, on):
    global leds
    value = 0x0
    if on:
        value = 0x7F
    leds.write_short(0xB0, cc & 0xFF, value & 0x7F)


def joyFeedbackCallback(data):
    # print "got JoyFeedback type %d id %d value %f " % (data.type, data.id, data.intensity)
    cc = data.id
    if button_mapping[cc] >= 0:  # requested cc/LED exists
        ledOn = 0x00
        if data.intensity > 0:
            ledOn = 0x7F
        # print "calling setLED %d %d" % (cc, ledOn)
        setLED(cc, ledOn)


def ledFeedbackCallback(data):
    # print "got ledFeedback Joy message at %s " % data.header.stamp
    for i in range(len(data.buttons)):
        # print " %d %d " % (len(data.buttons), i )
        if data.buttons[i] == 1:
            setLED(inverse_button_mapping[i], LED_ON)
        else:
            setLED(inverse_button_mapping[i], LED_OFF)


def main():
    global leds

    pygame.midi.init()
    devices = pygame.midi.get_count()
    if devices < 1:
        print("No MIDI devices detected")
        exit(-1)
    print("Found %d MIDI devices" % devices)

    if len(sys.argv) > 2:
        input_dev = int(sys.argv[1])
        output_dev = int(sys.argv[2])
    else:
        print("Usage: nanokontrol.py <input-id> <output-id>")
        print("Interface Device-Name Input Output Opened:")
        for i in range(devices):
            print(pygame.midi.get_device_info(i))
        exit(-1)

    print("Using input device %d" % input_dev)

    controller = pygame.midi.Input(input_dev)
    leds = pygame.midi.Output(output_dev)

    rospy.init_node("nanokontrol")
    namespace = ""  # rospy.get_namespace() # 'nanokontrol/'
    # joyPublisher = rospy.Publisher( namespace + 'joy' , Joy, latch=True)
    joyPublisher = rospy.Publisher(namespace + "joy", Joy, queue_size=1)
    joyFeedbackSubscriber = rospy.Subscriber(
        namespace + "feedback", JoyFeedback, joyFeedbackCallback
    )
    ledFeedbackSubscriber = rospy.Subscriber(
        namespace + "leds", Joy, ledFeedbackCallback
    )

    m = Joy()
    m.axes = [0] * (8 + 8)
    m.buttons = [0] * (11 + 3 * 8)

    p = False

    while not rospy.is_shutdown():
        m.header.stamp = rospy.Time.now()
        # count the number of events that are coalesced together
        c = 0
        while controller.poll():
            c += 1
            data = controller.read(1)
            # print data
            # loop through events received
            for event in data:
                control = event[0]
                timestamp = event[1]

                # look for continuous controller commands
                if (control[0] & 0xF0) == 0xB0:  # controller change: faders and knobs
                    control_id = ((control[0] & 0x0F) << 8) | (control[1] & 0xFF)

                    control_val = float(control[2] - 63) / 63.0
                    if control_val < -1.0:
                        control_val = -1.0
                    if control_val > 1.0:
                        control_val = 1.0

                    # print "Control ID is %d " % control_id

                    if control_id in axis_mapping:
                        axis = axis_mapping[control_id]
                        m.axes[axis] = control_val
                        p = True

                        ###
                        # mask = 0x80;
                        # for i in range(0x30, 0x38):
                        #    ledOn = False
                        #    if (mask & control[2]) != 0:
                        #       ledOn = True
                        #    setLED( i, ledOn )
                        #    mask = mask >> 1;
                        ###

                    elif control_id in button_mapping:
                        button = button_mapping[control_id]
                        if control[2] != 0:
                            m.buttons[button] = 1
                        else:
                            m.buttons[button] = 0
                        p = True

                    else:
                        print("Unknown control ID %d " % control_id)

                # look for mode commands
                elif control[0] == 79:
                    mode = control[1]
                    print("WARNING: ignoring mode command %d" % mode)

        if p:
            joyPublisher.publish(m)
            p = False

        rospy.sleep(0.01)  # 100Hz maximum input


# rosrun tams_f329 nanokontrol.py 3 2
#
# rosrun tams_f329 nanokontrol.py
# Found 4 MIDI devices
# Usage: nanokontrol.py <input-id> <output-id>
# Interface Device-Name Input Output Opened:
# ('ALSA', 'Midi Through Port-0', 0, 1, 0)
# ('ALSA', 'Midi Through Port-0', 1, 0, 0)
# ('ALSA', 'nanoKONTROL2 MIDI 1', 0, 1, 0)
# ('ALSA', 'nanoKONTROL2 MIDI 1', 1, 0, 0)
#
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
