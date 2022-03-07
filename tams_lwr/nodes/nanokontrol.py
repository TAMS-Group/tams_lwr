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

import sys
import pygame
import pygame.midi
import rospy
from sensor_msgs.msg import Joy, JoyFeedback
from nano_common import axis_mapping, button_mapping, inverse_button_mapping, LED_ON, LED_OFF


class MiDi:
    def __init__(self):
        pygame.midi.init()
        devices = pygame.midi.get_count()
        if devices < 1:
            print("No MIDI devices detected")
            exit()
        print("Found %d MIDI devices" % devices)

        if len(sys.argv) > 2:
            input_dev = int(sys.argv[1])
            output_dev = int(sys.argv[2])
        else:
            print("Usage: nanokontrol.py <input-id> <output-id>")
            print("Interface Device-Name Input Output Opened:")
            for i in range(devices):
                print(pygame.midi.get_device_info(i))
            raise NotImplementedError

        print("Using input device %d" % input_dev)

        controller = pygame.midi.Input(input_dev)
        self.leds = pygame.midi.Output(output_dev)

        rospy.init_node("nanokontrol")
        namespace = ""
        joyPublisher = rospy.Publisher(namespace + "joy", Joy, queue_size=1)
        rospy.Subscriber(namespace + "feedback", JoyFeedback, self.joyFeedbackCallback)
        rospy.Subscriber(namespace + "leds", Joy, self.ledFeedbackCallback)

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
                for event in data:
                    control = event[0]

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
            rospy.sleep(0.01)  # 100Hz maximum input_

    def setLED(self, cc, on):
        """
        # set the LED selected by cc to the given value, where 0..0x40=off,
        # and 0x41..0x7F = on. Note that this function only works if the
        # nanokontrol device is operating in "external-LED-control" mode.
        # You may have to use their Windows tool to select this mode once...
        """
        value = 0x0
        if on:
            value = 0x7F
        self.leds.write_short(0xB0, cc & 0xFF, value & 0x7F)

    def joyFeedbackCallback(self, data):
        cc = data.id
        if button_mapping[cc] >= 0:  # requested cc/LED exists
            ledOn = 0x00
            if data.intensity > 0:
                ledOn = 0x7F
            self.setLED(cc, ledOn)

    def ledFeedbackCallback(self, data):
        for i in range(len(data.buttons)):
            if data.buttons[i] == 1:
                self.setLED(inverse_button_mapping[i], LED_ON)
            else:
                self.setLED(inverse_button_mapping[i], LED_OFF)


def main():
    MiDi()


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
    main()
