#!/usr/bin/env python3

'''
This file is part of the ddcontroller library (https://github.com/ansarid/ddcontroller).
Copyright (C) 2022  Daniyal Ansari

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

import RPi.GPIO as GPIO
GPIO.setwarnings(False)
import time

class PayloadBucket:
    """Class for controlling a servo using the PWM.

    Args:
        pin: The GPIO pin that will be used to control the servo.
        pwm_frequency (int): The frequency of the pulse width modulation (PWM) signal
            that will be used to control the motor.  Defaults to 300 Hz
        servo_angle_min (int, optional): Servo angle at minimum pulse width. Defaults to 0 degrees
        servo_angle_max (int, optional): Servo angle at maximum pulse width. Defaults to  270 degrees
        min_pulse_width (int, optional): The minimum pulse width in microseconds.  Defaults to 500.
        max_pulse_width (int, optional): The maximum pulse width in microseconds.  Defaults to 2500.
        initial_angle (int, optional): The initial angle of the servo in degrees.
        servo_range_min (int, optional): Minimum angle of the servo in degrees.
        servo_range_max (int, optional): Maximum anagle of the servo in degrees.
    """

    def __init__(self, pin, pwm_frequency=300,
                 servo_angle_min=0,
                 servo_angle_max=270,
                 min_pulse_width=500,
                 max_pulse_width=2500,
                 initial_angle=0,
                 range_angle_min=0,
                 range_angle_max=45):
        self.pin = pin

        # Initial angle
        self.init_angle = initial_angle
        self.servo_angle_min = servo_angle_min
        self.servo_angle_max = servo_angle_max

        # PWM frequency (Hz)
        self.pwm_frequency = pwm_frequency

        # Min pulse width at min servo angle
        self.min_pulse_width = min_pulse_width

        # Max pulse width at max servo angle
        self.max_pulse_width = max_pulse_width

        # Permitted servo range angles
        self.servo_range_min = range_angle_min
        self.servo_range_max = range_angle_max

        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BOARD)
        else:
            pass

        # setup pin as PWM
        GPIO.setup(self.pin, GPIO.OUT)
        self._pwm = GPIO.PWM(self.pin, self.pwm_frequency)
        initial_duty = self._angle_to_duty(self.init_angle)
        self._pwm.start(initial_duty)

    def set_pwm_frequency(self, frequency):
        """Sets the frequency of the PWM signal.

        Args:
            frequency (int): The new frequency of the PWM signal.
        """
        self.pwm_frequency = frequency
        self._pwm.ChangeFrequency(self.pwm_frequency)

    def _angle_to_duty(self, angle):
        """Calculates the duty for a given angle, bounded to permissible range
        Args:
            angle (float): The input angle in degrees
        """        
        # make sure the angle is in range. If not, clip to the set range
        if angle < self.servo_angle_min :
            angle = self.servo_angle_min
        elif angle > self.servo_angle_max :
            angle = self.servo_angle_max

        # Calculate the line equation
        m = (self.servo_angle_max - self.servo_angle_min)/(self.max_pulse_width-self.min_pulse_width)
        b = -m*(self.min_pulse_width)

        # Calculate the desired pulse width in microseconds
        pulse_width = (angle - b)/m

        # Caclulate the duty cycle based on the desired pulse width in microseconds
        period = 1/self.pwm_frequency * 1e6
        duty = pulse_width/period*100

        return duty


    def set_servo(self, angle):
        """Sets the angle of the servo
        Args:
            angle (float): The new angle of the servo in degrees
        """

        duty = self._angle_to_duty(angle)

        #print(f'Setting PWM duty to {duty:.3f}')

        # Set the duty cycle for the PWM
        self._pwm.ChangeDutyCycle(duty)

    def dispense_one_ball(self):
        """ Dispenses one ball from the payload bucket
        Arg:
            None
        """
        min_angle=0
        max_angle=20
        shake_angle=20
        num_steps=100
        ramp_time=2.5
        lower_time=0.25
        self.set_servo(min_angle)

        angle_step = (max_angle-min_angle)/num_steps
        angle = min_angle
        while angle < max_angle:
            self.set_servo(angle)
            time.sleep(ramp_time/num_steps)
            angle += angle_step

        self.set_servo(angle+shake_angle)
        time.sleep(.15)
        self.set_servo(angle)
        time.sleep(.15)

        while angle > min_angle:
            self.set_servo(angle)
            time.sleep(lower_time/num_steps)
            angle -= angle_step

    def shake(self):
        """ shakes the bucket
        Arg:
            None
        """
        min_angle=0
        max_angle=15
        num_steps=5
        ramp_time=0.25
        lower_time=0.25
        num_shakes = 2
        self.set_servo(min_angle)

        angle_step = (max_angle-min_angle)/num_steps
        angle = min_angle

        for i in range(num_shakes):
            # Raise the bucket
            while angle < max_angle:
                self.set_servo(angle)
                time.sleep(ramp_time/num_steps)
                angle += angle_step
            # Lower the bucket
            while angle > min_angle:
                self.set_servo(angle)
                time.sleep(lower_time/num_steps)
                angle -= angle_step
                     
        self.set_servo(min_angle)
        
    def stop(self):
        """Stops the motor by setting the duty cycle of the PWM signal to 0."""

        duty = self._angle_to_duty(self.init_angle)
        self._pwm.ChangeDutyCycle(duty)
        time.sleep(0.2)
        self._pwm.stop()

        GPIO.cleanup(self.pin)

if __name__ == "__main__":

    min_angle=0
    max_angle=30
    num_steps=100
    ramp_time=5
    try:
        bucket = PayloadBucket(pin=33,range_angle_min=0, range_angle_max=45)
        while True:
            # Simple ramp from min to max range over ramp time

            bucket.dispense_one_ball()

            time.sleep(1)
    except KeyboardInterrupt:
        print('Stopping...')
    finally:
        bucket.stop()

