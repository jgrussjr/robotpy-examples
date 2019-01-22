#!/usr/bin/env python3

import wpilib


class MyRobot(wpilib.SampleRobot):
    """Main robot class"""

    def robotInit(self):
        """Robot-wide initialization code should go here"""

        self.lstick = wpilib.Joystick(0)
        self.rstick = wpilib.Joystick(1)

        self.l_motor = wpilib.Jaguar(1)
        self.r_motor = wpilib.Jaguar(2)

        # Position gets automatically updated as robot moves
        self.gyro = wpilib.AnalogGyro(1)

        self.robot_drive = wpilib.RobotDrive(self.l_motor, self.r_motor)

        self.motor = wpilib.Jaguar(4)

        # tape sensor inputs
        self.sensor_c = wpilib.DigitalInput(4)
        self.sensor_l = wpilib.DigitalInput(5)
        self.sensor_r = wpilib.DigitalInput(6)

        self.position = wpilib.AnalogInput(2)

    def disabled(self):
        """Called when the robot is disabled"""
        while self.isDisabled():
            wpilib.Timer.delay(0.01)

    def autonomous(self):
        """Called when autonomous mode is enabled"""

        timer = wpilib.Timer()
        timer.start()

        while self.isAutonomous() and self.isEnabled():

            if timer.get() < 2.0:
                self.robot_drive.arcadeDrive(-1.0, 0.3)
            else:
                self.robot_drive.arcadeDrive(0, 0)

            wpilib.Timer.delay(0.01)

    def operatorControl(self):
        """Called when operation control mode is enabled"""

        timer = wpilib.Timer()
        timer.start()

        while self.isOperatorControl() and self.isEnabled():

            self.robot_drive.arcadeDrive(self.lstick)

            # Move a motor with a Joystick
            y = self.rstick.getY()
            self.motor.set(y)

            # Don't print every time
            if timer.hasPeriodPassed(0.5):
                print("Tape: -- " + str(int(self.sensor_l.get())) + " | " +  str(int(self.sensor_c.get())) + " | " + str(int(self.sensor_r.get())) + " --")

            wpilib.Timer.delay(0.04)


if __name__ == "__main__":

    wpilib.run(MyRobot, physics_enabled=True)
