#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from pykit.logger import Logger

from constants import kRobotUpdatePeriod
from pykit.loggedrobot import LoggedRobot

from utils.calibration import Calibration, CalibrationWrangler
from utils.loggerSupport import startLogger
from wpilib import run

LoggedRobot.default_period = kRobotUpdatePeriod
class MyRobot(LoggedRobot):

    #########################################################
    ## The Dunder init constructor
    def __init__(self) -> None:
        super().__init__()
        startLogger(self)

    #########################################################
    ## Common initialization for all modes
    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """

        self.teleopCountRollOverCalibration = Calibration("Count roll-over value", 100)
        self.rolloverValue = int(self.teleopCountRollOverCalibration.get())
        self.count = 0

    #########################################################
    ## Common update for all modes
    def robotPeriodic(self) -> None:
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""
        CalibrationWrangler().update

    #########################################################
    ## Initialization for the autonomous mode
    ## Called once when autonomous starts
    def autonomousInit(self) -> None:
        """Called after robotInit, but before autonomousPeriodic"""
        pass

    #########################################################
    ## Common update for the autonomouse mode
    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        pass

    #########################################################
    ## Post autonomous mode cleanup
    ## Called once when autonomous finishes
    def autonomousExit(self):
        """This function is called after autonomous"""
        pass

    #########################################################
    ## Initialization for the teleop mode
    ## Called once when teleop starts
    def teleopInit(self) -> None:
        """Called after robotInit, but before teleopPeriodic"""
        pass

    #########################################################
    ## Post teleop mode cleanup
    ## Called once when teleop starts
    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control (teleop)"""

        #self.rolloverValue = int(self.teleopCountRollOverCalibration.get())
        self.count = (self.count + 1) % self.rolloverValue
        Logger.recordOutput("count", self.count)

    #########################################################
    ## Post teleop mode cleanup
    ## Called once when teleop finishes
    def telepExit(self) -> None:
        """This function is called after teleop"""
        pass


if __name__ == "__main__":
    run(MyRobot)
