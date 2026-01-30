import os
from typing import Optional
from commands2 import Command, cmd
from commands2.sysid import SysIdRoutine
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from pathplannerlib.auto import AutoBuilder
from wpilib import getDeployDirectory, XboxController

from commands.drivecommands import DriveCommands
from subsystems.drive.drive import Drive
from subsystems.drive.driveio import DriveIO
from subsystems.drive.driveiosim import DriveIOSim
from subsystems.drive.driveioromispark import DriveIORomiSpark

from subsystems.drive.gyroio import GyroIO
from subsystems.drive.gyroioromi import GyroIORomi

import constants


class RobotContainer:


    def __init__(self) -> None:


        self.controller = XboxController(0)

        match constants.kRobotMode:
            case constants.RobotModes.REAL:
                self.drive = Drive(DriveIORomiSpark(), GyroIORomi())
                self.drive.io.debugController : XboxController|None = self.controller
                #TODO self.drive = Drive(DriveIORomiSpark(), GyroIOPigeon2())
            case constants.RobotModes.SIMULATION:
                self.drive = Drive(DriveIOSim(), GyroIO())
            case constants.RobotModes.REPLAY:
                self.drive = Drive(DriveIO(), GyroIO())

        auto_folder_path = os.path.join(getDeployDirectory(), "pathplanner", "autos")
        auto_list = os.listdir(auto_folder_path)

        self.autoChooser: LoggedDashboardChooser[Command] = LoggedDashboardChooser(
            "Auto Choices"
        )
        for auto in auto_list:
            auto = auto.removesuffix(".auto")
            self.autoChooser.addOption(
                auto,
                AutoBuilder.buildAuto(auto),
            )
        self.autoChooser.setDefaultOption("Do Nothing", cmd.none())

        self.testChooser: LoggedDashboardChooser[Command] = LoggedDashboardChooser(
            "Test Choices"
        )

        ffWaitCmd = cmd.waitUntil(lambda: self.controller.getRightBumper())
        ffCmd = DriveCommands.feedForwardCharacterization(self.drive)
        ffWithWaits = ffWaitCmd.andThen(ffCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        self.testChooser.addOption(
            "Drive Simple FF Characterization",
            ffWithWaits
        )

        sysIdQFWaitCmd = cmd.waitUntil(lambda: self.controller.getRightBumper())
        sysIdQFCmd = self.drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        sysIdQFWithWaits = sysIdQFWaitCmd.andThen(sysIdQFCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        self.testChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            sysIdQFWithWaits
        )

        sysIdQRWaitCmd = cmd.waitUntil(lambda: self.controller.getRightBumper())
        sysIdQRCmd = self.drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        sysIdQRWithWaits = sysIdQRWaitCmd.andThen(sysIdQRCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        self.testChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            sysIdQRWithWaits
        )

        sysIdDFWaitCmd = cmd.waitUntil(lambda: self.controller.getRightBumper())
        sysIdDFCmd = self.drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        sysIdDFWithWaits = sysIdDFWaitCmd.andThen(sysIdDFCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        self.testChooser.addOption(
            "Drive SysId (Dynamic Forward)",
            sysIdDFWithWaits
        )

        sysIdDRWaitCmd = cmd.waitUntil(lambda: self.controller.getRightBumper())
        sysIdDRCmd = self.drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        sysIdDRWithWaits = sysIdDRWaitCmd.andThen(sysIdDRCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        self.testChooser.addOption(
            "Drive SysId (Dynamic Reverse)",
            sysIdDRWithWaits
        )

        self.testChooser.addOption(
            "Do Nothing Repeatedly",
            cmd.idle(self.drive)
        )

        self.testChooser.setDefaultOption("Do Nothing Once", cmd.none())

        self.configureButtonBindingsNone()

    def configureButtonBindingsNone(self) -> None:
        self.drive.setDefaultCommand(
            DriveCommands.arcadeDriveOpenLoop(
                self.drive,
                lambda: 0.0,
                lambda: 0.0,
                lambda: 0.25
            )
        )

    def configureButtonBindingsClosedLoop(self) -> None:
        self.drive.setDefaultCommand(
            DriveCommands.arcadeDriveClosedLoop(self.drive, lambda: -self.controller.getLeftY(),
                                                lambda: -self.controller.getRightX(),
                                                lambda: 1.0 if (self.controller.getRightBumper()) else 0.25,
                                                lambda: False)
        )

    def configureButtonBindingsOpenLoop(self) -> None:
        self.drive.setDefaultCommand(
            DriveCommands.arcadeDriveOpenLoop(
                self.drive,
                lambda: -self.controller.getLeftY(),
                lambda: -self.controller.getRightX(),
                lambda: 1.0 if (self.controller.getRightBumper()) else 0.25,
            )
        )


    def getAutonomousCommand(self) -> Optional[Command]:
        return self.autoChooser.getSelected()

    def getTestCommand(self) -> Optional[Command]:
        return self.testChooser.getSelected()
