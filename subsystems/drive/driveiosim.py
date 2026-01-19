from math import pi
from wpilib.simulation import DifferentialDrivetrainSim
from wpimath.controller import PIDController
from subsystems.drive.driveio import DriveIO
from subsystems.drive import driveconstants

from util.helpfulmath import clamp
from constants import kRobotPeriod
from constants.math import kMetersPerInch

class DriveIOSim(DriveIO):
    def __init__(self) -> None:
        self.sim = DifferentialDrivetrainSim.createKitbotSim(
            driveconstants.kGearbox,
            driveconstants.kMotorReduction,
            driveconstants.kWheelRadius,
        )

        self.leftAppliedVolts = 0.0
        self.rightAppliedVolts = 0.0
        self.closedLoop = False

        self.leftPID = PIDController(driveconstants.kSimKp, 0.0, driveconstants.kSimKd)
        self.rightPID = PIDController(driveconstants.kSimKp, 0.0, driveconstants.kSimKd)

        self.leftFFVolts = 0.0
        self.rightFFVolts = 0.0

    def updateInputs(self, inputs: DriveIO.DriveIOInputs) -> None:
        if self.closedLoop:
            self.leftAppliedVolts = (
                self.leftPID.calculate(
                    self.sim.getLeftVelocity() / driveconstants.kWheelRadius,
                )
                + self.leftFFVolts
            )
            self.rightAppliedVolts = (
                self.rightPID.calculate(
                    self.sim.getRightVelocity() / driveconstants.kWheelRadius,
                )
                + self.rightFFVolts
            )

        self.sim.setInputs(
            clamp(self.leftAppliedVolts, -12.0, 12.0),
            clamp(self.rightAppliedVolts, -12.0, 12.0),
        )
        self.sim.update(kRobotPeriod)

        inputs.leftPositionRad = (
            self.sim.getLeftPosition() / driveconstants.kWheelRadius
        )
        inputs.leftPositionCount = int(inputs.leftPositionRad / 2.0 / pi)
        inputs.leftVelocityRadPerSec = (
            self.sim.getLeftVelocity() / driveconstants.kWheelRadius
        )
        inputs.leftDriveDistanceInches = self.sim.getLeftPosition() / kMetersPerInch;
        inputs.leftAppliedVolts = self.leftAppliedVolts
        inputs.leftCurrentAmps = [self.sim.getLeftCurrentDraw()]

        inputs.rightPositionRad = (
            self.sim.getRightPosition() / driveconstants.kWheelRadius
        )
        inputs.rightPositionCount = int(inputs.rightPositionRad / 2.0 / pi)
        inputs.rightVelocityRadPerSec = (
            self.sim.getRightVelocity() / driveconstants.kWheelRadius
        )
        inputs.rightDriveDistanceInches = self.sim.getRightPosition() / kMetersPerInch;
        inputs.rightAppliedVolts = self.rightAppliedVolts
        inputs.rightCurrentAmps = [self.sim.getRightCurrentDraw()]

    def setVoltage(self, leftVolts: float, rightVolts: float) -> None:
        self.closedLoop = False
        self.leftAppliedVolts = leftVolts
        self.rightAppliedVolts = rightVolts

    def setVelocity(
        self,
        leftRadPerSec: float,
        rightRadPerSec: float,
        leftFFVolts: float,
        rightFFVolts: float,
    ) -> None:
        self.closedLoop = True
        self.leftFFVolts = leftFFVolts
        self.rightFFVolts = rightFFVolts
        self.leftPID.setSetpoint(leftRadPerSec)
        self.rightPID.setSetpoint(rightRadPerSec)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        pass # TODO