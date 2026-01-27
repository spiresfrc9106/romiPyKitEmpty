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
            driveconstants.kWheelRadiusM,
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
                    self.sim.getLeftVelocity() / driveconstants.kWheelRadiusM,
                )
                + self.leftFFVolts
            )
            self.rightAppliedVolts = (
                self.rightPID.calculate(
                    self.sim.getRightVelocity() / driveconstants.kWheelRadiusM,
                )
                + self.rightFFVolts
            )

        self.sim.setInputs(
            clamp(self.leftAppliedVolts, -12.0, 12.0),
            clamp(self.rightAppliedVolts, -12.0, 12.0),
        )
        self.sim.update(kRobotPeriod)

        inputs.leftPositionRad = (
            self.sim.getLeftPosition() / driveconstants.kWheelRadiusM
        )
        inputs.leftPositionCount = int(inputs.leftPositionRad / 2.0 / pi)
        inputs.leftVelocityRadPerSec = (
            self.sim.getLeftVelocity() / driveconstants.kWheelRadiusM
        )
        inputs.leftDriveDistanceInches = self.sim.getLeftPosition() / kMetersPerInch;
        inputs.leftAppliedVolts = self.leftAppliedVolts
        inputs.leftCurrentAmps = [self.sim.getLeftCurrentDraw()]

        inputs.rightPositionRad = (
            self.sim.getRightPosition() / driveconstants.kWheelRadiusM
        )
        inputs.rightPositionCount = int(inputs.rightPositionRad / 2.0 / pi)
        inputs.rightVelocityRadPerSec = (
            self.sim.getRightVelocity() / driveconstants.kWheelRadiusM
        )
        inputs.rightDriveDistanceInches = self.sim.getRightPosition() / kMetersPerInch;
        inputs.rightAppliedVolts = self.rightAppliedVolts
        inputs.rightCurrentAmps = [self.sim.getRightCurrentDraw()]

    def setVoltage(self, leftVolts: float, rightVolts: float) -> None:
        self.closedLoop = False
        self.leftSetVolts = leftVolts
        self.rightSetVolts = rightVolts
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