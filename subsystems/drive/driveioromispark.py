#TODO from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig
from subsystems.drive.driveio import DriveIO
from subsystems.drive import driveconstants
from wpilib import Spark,Encoder


from constants.math import kRadiansPerRevolution, kSecondsPerMinute
from util.sparkutil import tryUntilOk, isOk, isOkMulti
from math import pi
from subsystems.drive.driveconstants import kLeftInverted, kRightInverted, kWheelDiameterInch, kCountsPerRevolution


class DriveIORomiSpark(DriveIO):

    def __init__(self) -> None:
        # The Romi has the left and right motors set to
        # PWM channels 0 and 1 respectively
        self.leftLeader = Spark(0)
        self.leftLeader.setInverted(kLeftInverted)
        self.rightLeader = Spark(1)
        self.rightLeader.setInverted(kRightInverted)

        # The Romi has onboard encoders that are hardcoded
        # to use DIO pins 4/5 and 6/7 for the left and right
        self.leftEncoder = Encoder(4, 5)

        self.rightEncoder = Encoder(6, 7)

        self.resetEncoders()

        # Use inches as unit for encoder distances
        self.leftEncoder.setDistancePerPulse(
            (pi * kWheelDiameterInch) / kCountsPerRevolution
        )
        self.rightEncoder.setDistancePerPulse(
            (pi * kWheelDiameterInch) / kCountsPerRevolution
        )


    def updateInputs(self, inputs: DriveIO.DriveIOInputs) -> None:

        inputs.leftPositionCount = self.leftEncoder.get()
        inputs.rightPositionCount = self.rightEncoder.get()
        inputs.leftDriveDistanceInches = self.leftEncoder.getDistance()
        inputs.rightDriveDistanceInches = self.rightEncoder.getDistance()
        inputs.leftPositionRad = inputs.leftDriveDistanceInches * kRadiansPerRevolution / kWheelDiameterInch
        inputs.rightPositionRad = inputs.rightDriveDistanceInches * kRadiansPerRevolution / kWheelDiameterInch
        inputs.leftVelocityRadPerSec = self.leftEncoder.getRate() * kRadiansPerRevolution / kWheelDiameterInch
        inputs.rightVelocityRadPerSec = self.rightEncoder.getRate() * kRadiansPerRevolution / kWheelDiameterInch
        inputs.leftAppliedVolts = self.leftLeader.getVoltage()
        inputs.rightAppliedVolts = self.rightLeader.getVoltage()

    def setVoltage(self, leftVolts: float, rightVolts: float) -> None:
        self.leftSetVolts = leftVolts
        self.rightSetVolts = rightVolts
        self.leftLeader.setVoltage(leftVolts)
        self.rightLeader.setVoltage(rightVolts)

    def setVelocity(
        self,
        leftRadPerSec: float,
        rightRadPerSec: float,
        leftFFVolts: float,
        rightFFVolts: float,
    ) -> None:
        pass
        """
        self.leftController.setReference(
            leftRadPerSec,
            SparkMax.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            leftFFVolts,
        )
        self.rightController.setReference(
            rightRadPerSec,
            SparkMax.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            rightFFVolts,
        )
        """

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.reset()
        self.rightEncoder.reset()