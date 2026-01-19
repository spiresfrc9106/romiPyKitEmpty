#TODO from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig
from subsystems.drive.driveio import DriveIO
from subsystems.drive import driveconstants
from wpilib import Spark,Encoder


from constants.math import kRadiansPerRevolution, kSecondsPerMinute
from util.sparkutil import tryUntilOk, isOk, isOkMulti
from math import pi
from subsystems.drive.driveconstants import kWheelDiameterInch, kCountsPerRevolution


class DriveIORomiSpark(DriveIO):

    def __init__(self) -> None:
        # The Romi has the left and right motors set to
        # PWM channels 0 and 1 respectively
        self.leftLeader = Spark(0)
        self.leftLeader.setInverted(True)
        self.rightLeader =Spark(1)
        self.rightLeader.setInverted(False)

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
        """
        TODO
        isOk(
            self.leftLeader,
            self.leftEncoder.getPosition,
            lambda v: setattr(inputs, "leftPositionRad", v),
        )
        """
        inputs.leftPositionCount = self.leftEncoder.get()
        inputs.rightPositionCount = self.rightEncoder.get()
        inputs.leftDriveDistanceInches = -self.leftEncoder.getDistance()
        inputs.rightDriveDistanceInches = -self.rightEncoder.getDistance()

        """
        isOk(
            self.leftLeader,
            self.leftEncoder.getVelocity,
            lambda v: setattr(inputs, "leftVelocityRadPerSec", v),
        )
        isOk(
            self.leftLeader,
            lambda : -self.leftEncoder.getDistance(),
            lambda v: setattr(inputs, "leftDriveDistanceInches", v),
        )
        isOkMulti(
            self.leftLeader,
            [self.leftLeader.getAppliedOutput, self.leftLeader.getBusVoltage],
            lambda multiResults: setattr(
                inputs,
                "leftAppliedVolts",
                multiResults[0] * multiResults[1],
            ),
        )
        isOkMulti(
            self.leftLeader,
            [self.leftLeader.getOutputCurrent],
            lambda multiResults: setattr(
                inputs,
                "leftCurrentAmps",
                [multiResults[0]],
            ),
        )
        """
        """
        TODO
        isOk(
            self.rightLeader,
            self.rightEncoder.getPosition,
            lambda v: setattr(inputs, "rightPositionRad", v),
        )

        isOk(
            self.rightLeader,
            self.rightEncoder.get,
            lambda v: setattr(inputs, "rightPositionCount", v),
        )
        isOk(
            self.rightLeader,
            self.rightEncoder.getVelocity,
            lambda v: setattr(inputs, "rightVelocityRadPerSec", v),
        )
        isOk(
            self.rightLeader,
            lambda : -self.righttEncoder.getDistance(),
            lambda v: setattr(inputs, "rightDriveDistanceInches", v),
        )
        isOkMulti(
            self.rightLeader,
            [self.rightLeader.getAppliedOutput, self.rightLeader.getBusVoltage],
            lambda multiResults: setattr(
                inputs,
                "rightAppliedVolts",
                multiResults[0] * multiResults[1],
            ),
        )
        isOkMulti(
            self.rightLeader,
            [self.rightLeader.getOutputCurrent, self.rightFollower.getOutputCurrent],
            lambda multiResults: setattr(
                inputs,
                "rightCurrentAmps",
                [multiResults[0], multiResults[1]],
            ),
        )
        """

    def setVoltage(self, leftVolts: float, rightVolts: float) -> None:
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