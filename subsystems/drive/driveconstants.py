from enum import Enum
from wpimath.system.plant import DCMotor
from pathplannerlib.config import ModuleConfig, RobotConfig
from constants.math import kMetersPerInch

class DriveBaseVersion(Enum):
    ROMI = 1
    XPR = 2
    KITBOT2025 = 3
    KITBOT2026 = 4

driveBaseVersion = DriveBaseVersion.ROMI

match driveBaseVersion:
    case DriveBaseVersion.ROMI:
        kMaxSpeedMetersPerSecond = 4.0
        kTrackWidthM = 5.5 * kMetersPerInch

        kPigeonCanId = 9
        kLeftLeaderCanId = 1
        kLeftFollowerCanId = 2
        kRightLeaderCanId = 3
        kRightFollowerCanId = 4

        kCurrentLimit = 60.0
        kCountsPerRevolution = 1440.0
        kWheelDiameterInch = 2.75591
        kWheelRadiusM = (kWheelDiameterInch / 2.0) * kMetersPerInch
        kMotorReduction = 10.71 # TODO is this correct for ROMI?
        kLeftInverted = False
        kRightInverted = True
        kGearbox = DCMotor.CIM(2)

        kRealKp = 0.0
        kRealKd = 0.0
        kRealKs = 0.0
        kRealKv = 0.1

        kSimKp = 0.0
        kSimKd = 0.0
        kSimKs = 0.0
        kSimKv = 0.227

        kRobotMassKg = 74.088
        kRobotMOI = 6.883
        kWheelCOF = 1.2

    case DriveBaseVersion.XPR:
        pass

    case DriveBaseVersion.KITBOT2025:
        kMaxSpeedMetersPerSecond = 4.0
        kTrackWidthM = 26 * kMetersPerInch

        kPigeonCanId = 9
        kLeftLeaderCanId = 1
        kLeftFollowerCanId = 2
        kRightLeaderCanId = 3
        kRightFollowerCanId = 4

        kCurrentLimit = 60.0
        kCountsPerRevolution = 1440.0  # romi TODO do we need this for KITBOT2025
        kWheelDiameterInch = 6
        kWheelRadiusM = (kWheelDiameterInch / 2.0) * kMetersPerInch
        kMotorReduction = 10.71
        kLeftInverted = False
        kRightInverted = True
        kGearbox = DCMotor.CIM(2)

        kRealKp = 0.0
        kRealKd = 0.0
        kRealKs = 0.0
        kRealKv = 0.1

        kSimKp = 0.0
        kSimKd = 0.0
        kSimKs = 0.0
        kSimKv = 0.227

        kRobotMassKg = 74.088
        kRobotMOI = 6.883
        kWheelCOF = 1.2


kPPConfig = RobotConfig(
    kRobotMassKg,
    kRobotMOI,
    ModuleConfig(
        kWheelRadiusM,
        kMaxSpeedMetersPerSecond,
        kWheelCOF,
        kGearbox.withReduction(kMotorReduction),
        kCurrentLimit,
        2,
    ),
    [],
    kTrackWidthM,
)
