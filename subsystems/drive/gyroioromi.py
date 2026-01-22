from wpimath.geometry import Rotation2d
from subsystems.drive.gyroio import GyroIO
from navx import AHRS
from romi import RomiGyro

from constants.math import kRadiansPerDegree


class GyroIORomi(GyroIO):
    def __init__(self) -> None:
        self.romiGyro = RomiGyro()

    def updateInputs(self, inputs: GyroIO.GyroIOInputs) -> None:
        inputs.connected = True
        inputs.yawPositionDeg = -self.romiGyro.getAngle() / kRadiansPerDegree
        inputs.yawVelocityDegPerSec = -self.romiGyro.getRate() / kRadiansPerDegree
        inputs.yawPosition = Rotation2d.fromDegrees(inputs.yawPositionDeg)

