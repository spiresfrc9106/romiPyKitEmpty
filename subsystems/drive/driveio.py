from dataclasses import dataclass, field
from pykit.autolog import autolog


class DriveIO:
    @autolog
    @dataclass
    class DriveIOInputs:
        leftPositionCount: int  = 0
        leftPositionRad: float = 0.0
        leftVelocityRadPerSec: float = 0.0
        leftDriveDistanceInches: float = 0.0
        leftSetVolts: float = 0.0
        leftAppliedVolts: float = 0.0
        leftCurrentAmps: list[float] = field(default_factory=list)

        rightPositionCount: int = 0
        rightPositionRad: float = 0.0
        rightVelocityRadPerSec: float = 0.0
        rightDriveDistanceInches: float = 0.0
        rightSetVolts: float = 0.0
        rightAppliedVolts: float = 0.0
        rightCurrentAmps: list[float] = field(default_factory=list)

    def updateInputs(self, inputs: DriveIOInputs) -> None:
        pass

    def setVoltage(self, leftVolts: float, rightVolts: float) -> None:
        pass

    def setVelocity(
        self,
        leftRadPerSec: float,
        rightRadPerSec: float,
        leftFFVolts: float,
        rightFFVolts: float,
    ) -> None:
        pass
