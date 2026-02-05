import os
from constants import kRobotMode, RobotModes
from pykit.loggedrobot import LoggedRobot
from pykit.logger import Logger
from pykit.wpilog.wpilogwriter import WPILOGWriter
from pykit.wpilog.wpilogreader import WPILOGReader
from pykit.networktables.nt4Publisher import NT4Publisher
from wpilib.deployinfo import getDeployData


def startLogger(robot: LoggedRobot) -> None:
    Logger.recordMetadata("Robot", type(robot).__name__)
    match kRobotMode:
        case RobotModes.REAL | RobotModes.SIMULATION:
            deploy_config = getDeployData()
            if deploy_config is not None:
                Logger.recordMetadata(
                    "Deploy Host", deploy_config.get("deploy-host", "")
                )
                Logger.recordMetadata(
                    "Deploy User", deploy_config.get("deploy-user", "")
                )
                Logger.recordMetadata(
                    "Deploy Date", deploy_config.get("deploy-date", "")
                )
                Logger.recordMetadata(
                    "Code Path", deploy_config.get("code-path", "")
                )
                Logger.recordMetadata("Git Hash", deploy_config.get("git-hash", ""))
                Logger.recordMetadata(
                    "Git Branch", deploy_config.get("git-branch", "")
                )
                Logger.recordMetadata(
                    "Git Description", deploy_config.get("git-desc", "")
                )
            Logger.addDataReciever(NT4Publisher(True))
            Logger.addDataReciever(WPILOGWriter())
        case RobotModes.REPLAY:
            robot.useTiming = (
                False  # Disable timing in replay mode, run as fast as possible
            )
            log_path = os.environ["LOG_PATH"]
            log_path = os.path.abspath(log_path)
            print(f"Starting log from {log_path}")
            Logger.setReplaySource(WPILOGReader(log_path))
            Logger.addDataReciever(WPILOGWriter(log_path[:-7] + "_sim.wpilog"))
    Logger.start()
