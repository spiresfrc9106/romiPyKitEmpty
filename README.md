# RomiReference

This is forked from https://github.com/robotpy/examples/tree/main/RomiReference

This repo python source code based upon [RobotPy](https://robotpy.github.io/docs/) - [RobotPy at readthedocs.io](https://robotpy.readthedocs.io/en/stable/) -
code for the [Romi]( https://github.com/robotpy/examples/tree/main/RomiReference).

Robotpy add a Python programming environment to the [WPILib](https://docs.wpilib.org/en/stable/) software
development environment for [FRC](https://www.firstinspires.org/robotics/frc) robots, for Python development. Prior to robotpy, FRC
robots were programmed in C++ or Java.

## TL;DR

For those in the "too long; didn't read" camp, here are the steps to get Python program, using RobotPy and WPILib,
controlling a Romi.  These instructions are for a Windows PC, but similar steps should work on a Linux computer or
a Mac.


### Steps

* [Download and install wpilib](https://github.com/wpilibsuite/allwpilib/releases) - TODO need to verify this step is needed with RobotPy
* Follow **some** of the steps in https://docs.wpilib.org/en/stable/docs/Romi-robot/hardware-and-imaging.html and https://Romiusersguide.readthedocs.io/en/latest/course/building.html
  * Note 1: If you follow the complete video, it uses [Romi Code from WPI (https://Romicode.wpi.edu/)](https://Romicode.wpi.edu/) 
    to test out the robot build. Which is a different development environment than
    the [FRC Control System](https://docs.wpilib.org/en/stable/) development environment firmware that these steps
    load into the Romi.
  * Note 2: Follow the [steps](https://docs.wpilib.org/en/stable/docs/romi-robot/index.html):
    * Getting Started with Romi
      * Romi Hardware, Assembly and Imaging
      * Getting to know your Romi
      * Romi Hardware Support
      * The Romi Web UI
      * But stop before "Programming the Romi"
* Make sure your computer is reconnected to the internet and not your Romi
* [Download and install Python 3.12](https://www.python.org/downloads/release/python-3122/)
* Download this repo from https://github.com/spiresfrc9106/romiReferencePython
  * If you downloaded as a zip file, unzip into a directory
    * Open a Windows PowerShell Window
      * Because RobotPy uses the `pyproject.toml` that is in `romiReferencePython` the Change directory (cd) to the directory where you downloaded and perhaps unzip `romiReferencePython`:
         ```commandline
         cd C:\Users\MikeStitt\Downloads\first\sw\romiReferencePython
         ```

         ```commandline
         pwd
         ```
    
         results in:

        ```commandline
         Path
         ----
         C:\Users\MikeStitt\Downloads\first\sw\romiReferencePython 
         ```
         for reference, a dir command
         ```commandline
         dir
         ```

         results in something like this:

         ```commandline

         Directory: C:\Users\MikeStitt\Downloads\first\sw\romiReferencePython

              Mode                 LastWriteTime         Length Name
              ----                 -------------         ------ ----
              d-----          1/7/2026   4:12 PM                .idea
              d-----          1/7/2026   4:57 PM                .simulationLogs
              d-----          1/7/2026   4:44 PM                .venv
              d-----          1/7/2026   4:12 PM                tests
              d-----          1/7/2026   4:35 PM                utils
              d-----          1/7/2026   4:24 PM                __pycache__
              -a----          1/7/2026   4:12 PM             98 .gitignore
              -a----          1/7/2026   4:12 PM          22151 .pylintrc
              -a----          1/7/2026   4:35 PM              4 networktables.json
              -a----          1/7/2026   4:46 PM           1452 pyproject.toml
              -a----          1/7/2026   5:06 PM          16892 README.md
              -a----          1/7/2026   4:12 PM          10806 robot.py
              -a----          1/7/2026   4:58 PM           1612 simgui-ds.json
              -a----          1/7/2026   4:58 PM           1347 simgui-window.json
              -a----          1/7/2026   4:58 PM            743 simgui.json
              -a----          1/7/2026   4:35 PM         239608 uv.lock
      ```
      * Verify that your Python command is python 
         ```commandline
         python -VV
         Python 3.12.1 (tags/v3.12.1:2305ca5, Dec  7 2023, 22:03:25) [MSC v.1937 64 bit (AMD64)]
         ```
      * Install uv
        * See https://docs.astral.sh/uv/getting-started/installation/
      * Use uv to install required python packages
        ```commandline
        uv sync
        ```
  * Connect a **wired-only USB** xbox controller to your PC. This can be confusing because some xbox controllers have
    a USB connector for charging, but are **wireless** xbox controller. These will not work. Logitech xbox controllers
    will work, if the switch on the back of them are switched to the "X" mode using the  "X" or "D" switch on the back
    of the controller.
  * Powerup the Romi
  * Connect to the Romi WiFi network
    * The network SSID is something like `SSID WPILibPi-<number>` (where `<number>` is based on the Raspberry Pi serial number) with the WPA2 passphrase `WPILib2021!`.
  * TODO add the steps the first time you power-up your Romi using `http://10.0.0.2/`
  * Go back to your Windows PowerShell Window.
    * This step can be confusing for those that know how to simulate and deploy to full-sized FRC robots. Use RobotPy to
      launch the WPILib simulator to control the Romi from the Windows PowerShell that is in the `romiReferencePython` directory:
      ```commandline
      uv run -- robotpy sim --ws-client
      ```
      resulting in:
      ```commandline
      17:13:22:640 INFO    : halsim_gui          : WPILib HAL Simulation 2025.3.2.2
      HAL Extensions: Attempting to load: halsim_gui
      Simulator GUI Initializing.
      Simulator GUI Initialized!
      HAL Extensions: Successfully loaded extension
      17:13:22:743 INFO    : halsim_ws.client    : WPILib HAL Simulation websim client 2025.3.2.2
      HAL Extensions: Attempting to load: halsim_ws_client
      HALSim WS Client Extension Initializing
      HALSimWS Initialized
      No WS Message Filters specifiedWill attempt to connect to ws://10.0.0.2:3300/wpilibws
      Connection Attempt 1
      HALSim WS Client Extension Initialized
      HAL Extensions: Successfully loaded extension
      17:13:22:746 WARNING : pyfrc.physics       : Cannot enable physics support, C:\spires\romireftiny\physics.py not found
      17:13:22:748 INFO    : wpilib              : RobotPy version 2025.3.2.2
      17:13:22:748 INFO    : wpilib              : WPILib version 2025.3.2.2
      17:13:22:748 INFO    : wpilib              : Running with simulated HAL.
      17:13:22:751 INFO    : nt                  : Listening on NT3 port 1735, NT4 port 5810
      17:13:22:761 INFO    : nt                  : Got a NT4 connection from 127.0.0.1 port 64293
      Not loading CameraServerShared
      roborio SerialNumber=
      
      ********** Robot program startup complete **********
      17:13:22:769 INFO    : nt                  : Got a NT4 connection from 127.0.0.1 port 64300
      17:13:22:770 INFO    : nt                  : CONNECTED NT4 client 'AdvantageScope@1' (from 127.0.0.1:64300)
      Default frc::IterativeRobotBase::SimulationPeriodic() method... Override me!
      DataLog: Logging to './.simulationLogs\FRC_TBD_57fe5827e9ffbedc.wpilog' (772.5 GiB free space)
      HALSimWS: WebSocket Connected
      17:13:22:792 INFO    : nt                  : Got a NT4 connection from 127.0.0.1 port 64301
      17:13:22:792 INFO    : nt                  : CONNECTED RTT client (from 127.0.0.1:64301)
      17:13:22:823 INFO    : nt                  : Got a NT4 connection from 127.0.0.1 port 64297
      Button A Released
      17:13:22:917 INFO    : nt                  : Got a NT4 connection from 127.0.0.1 port 64292
      17:13:22:980 INFO    : nt                  : Got a NT4 connection from 127.0.0.1 port 64296
      17:13:23:134 INFO    : nt                  : Got a NT4 connection from 127.0.0.1 port 64295
      17:13:23:180 INFO    : nt                  : Got a NT4 connection from 127.0.0.1 port 64298
      17:13:23:698 INFO    : nt                  : Got a NT4 connection from 10.0.0.2 port 44938
      17:13:23:700 INFO    : nt                  : CONNECTED NT4 client 'multiCameraServer@2' (from 10.0.0.2:44938)
      ```
      and a "Robot Simulation" window appearing. (TODO add an image of the "Robot Simulation" window.)
  * In the "Robot Simulation", from the "System Joysticks" window, drag and drop "0: Xbox Controller" to the "Joysticks" window, "Joystick[0]" column header.
  * In the "Robot Simulation", in the "Robot Stat" window, select "Teleoperated".
  * Drive your Romi around with your xbox controller joysticks. TODO, on a Logitech controller the rotate left and right is backwards.

# Notes from Spires2023 and RobotCasserole

## Docs

- [High level code design patterns](.docs/designPatterns)
- [Commonly used modules in this repository](.docs/commonModules)
- [Most recent relationship diagram between classes](.docs/graph.md)
    - Keep this file up to date by periodically running `codeStructureReportGen/reportGen.py`


## The robot website

On a simulator
- http://localhost:5805/

On a RoboRIO
- http://10.91.6.2:5805/
- Be sure that you are connected to the RoboRIO's network via WiFi or Ethernet

## Interesting links

[RobotPy source code](https://github.com/robotpy/mostrobotpy)

[PyTest Docs](https://docs.pytest.org/en/7.4.x/)

[PyTest Examples](https://pytest.org/en/7.4.x/example/index.html)

## Deploying to the Robot

Use the `WPILib: Deploy Robot Code` task in VSCode's command palette.

OR

`robotpy deploy` will deploy all code to the robot. Be sure to be on the robot's network via WiFi or Ethernet.

### Deploy Notes

`robotpy deploy --skip-tests` to avoid requiring tests to pass before deployment can proceed. This is helpful for quick iterations, but don't make it a bad habit.

`.deploy_cfg` contains specific configuration about the deploy process.

Any folder or file prefixed with a `.` will be skipped in the deploy. This is good to avoid sending unnecessary files to the resource limited RoboRIO like documentation and images.

## Linting

"Linting" is the process of checking our code format and style to keep it looking nice and avoid unnecessary inconsistencies.

`pylint --rcfile=.pylintrc **\*.py`

`.pylintrc` contains configuration about what checks the linter runs, and what formatting it enforces

## Testing

Run the `WPILib: Test Robot Code` task in VSCode's command palette.

OR

`robotpy test`

## Simulating

Run the `WPILib: Simulate Robot Code` task in VSCode's command palette.

OR

`robotpy sim`

## RIO First-time Installation

Follow [the robotpy instructions for setting up the RIO](https://robotpy.readthedocs.io/en/stable/install/robot.html)

Then, install all packages specific to our repo, from `requirements_dev.txt`, following the
[two step process for roboRIO package installer](https://robotpy.readthedocs.io/en/stable/install/packages.html)

While on the internet:

`python -m robotpy_installer download -r requirements_dev.txt`

Then, while connected to the robot's network:

```cmd
python -m robotpy_installer install-python
python -m robotpy_installer install robotpy
python -m robotpy_installer list
```

To check what is installed:

`python -m robotpy_installer list`

example output:
```cmd
10:32:35:371 INFO    : robotpy.installer   : RobotPy Installer 2023.0.4
10:32:35:371 INFO    : robotpy.installer   : -> caching files at C:\Users\MikeStitt\wpilib\2023\robotpy
10:32:35:372 INFO    : robotpy.installer   : -> using existing config at 'C:\Users\MikeStitt\Documents\first\sw\spiresFrc9106\firstRoboPy\.installer_config'
10:32:35:374 INFO    : robotpy.installer   : Finding robot for team 9106
10:32:35:380 INFO    : robotpy.installer   : -> Robot is at 172.22.11.2
10:32:35:380 INFO    : robotpy.installer   : Connecting to robot via SSH at 172.22.11.2
10:32:35:495 INFO    : paramiko.transport  : Connected (version 2.0, client OpenSSH_8.3)
10:32:35:599 INFO    : paramiko.transport  : Auth banner: b'NI Linux Real-Time (run mode)\n\nLog in with your NI-Auth credentials.\n\n'
10:32:35:600 INFO    : paramiko.transport  : Authentication (password) successful!
10:32:35:676 INFO    : robotpy.installer   : -> RoboRIO 2 image version: 2023_v3.2
10:32:35:752 INFO    : robotpy.installer   : -> RoboRIO disk usage 584.0M/3.3G (17% full)
Package                   Version
------------------------- ----------
debugpy                   1.8.0
numpy                     1.24.2
pip                       22.3.1
pyntcore                  2023.4.3.0
robotpy                   2023.4.3.1
robotpy-apriltag          2023.4.3.0
robotpy-commands-v2       2023.4.3.0
robotpy-cscore            2023.4.3.0
robotpy-ctre              2023.1.0
robotpy-hal               2023.4.3.0
robotpy-libgfortran5      12.1.0+r5
robotpy-navx              2023.0.3
robotpy-openblas          0.3.21+r2
robotpy-opencv            4.6.0+r2
robotpy-opencv-core       4.6.0+r2
robotpy-pathplannerlib    2023.3.4.1
robotpy-photonvision      2023.4.2
robotpy-playingwithfusion 2023.1.0
robotpy-rev               2023.1.3.2
robotpy-wpilib-utilities  2023.1.0
robotpy-wpimath           2023.4.3.0
robotpy-wpinet            2023.4.3.0
robotpy-wpiutil           2023.4.3.0
setuptools                65.5.0
wpilib                    2023.4.3.0
```

## Dependency Management

`requirements_dev.txt` is used to list dependencies that we only need on our development computers, not on the RoboRIO. This is tools like linting and formatting; they aren't mission critical (or even used) at runtime, but still very helpful to have.

`pyproject.toml` defines the dependencies that are required to be installed on the RoboRIO for our code to successfully run. For tests and simulations you will also need these dependencies on your development machine. They are managed automatically via `robotpy sync`.

## Useful commands:

See network logs: `python -m netconsole roboRIO-9106-frc.local`

SSH into the robot: `ssh lvuser@roboRIO-9106-frc.local`

## Roborio 2.0 image install

Use balenaEtcher to install the roborio image

The 2023 roborio 2.0 image is here:

C:\Program Files (x86)\National Instruments\LabVIEW 2020\project\roboRIO Tool\FRC Images\SD Images

The 2024 roboio 2.0 image is here:

C:\Program Files (x86)\National Instruments\LabVIEW 2023\project\roboRIO Tool\FRC Images\SD Images

use roborio team number setter to set the team number


# Other notes

Todo add a link to https://docs.wpilib.org/en/stable/docs/romi-robot/index.html

https://github.com/wpilibsuite/WPILibPi

https://github.com/RPi-Distro/pi-gen

so far this has not worked: https://raspberrypihq.com/how-to-add-wifi-to-the-raspberry-pi/


To install photon vision, connect pi over ethernet, `ssh pi@wpilibpi.local`, first follow these instructions:

https://docs.photonvision.org/en/latest/docs/installation/sw_install/romi.html

Then these instructions:

https://docs.photonvision.org/en/latest/docs/installation/sw_install/other-coprocessors.html

Then use web browser at `pilibpi.local` to set filesystem to be writeable.

Then `sudo systemctl restart photonvision.service`

Then the this `http://10.0.0.2:5800/#/dashboard` is available.  But see: `https://www.chiefdelphi.com/t/romi-photonvision-startup-errors/452444/7` for the problem we are having

Perhaps this pull request: `https://github.com/wpilibsuite/WPILibPi/pull/233`

It might be good to compare this working Romi tag: `https://github.com/wpilibsuite/WPILibPi/releases/tag/v2023.2.1` to the latest
and to the pull request.

```commandline
http://10.0.0.2/
```

```commandline
ssh pi@wpilibpi.local
```

or

```commandline
ssh pi@10.0.0.2
```

password: `raspberry`

```commandline
cat /etc/os-release
```

# useful commands when learning PyKit

```
uv run -- robotpy sim
uv run -- robotpy sim --ws-client
dir pyLogs
cp .\pyLogs\pykit_1769.wpilog pykit.log
$env:LOG_PATH='C:\spires\romireftiny\pykit.log'
uv run -- robotpy watch
Remove-Item Env:LOG_PATH
```