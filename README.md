# romiPyKitEmpty

This is forked from https://github.com/robotpy/examples/tree/main/RomiReference

This repo python source code based upon:
* [RobotPy](https://robotpy.github.io/docs/) - [RobotPy at readthedocs.io](https://robotpy.readthedocs.io/en/stable/) -
code for the [Romi]( https://github.com/robotpy/examples/tree/main/RomiReference).
* PyKit

## Installation to get started

From your operating system GUI install:
* PyCharm professional (not free) or community (free) 
* AdvantageScope https://docs.advantagescope.org/
* Elastic https://frc-elastic.gitbook.io/docs
* WPILIb https://github.com/wpilibsuite/allwpilib/releases

From a terminal window, install:
* uv

From your operating system GUI, start PyCharm

* PyCharm -> Hamburger Menu Icon -> File -> Project from Version Control
  * https://github.com/spiresfrc9106/romiPyKitEmpty 
* TODO more details here

From a terminal window:
* `cd` to the folder (also called a directory) where `romiPyKitEmpty` is on your computer
  * TODO more details on the above
* synchronize the python version and packages that the project uses:
  * perhaps: 
```
uv python list
```
  * perhaps:
```
uv python install 3.14
```
  * definitely:
```
uv sync
uv run -- robotpy sync
```
## Run the empty program

Start Elastic
Start AdvantageScope
From a terminal window:
* `cd` to the folder (also called a directory) where `romiPyKitEmpty` is on your computer
```
uv run -- robotpy sim
```

* Connect AdvantageScope to your sim
* Connect Elastic to your sim
