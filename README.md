# 2024-Robot-Offseason

Source code for the 2024 FRC competition robot migrated from Java to Python (RobotPy)
(reference: https://github.com/frc2881/2024-Robot)

## Installation & Deployment
* Follow the official documentation for installing Python in your development environment (if Python 3.12 is not already installed): https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html
* For Visual Studio Code users:
  * Open this project in Visual Studio Code and install the Python extensions from Microsoft: https://marketplace.visualstudio.com/items?itemName=ms-python.python
  * Open the Command Palette (Ctrl/Command-Shift-P) -> `Python: Create Environment` to create a local Python virtual environment for the project using the installed Python 3.12 interpreter
  * Restart Visual Studio Code to reload the project in the Python virtual environment
* For PyCharm Community Edition users:
  * Open this project in PyCharm and follow the documented steps to create a local Python virtual environment found at: https://www.jetbrains.com/help/pycharm/creating-virtual-environment.html
    * Note: configure the virtual environment directory path with `.venv` as the root (as opposed to `venv`) to maintain compatibility between PyCharm and Visual Studio Code environments when needed
* After the Python virtual environment is created and ready in either Visual Studio Code or PyCharm, access and run the the install, build, and deployment tasks for the project as needed (Command Palette in Visual Studio Code and Run Configurations in PyCharm):
    * `RobotPy: Step 1 - Install / Upgrade RobotPy`
    * `RobotPy: Step 2 - Download & Sync RobotPy for roboRIO`
    * `RobotPy: Step 3 - Simulate Robot Code (Optional)`
    * `RobotPy: Step 4 - Deploy Robot Code`

## Project Notes & Status
* RobotPy API documentation and guides are available: https://robotpy.readthedocs.io/en/stable/index.html
* This code migration from the Java-based 2024 competition robot uses the same Command framework standard from WPILib while introducing updated patterns that are more appropriate for Python over Java (handling lambdas with Callable vs. Java generic Supplier, using enum and dataclass type classes in place of Java records)
* Pre-deployment tests are currently disabled as there is a known unresolved issue between the Python REV library and RobotPy where CANSparkMax/Flex instances are not cleaned up properly between test cycles
* For simulation mode, no physics for hardware / software models have been created for this project yet.