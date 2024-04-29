# 2024-Robot-Offseason

Source code for the 2024 FRC competition robot migrated from Java to Python (RobotPy)
(reference: https://github.com/frc2881/2024-Robot)

## Installation & Deployment
* Follow the official documentation for installing Python in your development environment (if Python 3.12 is not already installed): https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html
* Open this project in VSCode and install the Python extension (including extension pack items Pylance and Python Debugger): https://marketplace.visualstudio.com/items?itemName=ms-python.python
* Open the VSCode Command Palette (Ctrl/Command-Shift-P) -> `Tasks: Run Task`  to access the configured user tasks for installing/upgrading RobotPy, downloading (syncing) for the roboRIO, running the code in simulation mode, and most importantly deploying to the live robot

## Notes
* RobotPy API documentation and guides are available: https://robotpy.readthedocs.io/en/stable/index.html
* This code migration from the Java-based 2024 competition robot uses the same Command framework standard from WPILib while introducing updated patterns that are more appropriate for Python over Java (handling lambdas with Callable vs. Java generic Supplier, using enum and dataclass type classes in place of Java records)
* Pre-deployment tests are currently disabled as there is a known unresolved issue between the Python REV library and RobotPy where CANSparkMax/Flex instances are not cleaned up properly between test cycles
* For simulation mode, no physics for hardware / software models have been created for this project yet.