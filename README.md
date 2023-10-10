# Machine-Learning and Control Theory integration

![Production Status](https://img.shields.io/badge/Production%20State-In%20Progress-orange)

## Description
This repository contains the integrated control system code for controlling a Dynamixel motor, 
interfacing with a FLIR camera, and implementing a port-Hamiltonian machine learning model. 
The machine learning model utilizes feedback from the FLIR camera and dispatches output commands to the Dynamixel motor, guiding 
it to act accordingly. The synchronization and integration of these components are achieved using ROS (Robot Operating System). This is accomplished with the help
of two sub repositories linked below.


## Sub-Repositories Included
1. [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK.git)
2. [FLIR Camera Driver](https://github.com/ros-drivers/flir_camera_driver.git)


## License & Copyright
Except where otherwise noted within sub-repositories, the code within this repository is the property of the Thomas Beckers Research Lab at Vanderbilt University. 
Unauthorized distribution, modification, or commercial use without proper attribution is prohibited.

For more details, visit our [official website](https://www.tbeckers.com/).

---

© 2023 Thomas Beckers Research Lab, Vanderbilt University. All Rights Reserved.

