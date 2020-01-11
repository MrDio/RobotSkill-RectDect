# RobotSkill-RectDetect
Robot skill to detect storage spaces and move the robot to them

## Overview

| Service  | Control  | Web  |
|---|---|---|
| **Description**  | - Take pictures<br>- Detect storage spaces in pictures<br>- Move robot to storage space | - Publish REST interface to use robot skill  |
| **Service type**  | Control  | Web  |
| **Programming language**  | Python  | Python  |
| **External files,<br>external repositorys,<br>additional packages**  | - / - / requirements.txt  | - / - / -  |
| **Published APIs**  | rectDectApi  | -  |
| **Used APIs**  | cameraApi, robotApi  | rectDectApi  |

## Directory and file structure

| File name (with relative path)  | Description  |
|---|---|
| skill.yaml  | Description file of robot skill  |
| service_control  | Directory of control service  |
| service_control/main.py  | Entry point of control service  |
| service_control/requirements.txt  | File with required Python packages  |
| service_web  | Directory of web service  |
| service_web/rest_api.py  | Definition of the REST interface  |

**by Festo Developers**
