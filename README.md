# RobotSkill-RectDetect
Robot skill algorithm to detect storage spaces and move the robot to them

## Purpose

This repository is not ready to use as open source repo. It requires a special Cloud and Edge infrastructure which is not open.
for interest or any concerns feel free to contact me (MrDio).

![alt text](https://i.stack.imgur.com/5e2Mv.jpg)

## Overview

| Service  | Control  | Web  |
|---|---|---|
| **Description**  | - Take pictures<br>- Detect storage spaces in pictures<br>- Move robot to storage space | - Publish REST interface to use robot skill  |
| **Service type**  | Control  | Web  |
| **Programming language**  | Python  | Python  |
| **External files,<br>external repositorys,<br>additional packages**  | - / - / requirements.txt  | - / - / -  |
| **Published APIs**  | rectDetectApi  | -  |
| **Used APIs**  | cameraApi, robotApi  | rectDetectApi  |

## Directory and file structure

| File name (with relative path)  | Description  |
|---|---|
| skill.yaml  | Description file of robot skill  |
| service_control  | Directory of control service  |
| service_control/main.py  | Entry point of control service  |
| service_control/requirements.txt  | File with required Python packages  |
| service_web  | Directory of web service  |
| service_web/rest_api.py  | Definition of the REST interface  |

## Credits
Janfabian Fabriczeck

Firat Susan

Parts of the Algorithm and image derived from [here](https://stackoverflow.com/questions/8667818/opencv-c-obj-c-detecting-a-sheet-of-paper-square-detection/8863060)
