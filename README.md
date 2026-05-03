# ME571 - Intro to Robotics Technology

Welcome to ME571 - Intro to Robotics Technology! This repository contains the template code, setup files, and resources needed for your labs involving the Turtlebot3 Waffle with openManipulator.

## Table of Contents
- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Getting Started](#getting-started)
- [Repository Structure](#repository-structure)
- [Get Visulization](#get-visulization)
- [Turtlebot Instructions](#turtlebot-instructions)
- [Connecting to the Turtlebot](#connecting-to-the-turtlebot)
- [Happy Coding](#happy-coding)
---

## Introduction

In this course, you will get hands-on experience with the Turtlebot3 Waffle and openManipulator. This README provides guidance on setting up your development environment, accessing the template code, and getting started with the labs.

## Prerequisites

Before you begin, ensure you have the following installed:
- [Git](https://git-scm.com/)
- [Docker](https://www.docker.com/)
- [Visual Studio Code (VSCode)](https://code.visualstudio.com/)
- [Remote - Containers extension for VSCode](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

## Getting Started

1. **Clone the Repository**: Open your terminal and run the following command to clone this repository

2. **Open Repository in VSCode using DevContainer**:
    - Navigate to the cloned repository directory
    - Open VSCode and ensure you have the Remote - Containers extension installed.
    - In the bottom-left corner of the VSCode window, click on the green icon that says "Open a Remote Window".
    - Select "Remote-Containers: Reopen in Container" from the dropdown. This will use the `devcontainer.json` configuration file in the repository to set up the development environment inside a Docker container.

## Repository Structure

- `lab1/`: Contains template code and resources for Lab 1.
- `lab2/`: Contains template code and resources for Lab 2.
- `lab3/`: Contains template code and resources for Lab 3.
- `lab4/`: Contains template code and resources for LEGACY Lab 4.
- `searching_map_hw/`: Contains template code and resources Homework 3
- `dockerfile`: Dockerfile for setting up the development environment.
- `devcontainer.json`: Configuration file for the development container in VSCode.
- `README.md`: This README file.


## Get Visualization:

This will only work on **BASE LINUX MACHINES** like the lab computers, it will **NOT** work on Macs or Window machines:

In a local terminal ```ctrl+T``` run the following command

```
xhost +
```
![xhost](../Drexel_Intro_labs/graphics/xhost.png)




## Turtlebot Instructions

You can find the manual for the Turtlebot [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

Please look over the manual to understand how to control the robot, send commands, run Gazebo and Rviz, ect.

In this course, we are using the Turtlebot3 model **TURTLEBOT3_MODEL=waffle_pi**. This is already setup for you so **YOU SHOULD NOT** change the model type of the robot (```export TURTLEBOT3_MODEL=burger```). 


There is one small quirk that is not highlighted in the manual: 

![Gazebo Click button](../Drexel_Intro_labs/graphics/gazebo_click.png)

## Connecting to the Turtlebot

There are several steps required to connect and control the turtlebot. There is setup required on both the host computer and the turtlebot. 

First, you need to find which turtlebot you are using:

- `GROUP1 (EULER)` : `192.168.0.164`
- `GROUP3 (GAUSS)` : `192.168.0.163`
- `GROUP4 (MAXWELL)` : `192.168.0.67`
- `GROUP5 (CURIE)` : `192.168.0.162`
- `GROUP6 (NEWTON)` : `192.168.0.161`



First, turn on the robot either by connecting to the wall power or using a battery. Ensure you are on the **DrexelRobotics** wifi network. The robot is already on the network. 

You need to **ssh** into the robot in a new terminal (```ctrl+T``` ):

```bash
ssh ubuntu@<ROBOT_IP>
```

Enter the username and password:


`username`: `ubuntu` \
`password`: `turtlebot`

This terminal is now the turtlebot, it is no longer your computer of the Docker container. 

You can now run the commands to start the turtlebot, see the manual for the commands. 

In the Docker container you now need to setup you workspace to use the robot as the **ROS MASTER**. 

To do this change you need to change `ROS_MASTER_URI` and `ROS_HOSTNAME` in the `.bashrc' directory

To edit the file you need:

```bash
vim ~/.bashrc 
```
This will allow you to use the vim editor to edit the file. You can find out to use the the editor [here](https://www.vim.org/)

For using ROS locally you need to change the file so that 

```
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
```


To connect to a robot, first ensure that you are on our local LAN network ()
Then edit the `.bashrc` file to the following:


```
export ROS_HOSTNAME=MY_IP
export ROS_MASTER_URI=http://ROBOT_IP:11311
```

You can find your computer's IP address using the command 

```bashrc
ifconfig
```

## Happy Coding

Good luck, everyone. Please reach out if you need any help!


