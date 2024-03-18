# 1. HU Motion Manager

- [1. HU Motion Manager](#markdown-header-1-hu-motion-manager)
  - [1.1. Simulation](#markdown-header-11-simulation)
  - [1.2. URSim](#markdown-header-12-ursim)
    - [1.2.1. URSim in Docker](#markdown-header-121-ursim-in-docker)
  - [1.3. Topics to be resolved](#markdown-header-13-topics-to-be-resolved)


## 1.1. Simulation

Simulation is started for the BAUTIRO system is a whole and not only for the HU module. Check ReadMe file in the btr_bautiro repo.

## 1.2. URSim
URSIm is simulator for UR Robots. Description by UR:
...URSim is a simulation software that is used for offline programming and simulation of robot programs.
There are some limitations to the simulator since no real robot arm is connected. Specially the force control will be limited in use. If Simulation Mode is selected in the bottom left corner, it is possible to simulate digital inputs on the I/O page....

### 1.2.1. URSim in Docker

To start URSim in docker with programs and configuration from FUS<#> follow the steps below. This is for a Linux computer.

1. Pull docker container: ***docker pull universalrobots/ursim_e-series***
2. This step is optional. If you would like to start simulator with programs and configuration of some robot, copy programs and configuration to a folder on your computer, for example ***~/ur/programs***. Programs/configuration of the FUS1 robot are stored here: *\\BOSCH.COM\DfsRB\DfsDE\DIV\PT\PT-BI\TOP100_BAUTIRO\internal\03_Development\LAB\Lab_131\FuS-1\UR3_FuS-1.prog*
3. Create docker subnet: ***docker network create --subnet=192.168.2.0/24 ursim_net***
4. Run docker container with: ***docker run --rm -it --net ursim_net --ip 192.168.2.3 -e ROBOT_MODEL=UR16 -v "${HOME}/ur/programs:/ursim/programs" universalrobots/ursim_e-series***
5. Robot GUI is available at: http://192.168.2.3:6080/vnc.html?host=192.168.2.3&port=6080

* Docs for URSim docker: https://github.com/urrsk/ursim_docker/blob/main/README.md

## 1.3. Topics to be resolved
* Collision of the HU with RPM is not implemented
* Suction hose is not modeled
* Cable carrier is not modeled
