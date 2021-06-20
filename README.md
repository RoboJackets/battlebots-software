# BattleBots Software

## Repo Organization

-------

### Simulations

This folder has models and code for simulating the performance of the meltybrain robot in MATLAB.

### Hocki1

Contains written and untested code from 2020-2021. Only for reference

### Hocki_PIO

PlatformIO project containing all relevant files. Work in progress for 2021-2022

## Instructions

-------

### Editor setup

1. Install [Visual Studio Code](https://code.visualstudio.com/). This is the recommended way of editing robot code.
2. Install [PlatformIO](https://platformio.org/). This is an add-on for VSCode which features the tool chains for compiling and programming embedded circuit boards.

### Repo setup

Clone this repository by opening a terminal in the desired location on yoru computer, then entering the following command.

```git clone --recursive https://github.com/robojackets/battlebots-software.git```

This repo uses submodules for libraries, so the ```--recursive``` flag ensures that they get included in your local copy.

If you have already cloned the repo and do not see the submodules, run
```git submodule update --init --recursive``` from a terminal within the folder of the repo.
