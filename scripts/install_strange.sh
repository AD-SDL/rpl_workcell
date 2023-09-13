#!/bin/bash

$WS = ~/workspace

mkdir -p $WS
cd $WS

##PF400
git clone https://github.com/AD-SDL/pf400_module
cd pf400_module
pip install . 
cd $WS

## Camera Module
git clone https://github.com/AD-SDL/camera_module.git

