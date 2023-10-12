#!/bin/bash

$WS = ~/workspace

mkdir -p $WS
cd $WS

##PF400
cd pf400_module
git pull
pip install -r requirements.txt
pip install . 
cd $WS

## Camera Module
cd camera_module
git pull
pip install -r requirements.txt
cd $WS