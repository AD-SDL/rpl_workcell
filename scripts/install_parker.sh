#!/bin/bash
WS=~/workspace

mkdir -p $WS
cd $WS

##Peeler
git clone https://github.com/AD-SDL/a4s_sealer_module
cd a4s_sealer_module
pip install -r requirements.txt
pip install . 
cd $WS


##Sealer
git clone https://github.com/AD-SDL/brooks_xpeel_module
cd brooks_xpeel_module
pip install -r requirements.txt
pip install . 
cd $WS

##Sciclops
git clone https://github.com/AD-SDL/hudson_platecrane_module
cd hudson_platecrane_module
pip install -r requirements.txt
pip install . 
cd $WS

##OT2
git clone https://github.com/AD-SDl/ot2_module
cd ot2_module
pip install -r requirements.txt
pip install . 
cd $WS

##PF400
git clone https://github.com/AD-SDL/pf400_module
cd pf400_module
pip install -r requirements.txt
pip install . 
cd $WS

## Camera Module
git clone https://github.com/AD-SDL/camera_module.git
cd camera_module
pip install -r requirements.txt
cd $WS