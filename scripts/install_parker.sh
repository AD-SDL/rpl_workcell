$wS = ~/workspace


mkdir -p $WS
cd $WS

##Peeler
git clone https://github.com/AD-SDL/a4s_sealer_module
cd a4s_sealer_module/a4s_sealer_driver
pip install -r requirements.txt
pip install . 
cd $WS


##Sealer
git clone https://github.com/AD-SDL/brooks_xpeel_module
cd brooks_xpeel_module/brooks_xpeel_driver
pip install -r requirements.txt
pip install . 
cd $WS

##Sciclops
git clone https://github.com/AD-SDL/hudson_platecrane_module
cd hudson_platecrane_module/hudson_platecrane_driver
pip install -r requirements.txt
pip install . 
cd $WS

##OT2
git clone https://github.com/AD-SDl/ot2_module
cd ot2_module/ot2_driver
pip install -r requirements.txt
pip install . 
cd $WS