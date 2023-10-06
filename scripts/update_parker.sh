WS=~/workspace

mkdir -p $WS
cd $WS

##Peeler

cd a4s_sealer_module
git pull
pip install -r requirements.txt
pip install . 
cd $WS


##Sealer

cd brooks_xpeel_module
git pull
pip install -r requirements.txt
pip install . 
cd $WS

##Sciclops
cd hudson_platecrane_module
git pull
pip install -r requirements.txt
pip install . 
cd $WS

##OT2
cd ot2_module
git pull
pip install -r requirements.txt
pip install . 
cd $WS
