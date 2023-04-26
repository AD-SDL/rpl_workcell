# RPL Color Picker

The RPL Color picker system is designed as a test system for integrating machine learning and optimization techniques with the robotic experimentation available in our workcell. The system chooses a random target color, and then runs one of a number of different optimizers to find the optimal combination of 3 differently colored liquids to most closely match that color.  The colors are mixed using the robotic workcell in RPL, allowing for continuous running and higher throughput. The system is intended as a demonstration of the RPLs ability to facilitate automatic discovery.

## Workcell 
The color picker workflow integrates a number of different modules in the overall RPL system. Please refer to the RPL workcell documentation for the terminology around different levels of functioning. 

### Modules (in order of use in workflow): 
### Hudson Sciclops:  
This module stores and provides the well plates used in the color picking protocol. It consists of a crane capable of moving standard size well plates, and a set of storage towers with different plates in them. 96-well plates are kept in tower 1 of the system. When the workflow is started, the arm will move to tower 1, lower itself until it makes contact with the first available well plate, and then transfer this well plate from the storage racks onto  the exchange position on the module.  
Repo: https://github.com/AD-SDL/ot2_module

### PF400: 
This module moves the well plate between other modules. It consists of a 5-DOF robotic arm that moves along a rail placed in the center of the other modules. Its first action is to pick up the well plate from the Sciclops exchange location, and transfer it to the Camera Module, described below. During each iteration of the color picker it will transfer the plate from the Camera Module to the OT2 Module, and then back to the Camera once the OT2 has completed its protocol. When the well plate is full, the arm will transfer it to a trash bin located at the end of the rail.  
Repo: https://github.com/AD-SDL/ot2_module

### OT2: 
The OT2 module mixes all of the colors for the experiment. It is currently configured to use one 300mum tip per color per run, and to return it to the rack when it has finished.  
Repo: https://github.com/AD-SDL/ot2_module

### Camera Module: 
The Camera Module is takes a picture of the wellplate, and sends it to Logan.  
Repo: https://github.com/AD-SDL/camera_module
## Computers:
The username and password for the computers is written on top of Logan as username/password
### Parker:
NUC located on the Sealer-Peeler module cart on the left side of the setup.    
<img src="https://user-images.githubusercontent.com/73187720/232100088-9ae23729-c71b-45a3-8d05-9ca00f0bdd13.png"  width="300" height="400"><img src="https://user-images.githubusercontent.com/73187720/232099345-7c81732e-a3d7-45a9-a4ab-a90617c4e001.png"  width="300" height="400">

#### Modules:SciclopsOT2_gamma
### Strange:
NUC located on right side of the trash module setup connected to the PF-400  
<img src="https://user-images.githubusercontent.com/73187720/232096343-212a562a-e812-4a55-85e7-2fe8eebe4de8.png"  width="300" height="400"><img src="https://user-images.githubusercontent.com/73187720/232096218-127fba12-eebf-4bdb-8cdc-f3e4d3059723.png"  width="300" height="400">

#### Modules:PF-400Camera Module
### Logan: NUC located on the left side of setup of NUCs with monitors, runs the main loop for the color_picker  
<img src="https://user-images.githubusercontent.com/73187720/232106097-ebe051bf-8085-4a5c-85be-ee22ea282e48.png"  width="300" height="400">
<img src="https://user-images.githubusercontent.com/73187720/232529076-011500a5-7d9b-4ebf-90dd-26b40df82093.png"  width="300" height="400">

# Running Instructions:
## Basic steps:
1. Turn  on Strange, Parker and Logan NUCs  
2. Check that each of the liquid containers on the OT2 has enough liquid for the full run,  
and check that all of the tips for the OT2 are properly in the tip block  
<img src="https://user-images.githubusercontent.com/73187720/232107500-ded7d73e-1d4b-4111-bb41-fa5a4536f7ad.png"  width="300" height="400">

3. From Logan, in separate terminals, run  ```ssh rpl@parker ``` and  ```ssh rpl@strange ``` and use the password written on Logan  
4. On both terminals,run  ```cd ~/wei_ws/demo/rpl_workcell/scripts ```  
5. On Parker, try ``` tmux attach-session -t nodes```, and check that all nodes are displaying ready. If the session doesn't attach run  ```./run_nodes_parker```  
6. On Strange try ``` tmux attach-session -t nodes```, and check that all nodes are displaying ready. If the session doesn't attach run  ```./run_nodes_strange```  
<img src="https://user-images.githubusercontent.com/73187720/232088734-ced6f822-847e-4e9e-bd37-165fc8b0982e.png"  width="1000" height="400">  


  7. In a new terminal on Logan, run  
  ```
  source ~/wei_ws/install/setup.bash
  ```  
  8. In the same terminal, run 
  ```
  cd globusconnectpersonal-3.2.0 
  ```
  and then  
  ```
  run ./globusconnectpersonal &
  ``` 
  to start the globus endpoint for publishing. A window will pop up with a connect button. If when this connect button is pressed it says there is already an instance running, then you are free to close it and skip this step.     
  9. In the same terminal, Run  
  ```
   funcx-endpoint start default 
  ```
   to start funcx for publishing
  10. In the same terminal Run  
  ```
  cd ~/workspace/rpl_workcell/color_picker
   ```
  11. In the same terminal Run  
  ```
  ./color_picker_loop 
  ``` 
  with the following Arguments:
	--pop_size: number of wells per loop of the color_picker algorithm
	--exp_budget: number of wells total allowed for the experiment
	--solver: a string denoting the solver to be used for experiment
   Example:  
   ```
   ./color_picker_loop --pop_size=8 --exp_budget=96 --solver=Agg 
   ```
## Troubleshooting

### Hardware
1. Check power on all relevant computers
2. Check that all systems are plugged in
3. Check that the plates are dry enough not to stick to eachother
4. If the camera system is stalled out, check that the camera for the camera module is on the right port. This can be done by running 
```
cd ~/workspace/rosboard
/run
```
this will open a dashboard on `localhost:8888`. If you open the camera module channel on this site using the menu on the left, you can see what feed is coming from that camera. It should point down at the plate image station. This can be adjusted on strange by changing the number in this command: 
```
ros2 launch camera_module_client camera_publisher.launch.py camera_name:=camera_module camera_number:=$cam_num
```

## Software
1. If the publish flow is failing, rerun the funcx-endpoint using the same command above, and the same globus command above
2. Make sure to check the endpoints in `~/workspace/rpl_workcell/color_picker/tools/publish.py` line up with the local endpoints on Logan
