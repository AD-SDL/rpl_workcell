<h1>RPL Color Picker</h1>

The RPL Color picker system is designed as a test system for integrating machine learning and optimization techniques with the robotic experimentation available in our workcell. The system chooses a random target color, and then runs one of a number of different optimizers to find the optimal combination of 3 differently colored liquids to most closely match that color.  The colors are mixed using the robotic workcell in RPL, allowing for continuous running and higher throughput. The system is intended as a demonstration of the RPLs ability to facilitate automatic discovery.

<h1>Workcell</h1>
The color picker workflow integrates a number of different modules in the overall RPL system. Please refer to the RPL workcell documentation for the terminology around different levels of functioning. 
Modules (in order of use in workflow):
Hudson Sciclops:  This module stores and provides the well plates used in the color picking protocol. It consists of a crane capable of moving standard size well plates, and a set of storage towers with different plates in them. 96-well plates are kept in tower 1 of the system. When the workflow is started, the arm will move to tower 1, lower itself until it makes contact with the first available well plate, and then transfer this well plate from the storage racks onto  the exchange position on the module. (Link to Sciclops Repo)

PF400: This module moves the well plate between other modules. It consists of a 5-DOF robotic arm that moves along a rail placed in the center of the other modules. Its first action is to pick up the well plate from the Sciclops exchange location, and transfer it to the Camera Module, described below. During each iteration of the color picker it will transfer the plate from the Camera Module to the OT2 Module, and then back to the Camera once the OT2 has completed its protocol. When the well plate is full, the arm will transfer it to a trash bin located at the end of the rail. The 


<h1>Running Instructions:</h1>
<h2>Basic steps:</h2>
	<br>1. Turn  on Strange, Parker and Logan NUCS 
	<br>Strange:
	![image](https://user-images.githubusercontent.com/73187720/232096218-127fba12-eebf-4bdb-8cdc-f3e4d3059723.png)
	![image](https://user-images.githubusercontent.com/73187720/232096343-212a562a-e812-4a55-85e7-2fe8eebe4de8.png)

	<br>2. Check that each of the liquid containers on the OT2 has enough liquid for the full run,
	    And check that all of the tips for the OT2 are properly in the tip block
	<br>3. From Logan, in separate terminals, run  <em>ssh rpl@parker </em> and  <em>ssh rpl@strange </em>
	<br>4. On both terminals,run  <em>cd ~/workspace/rpl_workcell/scripts </em> folder
	<br>5. On Parker,run  <em>./run_nodes_parker</em>
	<br>6. On Strange run  <em>./run_nodes_strange</em>
	
![image](https://user-images.githubusercontent.com/73187720/232088734-ced6f822-847e-4e9e-bd37-165fc8b0982e.png)


  7. In a new terminal on Logan, run  <em>source ~/wei_ws/install/setup.bash</em>
  8. On Logan, run  <em>cd globusconnectpersonal-3.2.0 </em>, and then  <em>run ./globusconnectpersonal & </em> to start the globus endpoint for publishing
  9. Run  <em>funcx-endpoint start default </em> to start funcx for publishing
  10.  Run  <em>cd ~/workspace/rpl_workcell/color_picker </em>
  11. Run  <em>./color_picker_loop </em> with the following Arguments:
	--pop_size: number of wells per loop of the color_picker algorithm
	--exp_budget: number of wells total allowed for the experiment
	--solver: a string denoting the solver to be used for experiment
   Example:  <em>./color_picker_loop —pop_size=8 —exp_budget=96 —solver=Agg </em><br>
<h1>Troubleshooting:</h1>
<h2> Hardware</h2>
Check power on all relevant computers
<h2>Code</h2>
	
