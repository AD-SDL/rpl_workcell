RPL Color Picker

The RPL Color picker system is designed as a test system for integrating machine learning and optimization techniques with the robotic experimentation available in our workcell. The system chooses a random target color, and then runs one of a number of different optimizers to find the optimal combination of 3 differently colored liquids to most closely match that color.  The colors are mixed using the robotic workcell in RPL, allowing for continuous running and higher throughput. The system is intended as a demonstration of the RPLs ability to facilitate automatic discovery.

Workcell
The color picker workflow integrates a number of different modules in the overall RPL system. Please refer to the RPL workcell documentation for the terminology around different levels of functioning. 
Modules (in order of use in workflow):
Hudson Sciclops:  This module stores and provides the well plates used in the color picking protocol. It consists of a crane capable of moving standard size well plates, and a set of storage towers with different plates in them. 96-well plates are kept in tower 1 of the system. When the workflow is started, the arm will move to tower 1, lower itself until it makes contact with the first available well plate, and then transfer this well plate from the storage racks onto  the exchange position on the module. (Link to Sciclops Repo)

PF400: This module moves the well plate between other modules. It consists of a 5-DOF robotic arm that moves along a rail placed in the center of the other modules. Its first action is to pick up the well plate from the Sciclops exchange location, and transfer it to the Camera Module, described below. During each iteration of the color picker it will transfer the plate from the Camera Module to the OT2 Module, and then back to the Camera once the OT2 has completed its protocol. When the well plate is full, the arm will transfer it to a trash bin located at the end of the rail. The 


Running Instructions:
Basic steps:
	1. Turn  on Strange, Parker and Logan NUCS
	2. Check that each of the liquid containers on the OT2 has enough liquid for the full run,
	    And check that all of the tips for the OT2 are properly in the tip block
	3. From Logan, in separate terminals, run ssh rpl@parker and ssh rpl@strange
	4. On both terminals, cd into  ~/workspace/rpl_workcell/scripts folder
	5. On Parker, run ./run_nodes_parker
	6. On Strange run ./run_nodes_strange
	7. In a new terminal on Logan, run source source ~/wei_ws/install/setup.bash
  8. cd globusconnectpersonal-3.2.0, and then run ./globusconnectpersonal & to start the globus endpoint for publishing
  9. run funcx-endpoint start default to start funcx for publishing
  10.  cd into ~/workspace/rpl_workcell/color_picker
  11. Run ./color_picker_loop with the following Arguments:
	—pop_size: number of wells per loop of the color_picker algorithm
	—exp_budget: number of wells total allowed for the experiment
	—solver: a string denoting the solver to be used for experiment
   Example: ./color_picker_loop —pop_size=8 —exp_budget=96 —solver=Agg
Troubleshooting:
	
