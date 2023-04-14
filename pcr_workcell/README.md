
## Initial setup 
- Make sure all the devices are turned on
- pipette tips
- correct pipettes for PCR
- deck the deck OT2
- Biometra lid
- Sealer check seal if have enough
- Sealer check seal type
- Peeler check tape

## Running the PCR Workflow

### Running ROS2 NODES
In order start the PCR Workflow, you should start ROS2 Nodes on "Parker" and "Strange" computers. Currently, Parker is assigned to run the nodes for OT2, Sealer, Pealer, Biometra and Sciclops robots and "Strange" is assigned to run the nodes for PF400 robot and Camera Module. To start the node follow the steps below.
### Start ROS2 Nodes on Strange
- Open a new shell and ssh into the Strange.
- Note: Password to Strange is ....

```
ssh rpl@146.137.240.63
./home/rpl/wei_ws/demo/rpl_workcell/scripts/run_nodes_strange.sh
```
### Start ROS2 Nodes on Parker
- Open a new shell and ssh into the Strange
- Note: Password to Parker is ....

```
ssh rpl@146.137.240.64
./home/rpl/wei_ws/demo/rpl_workcell/scripts/run_nodes_parker.sh
```

### Navigate in between TMUX shells
If TMUX session is already running, you can navigate in between windows to check all the nodes. Number that correstponse to the nodes listed along bottum of the shell.
- `Ctrl+B` 
- `Desired number`
- On Strange, available window numbers are: 0 and 1 
- On Parker, available window numbers are: 0 to 6 

### Run PCR Campaign on your local computer
- Before running the PCR Campaign check all the TMUX windows on both Parker and Strange to make sure all the robots are publishing "READY" state. If any of the robots are in "ERROR" state refer to the DEBUGGING Section in the Template. 
- Open a new shell on the local computer.
```
source /opt/ros/humble/setup.bash  
source wei_ws/install/setup.bash
./home/rpl/workspace/rpl_workcell/pcr_workcell/demo.py
```

### DEBUGGING Robot Problems

#### PF400 issues
#### Sealer issues
#### Peeler issues
#### OT2 issues
#### Sciclops issues
#### Other issues