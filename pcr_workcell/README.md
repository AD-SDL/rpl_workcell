
## Initial setup 
- Make sure all the devices are turned on
- pipette tips
- correct pipettes for PCR
- deck on the OT2
- Biometra lid
- Sealer check if there is enough seal
- Sealer check seal type
- Peeler check tape

## Running the PCR Workflow

### Running ROS2 NODES
In order to start the PCR Workflow, you should start ROS2 Nodes on "Parker" and "Strange" computers first. Currently, Parker is assigned to run the nodes for OT2, Sealer, Peeler, Biometra and Sciclops robots and "Strange" is assigned to run the nodes for PF400 robot and Camera Module. To start the node follow the steps below.
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
If TMUX sessions are already running, you can navigate in between windows to check all the nodes. Number that correspond to the nodes are listed along bottom of the window.
- `Ctrl+b` 
- `Desired window number`
- On Strange, available window numbers are: 0 and 1 
- On Parker, available window numbers are: 0 to 6 

If you know that TMUX sessions are running in the background, you reopen the session on your shell with below commands.

- `tmux a`
- `Ctrl+b`
- `w`
- Choose the window you want to display
### Run PCR Campaign on your local computer
- Before running the PCR Campaign check all the TMUX windows on both Parker and Strange to make sure all the robots are publishing "READY" state. If any of the robots are in "ERROR" state refer to the DEBUGGING Section in the Template. 
- Open a new shell on the local computer.
```
source /opt/ros/humble/setup.bash  
source ~/wei_ws/install/setup.bash
./home/rpl/workspace/rpl_workcell/pcr_workcell/pcr_full.py
```

### DEBUGGING Robot Problems

#### PF400 issues
#### Sealer issues
- If there is a connection problem because of the wrong port. Run the following commands to find the correct port name.
- Kill the Sealer node with `Ctrl+C`.
- `sudo dmesg | grep tty`
- Restart the Sealer node.
- `ros2 launch a4s_sealer_client a4s_sealer_client.launch.py sealer_port:={Your Port Name}` 
#### Peeler issues
- If there is a connection problem because of the wrong port. Run the following commands to find the correct port name.
- Kill the Sealer node with Ctrl+C.
- `sudo dmesg | grep tty`
- Restart the Peeler node.
- `ros2 launch brooks_peeler_client brooks_peeler_client.launch.py peeler_port:={Your Port Name}`
#### OT2 issues
#### Sciclops issues
#### Camera Module issues

#### Other issues