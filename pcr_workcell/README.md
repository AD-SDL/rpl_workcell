
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

Note: Password to Strange is ....

```
ssh rpl@146.137.240.63
./wei_ws/demo/rpl_workcell/scripts/run_nodes_strange.sh
```
### Start ROS2 Nodes on Parker
Note: Password to Parker is ....

```
ssh rpl@146.137.240.64
./wei_ws/demo/rpl_workcell/scripts/run_nodes_parker.sh
```

### Navigate in between TMUX shells
If TMUX session is already running, you can navigate in between windows to check all the nodes. Number that correstponse to the nodes listed along bottum of the shell.
- `Ctrl+B` 
- `Desired number`
- On Strange, available window numbers are: 0 and 1 
- On Parker, available window numbers are: 0 to 6 

### Run PCR Campaign on your local computer
