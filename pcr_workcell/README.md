
## Initial setup 
Make sure all the devices are turned on
pipette tips
correct pipettes for PCR
deck the deck OT2
Biometra lid
Sealer check seal if have enough
Sealer check seal type
Peeler check tape

## Running the PCR Workflow

### Running ROS2 NODES
In order start the PCR Workflow, you should start ROS2 Nodes on "Parker" and "Strange" computers. Currently, Parker is assigned to run the nodes for OT2, Sealer, Pealer, Biometra and Sciclops robots and "Strange" is assigned to run the nodes for PF400 robot and Camera Module. To start the node follow the steps below.
### Start ROS2 Nodes on Strange
```
ssh rpl@146.137.240.63
Note: Password is ....
./wei_ws/demo/rpl_workcell/scripts/run_nodes_strange.sh
```
### Start ROS2 Nodes on Parker
```
ssh rpl@146.137.240.64
Note: Password is ....
./wei_ws/demo/rpl_workcell/scripts/run_nodes_parker.sh
```
### Navigate in between TMUX shells

### Run PCR protocol on your local computer

## Running the Campaigns

For the PCR campaign:

```
source ~/wei_ws/install/setup.bash
./wc_client_run.py -wf workflows/pcr_workflow.yaml
```

For the MoveTest campaign:
```
source ~/wei_ws/install/setup.bash
./wc_client_run.py -wf /workflows/move_test.yaml
```


