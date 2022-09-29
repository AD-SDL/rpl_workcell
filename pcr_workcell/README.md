# RPL Weekly Campaigns



## Initial setup 

```
DEMO_PATH=~/wei_ws
mkdir $DEMO_PATH
cd $DEMO_PATH
mkdir demo
cd demo
git clone https://github.com/AD-SDL/rpl_workcell.git
```

## Installing the nodes

```
./$DEMO_PATH/demo/rpl_workcell/pcr_workcell/install.sh
```


## Running the example

```
./$DEMO_PATH/demo/rpl_workcell/pcr_workcell/run_nodes.sh
```

*This will consume your terminal with a running command. Open a new terminal and navigate to this folder again.*

And finally run the scripts with the steps you want to test (in the new terminal)


## Running the Campaigns

For the PCR campaign:

```
source $DEMO_PATH/install/setup.bash
python3 $DEMO_PATH/demo/rpl_workcell/pcr_workcell/wc_client_run.py -wc $DEMO_PATH/demo/rpl_workcell/pcr_workcell/pcr_workcell.yaml -wf $DEMO_PATH/demo/rpl_workcell/pcr_workcell/workflows/pcr_workflow.yaml
```

For the Growth campaign:

```
python3 $DEMO_PATH/demo/rpl_workcell/pcr_workcell/wc_client_run.py -wc $DEMO_PATH/demo/rpl_workcell/pcr_workcell/pcr_workcell.yaml -wf $DEMO_PATH/demo/rpl_workcell/pcr_workcell/workflows/growth_workflow.yaml
```

For the MoveTest campaign:
```
python3 $DEMO_PATH/demo/rpl_workcell/pcr_workcell/wc_client_run.py -wc $DEMO_PATH/demo/rpl_workcell/pcr_workcell/pcr_workcell.yaml -wf $DEMO_PATH/demo/rpl_workcell/pcr_workcell/workflows/move_test.yaml
```


