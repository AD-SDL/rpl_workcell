# rpl-workcell

Ian is writing some notes here that could eventually become useful documtnation--or not :-)

## Background on Workcells, Modules, and Workflows

In RPL we co

![Screenshot of a comment on a GitHub issue showing an image, added in the Markdown, of an Octocat smiling and raising a tentacle.](assets/workcells.jpg)

An RPL "workflow" is a program to cause one or more actions to be performed on OT2 robots. It comprises two components:
* The *workcell definition* defines the robots and associated static infrastructure that are to be used by the workflow
* The  *workflow definition* defines the sequence of actions that are to be executed in order on the robots.

### Workcell definition

This is specified by a YAML file that defines the robots and associated static infrastructure that are to be used by the workflow. E.g., see [this example](https://github.com/AD-SDL/rpl_workcell/blob/main/pcr_workcell/pcr_workcell.yaml). The file comprises two sections, *config* and *modules*:

* The **config** section defines various variables that may be used elsewhere in the workcell. For example, here is the config from the example just listed.

```
  ros_namespace: rpl_workcell                                 # ???
  funcx_local_ep: "299edea0-db9a-4693-84ba-babfa655b1be"      # UUID used for local computations
  globus_local_ep: ""                                         # 
  globus_search_index: "aefcecc6-e554-4f8c-a25b-147f23091944" # UUID for the Globus Search instance
  globus_portal_ep: "bb8d048a-2cad-4029-a9c7-671ec5d1f84d"    # ???
  globus_group: "dda56f31-53d1-11ed-bd8b-0db7472df7d6"        # ???
```

* The **modules** section lists the *modules* that are included in the workcell.  


Each robot defines its own protocols (ROS2, EPICS, TCP/IP, etc) and the variables necessary to interact with it (IP, PORT, NAME, ETC)


### Workflow definition

There are essentially 2 files necessary for a workflow
1 workcell.yaml : This defines the robots and all the static infrastructure.

2 - workflow.yaml : This defines the sequence of actions that will be executed in order on the robots. This file uses the "alias" defined for each robot above and a funcx style message:
Step Name: Name on the workflow
Robot: Target Robot
Action name: Action to be executed on the robot
Vars: variable dictionary for that particular action
