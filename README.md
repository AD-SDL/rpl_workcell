# Background on Workcells, Carts, Modules, and Workflows

In RPL we define standardized hardware and software configurations for robotic equipment and control software in order to simplify the assembly, modification, and scaling of experimental systems:
* A **cart** is a cart with zero or more modules 
* A **module** is an hardware component with a name, type, position, etc. (e.g., Pealer, Sealer, OT2 liquid handling robot, plate handler, plate mover, camera)
* A **workcell**, as show on the left of the image, is formed from multiple (8 in the photo on the left) carts that typically hold multiple modules (12 in the example, as described below).
* Multiple workcells and other components can be linked via mobile robots

![Image of a workcell with four carts, connected via mobile robots to other workcells; on the far right, a workflow definition.](assets/AD_Fig.jpg)

An RPL "workflow" is a program to cause one or more actions to be performed on equipment within a workcell. It comprises two components:
* The **workcell definition** defines the modules that comprise a workcell, and associated static infrastructure that are to be used by the workflow
* The **workflow definition** defines the sequence of actions that are to be executed in order on the modules.

## Workcell definition

A workcell definition is a YAML file (e.g., [pcr_workcell.yaml](https://github.com/AD-SDL/rpl_workcell/blob/main/pcr_workcell/pcr_workcell.yaml)) comprising two sections, *config* and *modules*:

The **config** section defines various infrastructure services that may be used elsewhere in the workcell. For example, here is the config from the example just listed.

```
  ros_namespace: rpl_workcell                                 # ROS variable namespace name
  funcx_local_ep: "299edea0-db9a-4693-84ba-babfa655b1be"      # UUID for funcX endpoint used for local computations
  globus_local_ep: ""                                         # 
  globus_search_index: "aefcecc6-e554-4f8c-a25b-147f23091944" # UUID for the Globus Search instance
  globus_portal_ep: "bb8d048a-2cad-4029-a9c7-671ec5d1f84d"    # UUID for the portal to which data may be published
  globus_group: "dda56f31-53d1-11ed-bd8b-0db7472df7d6"        # 
```

The **modules** section lists the *modules* that are included in the workcell. In the example just listed, there are 12 in total: 
* a [pf400 sample handler](https://preciseautomation.com/SampleHandler.html) (**pf400**) and two associated cameras, **pf400_camera_right** and **pf400_camera_left**; 
* a [SciClops plate stacker](https://hudsonrobotics.com/microplate-handling-2/platecrane-sciclops-3/) (**sciclops**)
* a A4S (**sealer**) and a Brooks XPeel (**peeler**), with an associated camera, **sp_module_camera**
* three OpenTrons OT2 liquid handlers, **ot2_pcr_alpha**, **ot2_pcr_beta**, and **ot2_cp_gamma**;
* a [Biometra thermal cycler](https://www.analytik-jena.com/products/life-science/pcr-qpcr-thermal-cycler/thermal-cycler-pcr/biometra-trio-series/) (**biometra**)
* another camera module, **camera_module**
           
Here is one of the 12 module specifications included in our example:

```
  - name: sealer                     # A name used for the module in the workflow: its "alias"
    type: wei_ros_node               # Indicates that module uses ROS2
    model: sealer                    # Not used at present
    config:
      ros_node: "/std_ns/SealerNode" # ROS2 network name (in name space)
    positions:                       # One or more spatial locations, with name 
      default: [205.128, -2.814, 264.373, 365.863, 79.144, 411.553]
```

The positions here are specific to the PF400: they give joint angles.

For other apparatus, the specification could include things like protocol and IP port.

## Workflow definition

This is specified by a YAML file that defines the sequence of actions that will be executed in order on the hardware. E.g., see [this example](https://github.com/AD-SDL/rpl_workcell/blob/main/color_picker/workflows/cp_wf_mixcolor.yaml), shown also in the following, and comprising four sections:
* **metadata**: Descriptive metadata for the workflow
* **workcell**: The location of the workcell for which the workflow is designed
* **modules**: A list of the modules included in the workcell--four in this case.
* **flowdef**: A list of steps, each with a name, module, command, and arguments.


```
metadata:
  name: PCR - Workflow
  author: Casey Stone, Rafael Vescovi
  info: Initial PCR workflow for RPL workcell
  version: 0.1

workcell: /home/rpl/workspace/rpl_workcell/pcr_workcell/pcr_workcell.yaml

modules:
  - name: ot2_cp_gamma
  - name: pf400
  - name: camera

flowdef:
  - name: Move from Camera Module to OT2
    module: pf400
    command: transfer
    args:
      source: camera_module.positions.plate_station
      target: ot2_cp_gamma.positions.deck2
      source_plate_rotation: narrow
      target_plate_rotation: wide
    comment: Place plate in ot2

  - name: Mix all colors
    module: ot2_cp_gamma
    command: run_protocol
    args:
      config_path:  /home/rpl/workspace/rpl_workcell/color_picker/protocol_files/combined_protocol.yaml
      red_volumes: payload.red_volumes
      green_volumes: payload.green_volumes
      blue_volumes: payload.blue_volumes
      destination_wells: payload.destination_wells
      use_existing_resources: payload.use_existing_resources
    comment: Mix the red portions according to input data

  - name: Move to Picture
    module: pf400
    command: transfer
    args:
      source: ot2_cp_gamma.positions.deck2
      target: camera_module.positions.plate_station
      source_plate_rotation: wide
      target_plate_rotation: narrow

  - name: Take Picture
    module: camera_module
    command: take_picture
    args:
      save_location: local_run_results
      file_name: "final_image.jpg"
```


This workflow uses three of 12 modules defined in the workcell definition earlier, **pf400**, **ot2_pcr_gamma**, and **camera_module**.
It comprises four steps:
* Transfer a plate from `camera_module.positions.plate_station` to `ot2_cp_gamma.positions.deck2`, while rotating the plate 90 degrees
* Run the "protocol" defined by the file [ot2_pcr_config.yaml](https://github.com/AD-SDL/rpl_workcell/blob/main/color_picker/protocol_files/combined_protocol.yaml). 
This file specifies a sequence of steps to be performed on the hardware.
* Transfer the plate to the camera
* Take a picture of the plate

> While a workflow and a protocol both specify a sequence of actions to be performed, they are quite different in role and syntax. A **workflow** uses a hardware-independent notation to specify actions to perform on one or more modules (e.g., action A1 on module M1, action A2 on module M2); a **protocol** uses a hardware-specific notation to specify steps to be performed on a single module (e.g., OT2). Why *workflow* and *protocol*? Perhaps because this technology was developed by a partnership of computer scientists ("module", "workflow") and biologists ("protocol") :-)

## Protocols

A protocol file gives the device-specific instructions to be executed on a specific piece of hardware to implement an intended action. For example, [ot2_pcr_config.yaml](https://github.com/AD-SDL/rpl_workcell/blob/main/pcr_workcell/protocol_files/ot2_pcr_config.yaml) gives instructions for an OpenTrons OT2. A protocol file specifies a list of **equipment** within the hardware component; a sequence of **commands** to be executed on the equipment; and some describptive **metadata**. For example, the following shows the contents of [combined_protocol.yaml](https://github.com/AD-SDL/rpl_workcell/blob/main/color_picker/protocol_files/combined_protocol.yaml), which comprise the equipment section, three commands, and the metadata section. 

This is OT2-specific. The plate locations are numbered 1..11.

A 96-well plate has its wells labeled A..G and 1..16 


```
equipment:
  - name: corning_96_wellplate_360ul_flat
    location: "2"
    alias: dest
  - name: opentrons_6_tuberack_nest_50ml_conical
    location: "7"
    alias: source
  - name: opentrons_96_tiprack_300ul
    location: "8"
  - name: opentrons_96_tiprack_300ul
    location: "9"
  - name: opentrons_96_tiprack_300ul
    location: "10"
  - name: opentrons_96_tiprack_300ul
    location: "11"
  - name: p300_single_gen2
    mount: left

commands:
  - name: Mix Color 1
    source: source:A1
    destination: payload.destination_wells  # Argument provided (a dictionary)
    volume: payload.red_volumes             # Argument
    dispense_clearance: 2
    aspirate_clearance: 1
    drop_tip: False

  - name: Mix color 2
    source: source:A2
    destination: payload.destination_wells
    volume: payload.green_volumes
    dispense_clearance: 2
    aspirate_clearance: 1
    drop_tip: False    
  
  - name: Mix color 3
    source: source:A3
    destination: payload.destination_wells
    volume: payload.blue_volumes
    dispense_clearance: 2
    aspirate_clearance: 1
    mix_cycles: 3
    mix_volume: 100
    drop_tip: False

metadata:
  protocolName: Color Mixing all
  author: Kyle khippe@anl.gov
  description: Mixing all colors
  apiLevel: "2.12"
```

&#x1F34E;**Note for Raf**&#x1F34E;: You write as follows, about the protocol file I think (?).  However, this is not clear to me, as I do not see any of the words that you list  (step name, robot, action name, vars) in the example, or anything that looks like a "funcx style message." (What is a "funcx style message"?)

This file uses the "alias" defined for each robot above and a funcx style message:
* Step Name: Name on the workflow
* Robot: Target Robot
* Action name: Action to be executed on the robot
* Vars: variable dictionary for that particular action

&#x1F34E;**Note for Raf**&#x1F34E;: To know how to generate these files, we need to understand what they mean. Some questions:
* What do the commands mean? E.g., what are C1, F1, etc. What does volume: [13] mean, what is a mix_cycle, etc. Is that documented anywhere?
