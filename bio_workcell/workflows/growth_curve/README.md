# Growth Curve WEI Workflow

## Initial Setup

Before running the Growth Curve workflow, visually check each machine in the bio workcell to ensure that it is powered on, and ready to perform its task in the workflow as follows. See documentation on the bio workcell (TODO: link)
### SOLO Liquid Handler
Our Hudson SOLO liquid handler was expanded to contain 8 deck positions, 7 of which are required for this growth curve workflow.

- Make sure the SOLO liquid handler is powered on and that the proper labware is placed on the SOLO deck as described below
-  (TODO: link to SOLO module description)
#### Deck Layout

<img src="https://github.com/AD-SDL/rpl_workcell/blob/main/bio_workcell/workflows/growth_curve/growth_curve_description/resources/figures/SOLO_deck_positions.png"  width="60%" height="60%" alt="SOLO deck positions with labels">

<img src="https://github.com/AD-SDL/rpl_workcell/blob/main/bio_workcell/workflows/growth_curve/growth_curve_description/resources/figures/gc_layout_at_start.png"  width="60%" height="60%" alt="Labware layout at start of growth curve workflow">



- TODO: Photo of resource layout at start


- Position 1: Media stock plate
    - Example labware: Thermo Scientific™ Abgene™ 96 Well 2.2mL Polypropylene Deepwell Storage Plate
    - 2 adjacent columns filled with media per assay plate created
        (ex. media in columns 1 and 2 for 1 assay plate)

- Position 2: Heat Nest (EMPTY)
    - Note: Bio workcell contains a heat nest at SOLO deck position 2 but the heat nest is not required for this protocol

- Position 3: EMPTY at start
    - 180uL Filter Tip Box will be placed on this location by the plate crane at the start of the workflow
    - Example labware: Axygen™ Automation Tips - 180uL, sterile, filtered
    - 5 columns of tips used per assay plate created

- Position 4: EMPTY at start
    - an empty assay plate will be placed on this location by the plate crane at the start of the workflow

- Position 5: Culture stock plate
    - Example labware: Thermo Scientific™ Abgene™ 96 Well 2.2mL Polypropylene Deepwell Storage Plate
    - 1 column of cultured stock cells per assay plate created
    - Thaw plate before workflow begins if culture stock plate is taken from the freezer.

- Position 6: Treatment serial dilution plate
    - Example labware: Thermo Scientific™ Abgene™ 96 Well 2.2mL Polypropylene Deepwell Storage Plate
    - An empty half of treatment serial dilution plate is needed per assay plate created

- Position 7: Culture dilution plate
    - Example labware: Thermo Scientific™ Abgene™ 96 Well 2.2mL Polypropylene Deepwell Storage Plate
    - One empty column required per assay plate created

- Position 8: Treatment stock plate 
    - This labware contains the treatment which will be serial diluted and applied to the cells in the assay plate.
    - Thermo Scientific™ Abgene™ 96 Well 2.2mL Polypropylene Deepwell Storage Plate
    - One column of treatment stock plate used per assay plate created

### Plate Crane EX

(TODO: Photo of stacks at start of run)
#### Stack layout

- Stack 1: EMPTY at start 
    - Used assay plates will be stored here after the growth curve workflow is complete
- Stack 2: EMPTY at start 
    - Used tip boxes will be stored here after the growth curve workflow is complete
- Stack 3: EMPTY
- Stack 4: New 180uL tip boxes 
    - Axygen™ Automation Tips - 180uL, sterile, filtered
- Stack 5: Empty assay plates
    - Falcon™ 96-Well, Cell Culture-Treated, Flat-Bottom Microplate, Non-sterile

### Azenta Microplate Sealer


(TODO: link to sealer module setup and details)

Loaded Seal
- Azenta Gas Permiable Heat Seals (4ti-0598) 
https://www.azenta.com/products/gas-permeable-heat-seal

### Azenta Microplate Seal Remover (Peeler)
- TODO 
### LiCONiC STX88 Incubator

- TODO: Link to incubtor repo

- Ensure the incubator is powered on and set up according to instructions in the above repo
    - TODO: make the above repo contain those instructions

## Workflow Details 

### Main Workflow Files  
- TODO: link to main workflow file 
- TODO: link to both yaml files
- TODO: link to all 3 hso creation files

### Workflow Steps 
- TODO: Diagram of workflow like Ian made 










    


    





