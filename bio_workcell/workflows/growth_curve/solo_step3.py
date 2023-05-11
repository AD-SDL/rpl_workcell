import os
import sys
import time
import argparse
from liquidhandling import SoloSoft
from liquidhandling import Reservoir_12col_Agilent_201256_100_BATSgroup, Plate_96_Corning_3635_ClearUVAssay, DeepBlock_96VWR_75870_792_sterile


def generate_hso_file(
        payload, 
        temp_file_path,
): 
    """generate_hso_file

    Description: 
        Generates SOLOSoft .hso file for step 3 of the growth curve workflow

        Step 3 of the growth curve protocol includes:
            - Transfer of serial diluted treatment (from step 2) into assay plate

    Args:
        payload (dict): input variables from the wei workflow
        temp_file_path (str): file path to temporarily save hso file to 
    """
    
    # extract payload variables
    try: 
        treatment = payload['treatment'] 
        culture_column = payload['culture_column']
        culture_dil_column = payload['culture_dil_column']
        media_start_column = payload['media_start_column']
        treatment_dil_half = payload['treatment_dil_half']
    except Exception as error_msg: 
        # TODO: how to handle this?
        raise error_msg
    
    # other protocol variables
    blowoff_volume = 10
    num_mixes = 3
    media_z_shift = 0.5
    reservoir_z_shift = 0.5  # z shift for deep blocks (Deck Positions 3 and 5)
    flat_bottom_z_shift = 2  # Note: 1 is not high enough (tested)

    # Step 3 variables
    antibiotic_transfer_volume_s3 = 90
    antibiotic_mix_volume_s3 = 90
    destination_mix_volume_s3 = 100

    """
    STEP 3: ADD ANTIBIOTIC TO CULTURE PLATES -------------------------------------------------------------------------------------
    """
    # * Initialize soloSoft (step 3)
    soloSoft = SoloSoft(
        filename=temp_file_path,
        plateList=[
            "DeepBlock.96.VWR-75870-792.sterile",
            "Empty",
            "TipBox.180uL.Axygen-EVF-180-R-S.bluebox",
            "Plate.96.Corning-3635.ClearUVAssay",
            "DeepBlock.96.VWR-75870-792.sterile",
            "DeepBlock.96.VWR-75870-792.sterile",
            "DeepBlock.96.VWR-75870-792.sterile",
            "DeepBlock.96.VWR-75870-792.sterile",
        ],
    )

    soloSoft.getTip("Position3")  
    for i in range(6, 0, -1):  # first half of plate
        # if i == 3:  # switch tips half way through to reduce error  # tested and ok to remove
        #     soloSoft.getTip()
        soloSoft.aspirate(
            position="Position6",
            aspirate_volumes=Reservoir_12col_Agilent_201256_100_BATSgroup().setColumn(
                (6 * (treatment_dil_half - 1)) + i, antibiotic_transfer_volume_s3
            ),
            mix_at_start=True,
            mix_cycles=num_mixes,
            mix_volume=antibiotic_mix_volume_s3,
            dispense_height=reservoir_z_shift,
            aspirate_shift=[0, 0, reservoir_z_shift],
        )
        soloSoft.dispense(
            position="Position4",
            dispense_volumes=Plate_96_Corning_3635_ClearUVAssay().setColumn(
                i, antibiotic_transfer_volume_s3
            ),
            mix_at_finish=True,
            mix_cycles=num_mixes,
            mix_volume=destination_mix_volume_s3,
            aspirate_height=flat_bottom_z_shift,
            dispense_shift=[0, 0, flat_bottom_z_shift],
        )

    soloSoft.getTip("Position3")
    for i in range(6, 0, -1):  # second half of plate
        # if i == 3:  # switch tips half way through to reduce error  # tested and ok to remove
        #     soloSoft.getTip()
        soloSoft.aspirate(
            position="Position6",
            aspirate_volumes=Reservoir_12col_Agilent_201256_100_BATSgroup().setColumn(
                (6 * (treatment_dil_half - 1)) + i, antibiotic_transfer_volume_s3
            ),
            mix_at_start=True,
            mix_cycles=num_mixes,
            mix_volume=antibiotic_mix_volume_s3,
            dispense_height=reservoir_z_shift,
            aspirate_shift=[0, 0, reservoir_z_shift],
        )
        soloSoft.dispense(
            position="Position4",
            dispense_volumes=Plate_96_Corning_3635_ClearUVAssay().setColumn(
                i + 6, antibiotic_transfer_volume_s3
            ),
            mix_at_finish=True,
            mix_cycles=num_mixes,
            mix_volume=destination_mix_volume_s3,
            aspirate_height=flat_bottom_z_shift,
            dispense_shift=[0, 0, flat_bottom_z_shift],
        )

    soloSoft.shuckTip()
    soloSoft.savePipeline()
