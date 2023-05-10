"""
Generates steps 1, 2, and 3 SOLO hso files given command line inputs

Returns paths to newly generated .hso files
"""
import os
import sys
import time
import argparse
from liquidhandling import SoloSoft
from liquidhandling import Reservoir_12col_Agilent_201256_100_BATSgroup, Plate_96_Corning_3635_ClearUVAssay, DeepBlock_96VWR_75870_792_sterile


# SOLO PROTOCOL STEPS    
def generate_hso_file(
        payload, 
        temp_file_path,
): 
    """generate_hso_file

    Description: 
        Generates SOLOSoft .hso file for step 1 of the growth curve workflow

        Step 1 of the growth curve protocol includes:
            - 1:10 dilution of cells from source culture plate
            - transfer of 1:10 diluted cells into assay plate

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
        # TODO
        raise error_msg


# * Other program variables
    blowoff_volume = 10
    num_mixes = 3
    media_z_shift = 0.5
    reservoir_z_shift = 0.5  # z shift for deep blocks (Deck Positions 3 and 5)
    flat_bottom_z_shift = 2  # Note: 1 is not high enough (tested)

    # Step 1 variables
    media_transfer_volume_s1 = 60
    culture_transfer_volume_s1 = 30
    half_dilution_media_volume = 99
    dilution_culture_volume = 22
    culture_plate_mix_volume_s1 = 100  # mix volume increased for test 09/07/21
    culture_plate_num_mix = 7
    culture_dilution_num_mix = 10
    growth_plate_mix_volume_s1 = 40
    culture_dilution_mix_volume = 180

    """
    STEP 1: INNOCULATE GROWTH PLATE FROM SOURCE BACTERIA PLATE -----------------------------------------------------------------
    """
    # * Initialize soloSoft (step 1)
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

    # * Fill all columns of empty 96 well plate (corning 3383 or Falcon - ref 353916) with fresh lb media (12 channel in Position 3, media_start_column and media_start_column+1)
    soloSoft.getTip("Position3")  
    j = 1
    for i in range(1, 7):  # first half plate = media from column 1
        soloSoft.aspirate(
            position="Position1",
            aspirate_volumes=Reservoir_12col_Agilent_201256_100_BATSgroup().setColumn(
                media_start_column, media_transfer_volume_s1
            ),
            aspirate_shift=[0, 0, media_z_shift],
        )
        soloSoft.dispense(
            position="Position4",
            dispense_volumes=Plate_96_Corning_3635_ClearUVAssay().setColumn(
                i, media_transfer_volume_s1
            ),
            dispense_shift=[0, 0, flat_bottom_z_shift],
        )

    for i in range(7, 13):  # second half plate = media from column 2
        soloSoft.aspirate(
            position="Position1",
            aspirate_volumes=Reservoir_12col_Agilent_201256_100_BATSgroup().setColumn(
                media_start_column + 1, media_transfer_volume_s1
            ),
            aspirate_shift=[0, 0, media_z_shift],
        )
        soloSoft.dispense(
            position="Position4",
            dispense_volumes=Plate_96_Corning_3635_ClearUVAssay().setColumn(
                i, media_transfer_volume_s1
            ),
            dispense_shift=[0, 0, flat_bottom_z_shift],
        )

    # * Fill one column of culture dilution plate with fresh lb media (do in two steps due to 180uL filter tips)
    for i in range(2):  # from first media column -> cell dilution plate, column = same as culture column
        soloSoft.aspirate(
            position="Position1",
            aspirate_volumes=Reservoir_12col_Agilent_201256_100_BATSgroup().setColumn(
                media_start_column, half_dilution_media_volume
            ),
            aspirate_shift=[0, 0, media_z_shift],
        )
        soloSoft.dispense(
            position="Position7",
            dispense_volumes=Reservoir_12col_Agilent_201256_100_BATSgroup().setColumn(
                culture_dil_column, half_dilution_media_volume
            ),
            dispense_shift=[0, 0, reservoir_z_shift],
        )

    for i in range(2):  # from second media column -> cell dilution plate
        soloSoft.aspirate(
            position="Position1",
            aspirate_volumes=Reservoir_12col_Agilent_201256_100_BATSgroup().setColumn(
                media_start_column + 1, half_dilution_media_volume
            ),
            aspirate_shift=[0, 0, media_z_shift],
        )
        soloSoft.dispense(
            position="Position7",
            dispense_volumes=Reservoir_12col_Agilent_201256_100_BATSgroup().setColumn(
                culture_dil_column, half_dilution_media_volume
            ),
            dispense_shift=[0, 0, reservoir_z_shift],
        )

    # * Make culture 10 fold dilution
    for i in range(1, 3):  # all cells dispensed into same cell dilution column
        soloSoft.aspirate(
            position="Position5",
            aspirate_volumes=DeepBlock_96VWR_75870_792_sterile().setColumn(
                culture_column, dilution_culture_volume
            ),
            aspirate_shift=[0, 0, 2],
            mix_at_start=True,
            mix_cycles=culture_plate_num_mix,
            mix_volume=culture_plate_mix_volume_s1,
            dispense_height=2,
            # pre_aspirate=blowoff_volume,
            syringe_speed=25,
        )
        soloSoft.dispense(
            position="Position7",
            dispense_volumes=Reservoir_12col_Agilent_201256_100_BATSgroup().setColumn(
                culture_dil_column, dilution_culture_volume
            ),
            dispense_shift=[0, 0, reservoir_z_shift],
            mix_at_finish=True,
            mix_cycles=num_mixes,
            mix_volume=culture_plate_mix_volume_s1,
            aspirate_height=reservoir_z_shift,
            syringe_speed=25,
            # blowoff=blowoff_volume,
        )

    # * Separate big mix step to ensure cell diluton column is well mixed  # added for 09/07/21
    soloSoft.aspirate(
        position="Position7",
        aspirate_volumes=Reservoir_12col_Agilent_201256_100_BATSgroup().setColumn(
            culture_dil_column, dilution_culture_volume
        ),
        aspirate_shift=[0, 0, reservoir_z_shift],
        # 100% syringe speed
    )
    soloSoft.dispense(
        position="Position7",
        dispense_volumes=Reservoir_12col_Agilent_201256_100_BATSgroup().setColumn(
            culture_dil_column, dilution_culture_volume
        ),
        dispense_shift=[0, 0, reservoir_z_shift],
        mix_at_finish=True,
        mix_cycles=culture_dilution_num_mix,
        mix_volume=culture_dilution_mix_volume,
        aspirate_height=reservoir_z_shift,
        syringe_speed=75,
        # blowoff=blowoff_volume,
    )

    # * Add bacteria from 10 fold diluted culture plate (Position 7, column = culture_column[k]) to growth plate with fresh media (both halves)
    soloSoft.getTip("Position3")  
    for i in range(1, 7):  # trying a different method of cell dispensing (09/07/21)
        soloSoft.aspirate(  # well in first half
            position="Position7",
            aspirate_volumes=Reservoir_12col_Agilent_201256_100_BATSgroup().setColumn(
                culture_dil_column, culture_transfer_volume_s1
            ),
            aspirate_shift=[
                0,
                0,
                reservoir_z_shift,
            ],
            mix_at_start=True,
            mix_cycles=num_mixes,
            dispense_height=reservoir_z_shift,
            mix_volume=culture_transfer_volume_s1,
            syringe_speed=25,
        )
        soloSoft.dispense(  # do need to mix at end of transfer
            position="Position4",
            dispense_volumes=Plate_96_Corning_3635_ClearUVAssay().setColumn(
                i, culture_transfer_volume_s1
            ),
            mix_at_finish=True,
            mix_cycles=num_mixes,
            mix_volume=growth_plate_mix_volume_s1,
            aspirate_height=flat_bottom_z_shift,
            dispense_shift=[0, 0, flat_bottom_z_shift],
            syringe_speed=25,
        )

        soloSoft.aspirate(  # well in second half
            position="Position7",
            aspirate_volumes=Reservoir_12col_Agilent_201256_100_BATSgroup().setColumn(
                culture_dil_column, culture_transfer_volume_s1
            ),
            aspirate_shift=[
                0,
                0,
                reservoir_z_shift,
            ],
            mix_at_start=True,
            mix_cycles=num_mixes,
            dispense_height=reservoir_z_shift,
            mix_volume=culture_transfer_volume_s1,
            syringe_speed=25,
        )
        soloSoft.dispense(  # do need to mix at end of transfer
            position="Position4",
            dispense_volumes=Plate_96_Corning_3635_ClearUVAssay().setColumn(
                6 + i, culture_transfer_volume_s1
            ),
            mix_at_finish=True,
            mix_cycles=num_mixes,
            mix_volume=growth_plate_mix_volume_s1,
            aspirate_height=flat_bottom_z_shift,
            dispense_shift=[0, 0, flat_bottom_z_shift],
            syringe_speed=25,
        )

    soloSoft.shuckTip()
    soloSoft.savePipeline()