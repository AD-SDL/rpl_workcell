
import os
import sys
import time
import argparse
from liquidhandling import SoloSoft
from liquidhandling import Reservoir_12col_Agilent_201256_100_BATSgroup, Plate_96_Corning_3635_ClearUVAssay, DeepBlock_96VWR_75870_792_sterile


def package_hso(
        create_hso_method,
        payload,
        temp_file_path,
        
):
    """package_hso_1

    Description: Calls method to create hso then reads contents into string and counts num lines

    Args:
        payload (_type_): _description_
        temp_file_path (_type_): _description_

    Raises:
        TODO 

    Returns:
        hso_contents: (str) contents of new hso file produced
        hso_num_lines: (int) number of lines in new hso file produced
    """
    
    try: 
        # generate hso file at temp file path
        create_hso_method(payload=payload, temp_file_path=temp_file_path)
    except Exception as erorr_msg: 
        # TODO 
        print("Could not create hso at specified temp file path")
        raise error_msg

    # extract text and number of lines from new hso file
    hso_contents = ""
    hso_num_lines = 0
    try: 
        with open(temp_file_path, 'r') as hso: 
            for line in hso: 
                hso_contents += line 
                hso_num_lines += 1
    except Exception as error_msg:
        # TODO
        print("Could not read in contents of new hso file")
        raise error_msg
    
    # delete temp hso file 
    try: 
        os.remove(temp_file_path)
    except FileNotFoundError as error_msg: 
        # TODO
        raise error_msg
    except Exception as error_msg: 
        # TODO
        raise error_msg

    return hso_contents, hso_num_lines    
            
