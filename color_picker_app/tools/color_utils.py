from itertools import product
from typing import List, Dict, Any, Tuple
import copy


def convert_volumes_to_payload(
    volumes: List[List[float]], curr_wells_used: List[Any]
) -> Tuple[Dict[str, Any], List[Any]]:
    """Chooses wells for the OT2 to use for this iteration and then assign the
    combination of volumes that will be mixed in that well
    @Inputs:
      volumes: The list of mix volumes to try for this experiment iterations
      curr_wells_used: The wells on the plate that have been used so far
    @Outputs:
      payload: A dictionary containing a list of the volumes for each colorand the wells
               They are assigned to.
      curr_wells_used: An updated list of the wells that have been used in the current experiment and plate.
    """

    def new_plate():
        well_rows = ["A", "B", "C", "D", "E", "F", "G", "H"]
        well_cols = [str(elem) for elem in range(1, 13)]
        well_names = ["".join(elem) for elem in product(well_rows, well_cols)]
        return well_names

    well_names = new_plate()
    curr_wells_available = copy.copy(well_names)
    if len(volumes) <= len(well_names) and len(volumes) <= 96 - len(curr_wells_used):
        for i in curr_wells_used:
            curr_wells_available.remove(i)

    a_vol, b_vol, c_vol, d_vol = [], [], [], []
    dest_wells = []
    for color, well in zip(volumes, curr_wells_available):
        # for color, well in zip(volumes, well_names):
        A, B, C, D= color
        a_vol.append(A)
        b_vol.append(B)
        c_vol.append(C)
        d_vol.append(D)
        dest_wells.append(well)
        curr_wells_used.append(well)
    return {
        "color_A_volumes": a_vol,
        "color_B_volumes": b_vol,
        "color_C_volumes": c_vol,
        "color_D_volumes": d_vol,
        "destination_wells": dest_wells
    }, curr_wells_used
