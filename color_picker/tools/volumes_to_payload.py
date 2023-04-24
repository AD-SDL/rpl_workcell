from itertools import product
from typing import List, Dict, Any, Tuple
from copy import copy
def convert_volumes_to_payload(volumes: List[List[float]], curr_wells_used: List[Any]) -> Tuple[Dict[str, Any], List[Any]]:
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
        
    r_vol, g_vol, b_vol = [], [], []
    dest_wells = []
    for color, well in zip(volumes, curr_wells_available):
    # for color, well in zip(volumes, well_names):
        r, g, b = color
        r_vol.append(r)
        g_vol.append(g)
        b_vol.append(b)
        dest_wells.append(well)
        curr_wells_used.append(well)
    return {
        "red_volumes": r_vol,
        "green_volumes": g_vol,
        "blue_volumes": b_vol,
        "destination_wells": dest_wells,
    }, curr_wells_used