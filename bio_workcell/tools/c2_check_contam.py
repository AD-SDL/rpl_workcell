from gladier import GladierBaseClient, generate_flow_definition, GladierBaseTool


def c2_check_contam(**data):
    import pandas as pd
    import json
    """check_for_contaminaton

    Description: checks data from all timepoints for contaminated blanks. 
        (All wells in Row H are blanks)

    Parameters: 
        raw_df = dataframe of raw OD(590) absorbance readings
        timepoint_list = list of timepoints at which the data was collected 
        (both parsed directly from hidex csv data)

    Returns: 
        ret_val: TODO
    """ 
    
    ret_val = "PASS"
    print("TODO: reformat campaign2 qc check")  #TEST
    raw_df = pd.Dataframe.from_dict(data["pandas"])
    timepoint_list = data["timestamp_list"]
    blanks = raw_df.iloc[84:,3:]
    numpy_blanks = blanks.to_numpy()
    flat_numpy_blanks = numpy_blanks.ravel().tolist()
    
    for blank_raw_OD in flat_numpy_blanks: 
        if float(blank_raw_OD) > 0.07: 
            ret_val == "FAIL"
            print(f"FAIL control sample {blank_raw_OD} has Raw OD value greater than 0.07")
            # TODO: improve transparency about which sample failed at what timepoint
    data["ret_val"] = ret_val
    #save json.dumps(data)
    return 



@generate_flow_definition
class C2_check_contam(GladierBaseTool):
    funcx_functions = [c2_check_contam]
    required_input = [
       
        'funcx_endpoint_compute'
    ]
