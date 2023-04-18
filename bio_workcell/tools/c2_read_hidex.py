from gladier import GladierBaseClient, generate_flow_definition, GladierBaseTool


def excel_to_csv(**data):
    """
    Extracts Raw OD(590) data from Hidex excel file into csv file

    :param str filename: filename of Hidex excel file to convert

    output: path of new csv file (str)

    """
    import os
    from pathlib import Path
    import pandas as pd

    filepath = data.get('local_path')
    filename = data.get('filename')
    sheet_name = "Raw OD(590)"
    csv_filename = None

    if os.path.exists(filename):
        excel_basename = os.path.splitext(os.path.basename(filename))[0]
        csv_filename = excel_basename + "_RawOD.csv"
        csv_filepath = filename.replace(os.path.basename(filename), csv_filename)

    # convert Raw OD(590) excel sheet to new csv file
    excel_OD_data = pd.read_excel(filename, sheet_name=sheet_name, index_col=None)
    excel_OD_data.to_csv(csv_filepath, encoding="utf-8", index=False)

    return csv_filepath



@generate_flow_definition
class C2_read_hidex(GladierBaseTool):
    funcx_functions = [excel_to_csv]
    required_input = [
        'make_input',
        'funcx_endpoint_compute'
    ]
