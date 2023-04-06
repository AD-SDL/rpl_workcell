from gladier import GladierBaseClient, generate_flow_definition, GladierBaseTool

def gather_metadata(**data):

    from pathlib import Path
    import json
    import os
    import csv
    import re
    GENERAL_METADATA = {
    "creators": [{"creatorName": "BIO Team"}],
    "publicationYear": "2023", 
    "publisher": "Argonne National Lab",
    "resourceType": {
        "resourceType": "Dataset",
        "resourceTypeGeneral": "Dataset"
    },
    "subjects": [{"subject": "SDL"}],
    "exp_type": "Campaign2"

    }

    input_path = Path(data['make_input']).expanduser()
    datal = {}
    for file in os.listdir(input_path):
     if re.match(".*csv", file):
       with open(input_path / file) as f:
          reader = csv.reader(f)
          vals = []
          for row in reader:
             vals.append(row)
          datal["csvdata"] = vals
     else:
       with open(input_path / file) as f:
         datal[file] =  f.read()

    GENERAL_METADATA.update(datal)
    final_data = data["publishv2"]
    final_data['metadata'] = GENERAL_METADATA
    return final_data

@generate_flow_definition
class GatherMetaData(GladierBaseTool):
    funcx_functions = [gather_metadata]
    required_input = [
        'make_input',
        'funcx_endpoint_compute'
    ]
