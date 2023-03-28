from gladier import GladierBaseClient, generate_flow_definition, GladierBaseTool
def gather_metadata(**data):
    from pathlib import Path
    import json
    GENERAL_METADATA = {
    "creators": [{"creatorName": "RPL Team"}],
    "publicationYear": "2023",
    "publisher": "Argonne National Lab",
    "resourceType": {
        "resourceType": "Dataset",
        "resourceTypeGeneral": "Dataset"
    },
    "subjects": [{"subject": "SDL"}],
    "exp_type": "color_picker"

    }
    print("break here")
    print(data['make_input'])
    input_path = Path(data['make_input']).expanduser()
    with open(input_path / "exp_data.txt") as f:
        datal = json.loads(f.read())
 
    datal.update(GENERAL_METADATA)
    pilot = data["pilot"]
    pilot['metadata'] = datal
    return pilot
@generate_flow_definition
class GatherMetaData(GladierBaseTool):
    funcx_functions = [gather_metadata]
    required_input = [
        'make_input',
        'funcx_endpoint_compute',
        'pilot'
        
    ]


