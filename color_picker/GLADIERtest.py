from gladier import GladierBaseClient, generate_flow_definition, GladierBaseTool
from pprint import pprint
import os
import re
import json
import numpy as np
GENERAL_METADATA = {
    "creators": [{"creatorName": "RPL Team"}],
    "publicationYear": "2023",
    "publisher": "Argonne National Lab",
    "resourceType": {
        "resourceType": "Dataset",
        "resourceTypeGeneral": "Dataset"
    },
    "subjects": [{"subject": "SDL"}],
}

def gather_metadata(**data):
    from pathlib import Path
    import os
    import re
    import json
    import numpy as np
    GENERAL_METADATA = {
    "creators": [{"creatorName": "RPL Team"}],
    "publicationYear": "2023",
    "publisher": "Argonne National Lab",
    "resourceType": {
        "resourceType": "Dataset",
        "resourceTypeGeneral": "Dataset"
    },
    "subjects": [{"subject": "SDL"}],
    }
    plates = 1
    target_color = []
    best_run = []
    best_diff = -1
    best_well = ""
    best_color = []
    print(data['make_input'])
    input_path = Path(data['make_input']).expanduser()
    test = []
    for root, dirs, files in os.walk(input_path):
     
        for dir in dirs:
            
            if re.match("run", dir):
                
                with open(input_path / dir / "exp_data.txt") as f:
                    
                    datal = json.loads(f.read())
                    target_color = datal["target_color"]
                    run_best_ind= np.argmin(datal["differences"])
                    run_best_diff = datal["differences"][run_best_ind]
                    if (best_diff < 0 or run_best_diff < best_diff):
                        best_diff = run_best_diff
                        best_well = datal["wells"][run_best_ind]
                        best_color = datal["best_on_plate"]
                        best_run = dir
    c  = {"target_color": target_color, "best_color": best_color, "best_well": best_well, "best_diff": best_diff, "best_run": best_run}
 
    c.update(GENERAL_METADATA)

    pilot = data["pilot"]
    pilot['metadata'] = c

    print(pilot)
    return pilot

    return str(input_path)
@generate_flow_definition
class GatherMetaData(GladierBaseTool):
    funcx_functions = [gather_metadata]
    required_input = [
        'make_input',
        'funcx_endpoint_compute',
        'pilot'
        
    ]

@generate_flow_definition(modifiers={'publish_gather_metadata': {'payload': '$.GatherMetadata.details.result[0]'}})
class EncryptAndTransfer(GladierBaseClient):
    globus_group = 'dda56f31-53d1-11ed-bd8b-0db7472df7d6'
    gladier_tools = [
        GatherMetaData,
        'gladier_tools.publish.Publish'
    ]


folder_path = '/home/rpl/experiments/feb24416/'
folder_path2 = '/home/rpl/experiments/feb24416'
dest_path = 'feb24416'
if __name__ == '__main__':

    flow_input = {
    'input': {
        'make_input': folder_path2,
        'funcx_endpoint_compute':'299edea0-db9a-4693-84ba-babfa655b1be',
        'funcx_endpoint_non_compute':'299edea0-db9a-4693-84ba-babfa655b1be',

        
        'pilot': {
            'dataset': folder_path,
            'index': 'aefcecc6-e554-4f8c-a25b-147f23091944',
            'project': 'reports',
            'source_globus_endpoint': 'eeabbb24-b47d-11ed-a504-1f2a3a60e896',
            'source_collection_basepath': '/home/rpl/',
            'metadata':GENERAL_METADATA,
            'destination':dest_path
        }
        }
    }

    # Create the Client
    publishFlow = EncryptAndTransfer()

    label = 'testPublishTobias'
    # Run the flow
    flow = publishFlow.run_flow(flow_input=flow_input,label=label)

    # Track progress
    action_id = flow['action_id']
    publishFlow.progress(action_id)
    pprint(publishFlow.get_status(action_id))