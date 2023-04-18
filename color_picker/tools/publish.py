from gladier import GladierBaseClient, generate_flow_definition, GladierBaseTool
from tools.gather_data import GatherMetaData
from pathlib import Path

@generate_flow_definition(modifiers={'publish_gather_metadata': {'payload': '$.GatherMetadata.details.result[0]'}})
class PublishRun(GladierBaseClient):
    globus_group = 'dda56f31-53d1-11ed-bd8b-0db7472df7d6'
    gladier_tools = [
        GatherMetaData,
        'gladier_tools.publish.Publish'
    ]

def publish_iter(folder_path, dest_path):
        # Gather some information and transfer it to the endpoint
        print(str(folder_path.expanduser()))
        print(str(dest_path ))    
        flow_input = {
            'input': {
                'make_input': str(folder_path.expanduser()),
                'funcx_endpoint_compute':'9e370560-9463-4a3d-a836-4db1dfb9ccb6',
                'funcx_endpoint_non_compute':'9e370560-9463-4a3d-a836-4db1dfb9ccb6',
                'pilot': {
                    'dataset': str(folder_path.expanduser()),
                    'index': 'aefcecc6-e554-4f8c-a25b-147f23091944',
                    'project': 'reports',
                    'source_globus_endpoint': '6e245524-d967-11ed-9720-e54704575ba0',
                    'source_collection_basepath': '/',
                    'metadata': {},
                    'destination':str(dest_path)
                }
            }
        }

        # Create the Client
        publishFlow = PublishRun()
        label = 'ColorPickerPublish'
        # Run the flow
        flow = publishFlow.run_flow(flow_input=flow_input,label=label)
        # Track progress
        # action_id = flow['action_id']
        # publishFlow.progress(action_id)
     
