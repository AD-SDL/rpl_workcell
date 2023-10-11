from gladier import GladierBaseClient, generate_flow_definition
from tools.gather_data_v2 import GatherMetaData


# @generate_flow_definition()
# class PublishRun(GladierBaseClient):
#     globus_group = "dda56f31-53d1-11ed-bd8b-0db7472df7d6"
#     gladier_tools = [GatherMetaData, "gladier_tools.publish.Publishv2"]
@generate_flow_definition(
    modifiers={
        "publishv2_gather_metadata": {"payload": "$.GatherMetadata.details.result[0]"}
    }
)
class PublishRun(GladierBaseClient):
    globus_group = "dda56f31-53d1-11ed-bd8b-0db7472df7d6"
    gladier_tools = [GatherMetaData, "gladier_tools.publish.Publishv2"]

def publish_iter(folder_path, dest_path, exp):
    print(str(folder_path))
    print(str(dest_path))
    flow_input = {
        "input": {
            
            "data_folder": str(folder_path),
            "compute_endpoint": "9e370560-9463-4a3d-a836-4db1dfb9ccb6",
            "publishv2": {
                "dataset": str(folder_path),
                "index": "aefcecc6-e554-4f8c-a25b-147f23091944",
                "project": "reports",
                "compute_endpoint": "9e370560-9463-4a3d-a836-4db1dfb9ccb6",
                "source_collection": "c6480a47-e864-11ed-9a66-83ef71fbf0ae",
                "source_collection_basepath": "/",
                "destination_collection": "bb8d048a-2cad-4029-a9c7-671ec5d1f84d",
                "metadata_file": str(folder_path / "metadata.json"),
                "ingest_enabled": True,
                "transfer_enabled": True,
                "destination": str("/portal/reports/" + str(dest_path)),
                "visible_to": ["public"],
            },
        }
    }

    # Create the Client
    publishFlow = PublishRun()
    label = "ColorPickerRPL"
    # Run the flow
    flow = publishFlow.run_flow(flow_input=flow_input, label=label)
    #exp.events.log_globus_flow(label, flow["action_id"])
    # Track progress
    # action_id = flow['action_id']
    # publishFlow.progress(action_id)
