from gladier import GladierBaseClient, generate_flow_definition, GladierBaseTool


def gather_metadata(**data):
    from pathlib import Path
    import json

    GENERAL_METADATA = {
        "creators": [{"creatorName": "RPL Team"}],
        "publicationYear": "2023",
        "publisher": "Argonne National Lab",
        "resourceType": {"resourceType": "Dataset", "resourceTypeGeneral": "Dataset"},
        "exp_type": "pcr",
        "exp_label": "first_image_test",
        "subjects": [{"subject": "SDL"}],
    }
    print(data["make_input"])
    input_path = Path(data["make_input"]).expanduser()
    with open(input_path / "wf_steps.txt") as f:
        datal = {"wf_steps": [json.loads(f.read())]}

    datal.update(GENERAL_METADATA)

    pilot = data["pilot"]
    pilot["metadata"] = datal
    return data["pilot"]
    # data["pilot"]["metadata"].update({"wf_steps": datal})
    # pilot['metadata'] = datal


@generate_flow_definition
class GatherMetaData(GladierBaseTool):
    compute_functions = [gather_metadata]
    required_input = ["make_input", "compute_endpoint", "pilot"]
