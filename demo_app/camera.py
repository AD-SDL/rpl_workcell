#!/usr/bin/env python3

from pathlib import Path
from wei.wei_workcell_base import WEI


def main():
    wf_path = Path(
        "/home/rpl/workspace/rpl_workcell/pcr_workcell/workflows/test_camera.yaml"
    )

    wei_client = WEI(wf_path.resolve())

    wf_id = list(wei_client.get_workflows().keys())[0]

    payload = {}
    wei_client.run_workflow(wf_id, payload=payload)


if __name__ == "__main__":
    main()
