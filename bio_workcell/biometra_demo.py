import logging
from pathlib import Path
from rpl_wei.wei_workcell_base import WEI

def main():
    wf_path = Path('/home/rpl/workspace/rpl_workcell/bio_workcell/workflows/biometra_test.yaml')

    wei_client = WEI(wf_config = wf_path.resolve(), workcell_log_level= logging.ERROR, workflow_log_level=logging.ERROR)


    payload={
        'program':4,
        }

    run_info = wei_client.run_workflow(payload=payload)
    print(run_info)
    ## ADD PUBLISH HERE!!

if __name__ == "__main__":
    main()