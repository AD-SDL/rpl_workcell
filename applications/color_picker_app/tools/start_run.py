# from wei import WEI
from tools.log_info import get_log_info
from wei import ExperimentClient
from pathlib import Path


def start_run_with_log_scraping(
    protocol, payload, steps_run, experiment: ExperimentClient
):
    """Runs a WEI flow and
    updates the steps that have been run in the experiment
    @Inputs:
        protocol: Filename for which WEI flow to run
        payload: Payload to run that flow with
        steps_run: The steps that have been run so far on this iteration of the experiment
    @Outputs:
        steps run: The list of steps run in this iteration of the experiment including the new ones added
        by the workflow run by this function
        run_info: The WEI output from running the flow"""

    result = experiment.start_run(
        protocol.resolve(), payload=payload, simulate=False, blocking=True
    )
    print(result)
    run_info = result["hist"]
    run_info["run_dir"] = Path(run_info["run_dir"])
    run_log = Path(result["run_log"].replace("/home/app", str(Path.home())))
    t_steps_run = get_log_info(run_log, protocol)
    steps_run.append(t_steps_run)
    return steps_run, result
