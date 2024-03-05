"""
Main application for testing instruments at RPL
"""

from pathlib import Path

import wei


def main():

    experiment = wei.ExperimentClient(
        server_addr="mj.cels.anl.gov",
        server_port="8000",
        experiment_name="RPL Module test",
    )

    wf_dir = Path(__file__).parent.absolute() / "workflows"

    experiment.start_run(
        workflow_file=wf_dir / "all_instruments.yaml",
        payload={},
        blocking=True,
        simulate=False,
    )


if __name__ == "__main__":
    main()
