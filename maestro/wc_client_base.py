from pathlib import Path
from argparse import ArgumentParser

from devtools import debug

from data_classes import WorkCell


class wc_client:
    def __init__(self, wc_config_file):
        self.state = None

        self.workflow = WorkCell.from_yaml(wc_config_file)
        self.modules = self.workflow.modules
        self.actions = self.workflow.actions

    def check_modules(self):
        for module in self.modules:
            print(f"Checking module: {module}")

    def check_actions(self):
        for step in self.actions:
            print(f"Checking step: {step}")

    def run_flow(self):

        for step in self.actions:
            print(step)

    def print_flow(self):
        debug(self.workflow)


def main(args):
    wc = wc_client(args.workflow)
    wc.print_flow()
    wc.check_modules()
    wc.check_actions()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-wf", "--workflow", help="Path to workflow file", type=Path, required=True)

    args = parser.parse_args()
    main(args)
