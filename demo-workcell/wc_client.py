from pathlib import Path
from argparse import ArgumentParser

from rpl_wei.wei_client_base import WEI


def main(args):
    wei = WEI(args.workflow)
    if args.verbose:
        wei.print_flow()
        wei.print_workcell()

    # This might have to happen outside of the users control
    wei.check_modules()
    wei.check_flowdef()

    wei.run_flow()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-wf", "--workflow", help="Path to workflow file", type=Path, required=True)
    parser.add_argument("-v", "--verbose", help="Extended printing options", action="store_true")

    args = parser.parse_args()
    main(args)
