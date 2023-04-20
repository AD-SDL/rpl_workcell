import yaml
import os
from datetime import datetime
def get_log_info(run_path, ptcl):
        lineiter=0
        steps_run = []
        with open(ptcl, 'r') as stream:
                wf = yaml.safe_load(stream)
                for test in wf["flowdef"]:
                    steps_run.append(test)
        print("starting")
        while not(os.path.isfile(run_path / "runLogger.log")):
            print("waiting ")
            pass
        with open(run_path/ "runLogger.log") as log:
            lines = log.read().splitlines()
            log.close()
            for i, step in enumerate(steps_run):
                starttime = []
                while starttime == [] and lineiter < len(lines):
                    line = lines[lineiter]
                    columns = [col.strip() for col in line.split(':') if col]
                    if columns[-2] == 'Started running step with name' and columns[-1] == step["name"]:
                        starttime = datetime.strptime(line[0:23], '%Y-%m-%d %H:%M:%S,%f')
                    lineiter += 1
                endtime = []
                while endtime == [] and lineiter < len(lines):
                    line = lines[lineiter]
                    columns = [col.strip() for col in line.split(':') if col]
                    if columns[-2] == 'Finished running step with name' and columns[-1] == step["name"]:
                        endtime = datetime.strptime(line[0:23], '%Y-%m-%d %H:%M:%S,%f')
                    lineiter += 1
                steps_run[i]["start_time"] = str(starttime)
                steps_run[i]["end_time"] = str(endtime)
                steps_run[i]["duration"] = str(endtime-starttime)
        return steps_run
