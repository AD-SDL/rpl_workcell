from log_info import get_log_info
import os
from pathlib import Path
from datetime import datetime as dt
import datetime
import json
import csv

c = Path("C:\\Users\\tgins\\Downloads\\experiments\\goodruns")
v = Path("C:\\Users\\tgins\\Downloads\\wei\\.wei\\cp_wf_mixcolor")
wf = Path("../workflows")
logs = {}
with open("test.csv", "w", newline="") as csvfile:
    writetest = csv.writer(csvfile, delimiter=",")
    for i in os.listdir(c):
        print(i)
        print(Path(i) / "results\\exp_data.txt")
        with open(c / Path(i) / "results\\exp_data.txt") as f:
            x = json.loads(f.read())
            N = x["exp_budget"]
            S = x["pop_size"]
            s = x["runs"][0]
            z = get_log_info(v / Path(s["run_label"]), wf / "cp_wf_mixcolor.yaml")[-1][
                "end_time"
            ]
            endpoint = dt.strptime(z, "%Y-%m-%d %H:%M:%S.%f")
            totaldur = dt.strptime(x["total_time"], "%H:%M:%S.%f")
            totaldur = datetime.timedelta(
                hours=totaldur.hour, minutes=totaldur.minute, seconds=totaldur.second
            )
            for run in x["runs"]:
                a = get_log_info(
                    v / Path(run["run_label"]), wf / "cp_wf_mixcolor.yaml"
                )[-1]["end_time"]
                endpoint2 = dt.strptime(a, "%Y-%m-%d %H:%M:%S.%f")
                print(type(endpoint - endpoint2))
                stepdur = totaldur - (endpoint - endpoint2)
                print(run["plate_best_diff"])
                writetest.writerow(
                    [N, S, run["run_number"], run["plate_best_diff"], stepdur]
                )


#  for t in os.listdir(c / i):
#     if not(re.match(str(t), "results")) and not(re.match(str(t), "desktop.ini")):
#         print(v / t)
#         print(t)
#         l = get_log_info(v / t, wf /'cp_wf_mixcolor.yaml')
#         print(l)
#         #datetime.strptime(l[0]["duration"])
#         print(l[0]["duration"])
