from log_info import get_log_info
import os
from pathlib import Path
import re
from datetime import datetime as dt
import datetime
import json
import time
import csv
c = Path("~/goodruns").expanduser()
print(c.expanduser())
v = Path("~/.wei/cp_wf_mixcolor").expanduser()
wf = Path('../workflows')
logs    = {}
with open("test4.csv", "w", newline='') as csvfile:
    writetest = csv.writer(csvfile, delimiter=',')
    print(os.listdir(c).sort())
    print("asdfsasdf")
    for i in os.listdir(c):
        #print(i)
        #print(Path(i) / "results/exp_data.txt" )
        with open(c /Path(i) / "results/exp_data.txt") as f:
            x = json.loads(f.read())
            N = x["exp_budget"]
            S = x["pop_size"]
            s = x["runs"][0]
            z = get_log_info(v / Path(s["run_label"]), wf /'cp_wf_mixcolor.yaml')[-1]["end_time"]
            endpoint = dt.strptime(z, '%Y-%m-%d %H:%M:%S.%f')
            totaldur = dt.strptime(x["total_time"], '%H:%M:%S.%f')
            totaldur = datetime.timedelta(hours=totaldur.hour, minutes=totaldur.minute, seconds=totaldur.second)
            runlist = x["runs"]
            runlist.reverse()
            mindiff=1000
            for run in runlist:
                a = get_log_info(v / Path(run["run_label"]), wf /'cp_wf_mixcolor.yaml')[-1]["end_time"]
                endpoint2 = dt.strptime(a, '%Y-%m-%d %H:%M:%S.%f')
                #print(type(endpoint-endpoint2))
                stepdur = totaldur-(endpoint-endpoint2)
               # print(run["plate_best_diff"])
                if (run["plate_best_diff"] < mindiff):
                    mindiff = run["plate_best_diff"]
                    best_so_far = run["best_so_far"]
                writetest.writerow([N, S, run["run_number"], run["plate_best_diff"], run["best_so_far"], mindiff, stepdur, ])
#  for t in os.listdir(c / i):
#     if not(re.match(str(t), "results")) and not(re.match(str(t), "desktop.ini")):
#         print(v / t)
#         print(t)
#         l = get_log_info(v / t, wf /'cp_wf_mixcolor.yaml')
#         print(l)
#         #datetime.strptime(l[0]["duration"])
#         print(l[0]["duration"])