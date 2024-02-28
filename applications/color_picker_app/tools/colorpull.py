from pathlib import Path
import json
import csv

with open("S16ColorData.csv", "w", newline="") as csvfile:
    writetest = csv.writer(csvfile, delimiter=",")

    exp_path = Path("~/experiments/ColorPicker_120_120_120_2023-05-11_16")
    i = 1
    with open(exp_path / "results/exp_data.txt") as f:
        x = json.loads(f.read())
        runs = x["runs"]
        runs.reverse()
        for run in runs:
            c = run["results"]
            v = run["exp_volumes"]
            d = run["differences"]
            for x in range(len(c)):
                test = [i, c[x][0], c[x][1], c[x][2], v[x][0], v[x][1], v[x][2], d[x]]
                writetest.writerow(test)
                i += 1
