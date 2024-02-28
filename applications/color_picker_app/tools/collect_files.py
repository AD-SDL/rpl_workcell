from typing import List, Dict, Any, Tuple
import copy
from pathlib import Path
from wei import ExperimentClient
import shutil


def collect_files(
        experiment: ExperimentClient,
        img_path: str,
        plate_n: int
):
    img_path = Path(img_path).parent / "plate_only.jpg"
    
    shutil.copy2(experiment.output_dir + "/exp_data.txt", experiment.working_dir / "publish/exp_data.txt")
    shutil.copy2(img_path, experiment.working_dir / ("publish/plate_" +str(plate_n)+".jpg"))
    
    