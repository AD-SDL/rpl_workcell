from math import floor
from random import sample, choice
from typing import List, Tuple, Union, Optional, Any

from pydantic import BaseModel

import numpy as np


# https://python-colormath.readthedocs.io/en/latest/color_objects.html
from colormath.color_objects import sRGBColor, LabColor
from colormath.color_conversions import convert_color
from colormath.color_diff import delta_e_cie2000
from skopt import Optimizer
from solvers.solver import Solver
import matplotlib.pyplot as plt

class BestColor(BaseModel):
    color: List[float]
    location: str
    population_index: int
    diff_to_target: float = float("inf")




# setattr(np, "asscalar", patch_asscalar)
class BayesColorSolver(Solver):
    def __init__(self) -> None:
        pass
        self.optimizer = Optimizer(dimensions=[(0.0, 1.0), (0.0, 1.0), (0.0, 1.0)],
                        # base_estimator='GP',
                        n_initial_points=4,
                        initial_point_generator='random',
                        # acq_func='EI',
                        # acq_optimizer='sampling',
        )
   

   
    
    def _augment(
        self,
        pop: List[sRGBColor],
        new_pop_size: int,
        previos_best_index: Optional[int] = None,
        target_color = None,
    ) -> List[sRGBColor]:
       grades = Solver._grade_population(pop,  target_color)
       test_pop = [[a for a in x.get_value_tuple()] for x in pop]
       print(test_pop)
       self.optimizer.tell(test_pop, grades)
       new_pop = self.optimizer.ask(new_pop_size)
       return new_pop
    
    @staticmethod
    def plot_diffs(
        difflist: List[List[float]],
        exp_folder: Any
    ) -> Any:
        import pathlib
        from pathlib import Path
        a = []
        print(range(1, len(difflist)+1))
        for i in difflist:
            if a == [] or min(i) < min(a):
                a.append(min(i))
        plt.figure()
        plt.plot(range(1, len(difflist)+1), a)
        plt.xlabel("Color Rank")
        plt.ylabel("Color Difference")
        plt.title("Loss Graph")
        print(exp_folder/"results" / "convergence_graph.png")
        plt.savefig(exp_folder/"results" / "convergence_graph.png", dpi=300)
        return a

def make_random_plate(dim: Tuple[int] = ()) -> List[List[List[float]]]:
    total = np.prod(dim)
    return np.random.random(size=total).reshape(dim).tolist()


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    show_visual = True
    print_color = True

    target_ratio = [237, 36, 36]
    mixing_colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]
    solver = BayesColorSolver()

    init_guesses = make_random_plate(dim=(8, 12, 3))
    if show_visual:
        plt.imshow(init_guesses)
        plt.show()

    cur_plate = init_guesses
    best_color = None
    best_diff = float("inf")
    for iter in range(4):
        new_plate = solver.run_iteration(target_ratio, cur_plate, return_volumes=False)
        new_plate = np.asarray(new_plate).reshape((8, 12, 3)).tolist()
        cur_plate = new_plate
        if print_color:
            plate_best_color_ind = solver._find_best_color(
                np.asarray(cur_plate).reshape((-1, 3)).tolist(), target_ratio
            )
            row = plate_best_color_ind // 12
            col = plate_best_color_ind % 12
            plate_best_color = cur_plate[row][col]

            plate_best_color_diff = solver._color_diff(plate_best_color, target_ratio)
            if plate_best_color_diff < best_diff:
                best_color = plate_best_color
                best_diff = plate_best_color_diff
                print(f"{plate_best_color}, {iter =}, diff = {plate_best_color_diff}")

        if show_visual:
            f, axarr = plt.subplots(1, 2)
            axarr[0].imshow(new_plate)
            axarr[0].set_title("Experiment plate")
            axarr[1].imshow([[target_ratio]])
            axarr[1].set_title("Target Color")

            plt.show()
