from typing import List, Tuple, Any

from pydantic import BaseModel

import numpy as np


# https://python-colormath.readthedocs.io/en/latest/color_objects.html
from colormath.color_objects import sRGBColor
from skopt import Optimizer
from solvers.solver import Solver, patch_asscalar
import matplotlib.pyplot as plt


class BestColor(BaseModel):
    color: List[float]
    location: str
    population_index: int
    diff_to_target: float = float("inf")


setattr(np, "asscalar", patch_asscalar)


class BayesColorSolver(Solver):
    def __init__(self, pop_size) -> None:
        self.optimizer = Optimizer(
            dimensions=[(0.0, 1.0), (0.0, 1.0), (0.0, 1.0), (0.0, 1.0)],
            # base_estimator='GP',
            n_initial_points=4,
            initial_point_generator="random",
            # acq_func='EI',
            # acq_optimizer='sampling',
        )
        super().__init__(pop_size=pop_size)

    def _augment(
        self,
        prev_pop: List[sRGBColor],
        prev_grades: int,
    ) -> List[float]:
        print(prev_pop)
        self.optimizer.tell(prev_pop, prev_grades)
        print("start")
        new_pop = self.optimizer.ask(self.pop_size)
        new_pop = [(x / np.sum(x)).round(3).tolist() for x in new_pop]
        print("end")
        return new_pop

    @staticmethod
    def plot_diffs(difflist: List[List[float]], exp_folder: Any) -> Any:
        a = []
        print(range(1, len(difflist) + 1))
        for i in difflist:
            if True:  # a == [] or min(i) < min(a):
                a.append(min(i))
        plt.figure()
        a.sort(reverse=True)
        plt.plot(range(1, len(a) + 1), a)
        plt.xlabel("Color Rank")
        plt.ylabel("Color Difference")
        plt.title("Loss Graph")
        print(exp_folder / "results" / "convergence_graph.png")
        plt.savefig(exp_folder / "results" / "convergence_graph.png", dpi=300)
        return a


def make_random_plate(dim: Tuple[int] = ()) -> List[List[List[float]]]:
    total = np.prod(dim)
    return np.random.random(size=total).reshape(dim).tolist()


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    show_visual = False
    print_color = True

    target_ratio = [237, 36, 36]
    mixing_colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]
    solver = BayesColorSolver()

    init_guesses = make_random_plate(dim=(1, 3, 3))
    if show_visual:
        plt.imshow(init_guesses)
        plt.show()
    print("here")
    cur_plate = init_guesses
    best_color = None
    best_diff = float("inf")
    for iter in range(4):
        new_plate = solver.run_iteration(target_ratio, cur_plate, return_volumes=False)
        print("start")
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
