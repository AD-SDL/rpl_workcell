from random import choice
from typing import List, Tuple, Optional, Any
from solvers.solver import Solver, patch_asscalar
from pydantic import BaseModel

import numpy as np

# https://python-colormath.readthedocs.io/en/latest/color_objects.html
from colormath.color_objects import sRGBColor
import matplotlib.pyplot as plt

setattr(np, "asscalar", patch_asscalar)


class BestColor(BaseModel):
    color: List[float]
    location: str
    population_index: int
    diff_to_target: float = float("inf")


class AggroColorSolver(Solver):
    def __init__(self) -> None:
        super().__init__()

    def _augment(
        self,
        pop: List[sRGBColor],
        new_pop_size: int,
        previos_best_index: Optional[int] = None,
        prev_best_diff: Optional[float] = None,
    ) -> List[sRGBColor]:
        new_pop = []

        n = new_pop_size
        if previos_best_index is not None:
            sample_pop = [pop[previos_best_index] for x in range(n)]
            print("prevbest")
        else:
            sample_pop = choice(pop, n)

        # combine colors towards average

        # shift some values up or down

        for color in sample_pop:
            t_color_ratios = color.get_value_tuple()

            new_color_ratio = []
            # randomly shift some of the values up or down
            lim = min(prev_best_diff / 200, 0.2)
            vals = np.random.rand(3)
            vals = vals / sum(vals)

            i = 0
            for r in t_color_ratios:
                delta = vals[i] * np.random.choice([-1, 1]) * lim
                new_color_ratio.append(round(np.clip(r + delta, 0.01, 1), 3))
                i += 1

            new_pop.append(sRGBColor(*new_color_ratio))

        # generate new randoms

        def _random_init():
            return sRGBColor(*np.random.rand(3).round(3).tolist())

        for _ in range(len(new_pop), new_pop_size):
            new_pop.append(_random_init())
            print("runninghere")
        if previos_best_index is None and len(new_pop) >= 3:
            new_pop[0] = sRGBColor(0.98, 0.01, 0.01)
            new_pop[1] = sRGBColor(0.01, 0.98, 0.01)
            new_pop[2] = sRGBColor(0.01, 0.01, 0.98)
        return new_pop

    @staticmethod
    def plot_diffs(difflist: List[List[float]], exp_folder: Any) -> Any:
        a = []
        print(range(1, len(difflist) + 1))
        for i in difflist:
            if a == [] or min(i) < min(a):
                a.append(min(i))
        plt.figure()
        plt.plot(range(1, len(difflist) + 1), a)
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

    show_visual = True
    print_color = True

    target_ratio = [237, 36, 36]
    mixing_colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]
    solver = AggroColorSolver

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
