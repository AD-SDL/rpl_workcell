from math import floor
from random import sample, choice
from typing import List, Tuple, Union

from pydantic import BaseModel
import numpy as np
import matplotlib.pyplot as plt

# https://python-colormath.readthedocs.io/en/latest/color_objects.html
from colormath.color_objects import sRGBColor
from solvers.solver import Solver, patch_asscalar

setattr(np, "asscalar", patch_asscalar)

class BestColor(BaseModel):
    color: List[float]
    location: str
    population_index: int
    diff_to_target: float = float("inf")


def make_random_plate(dim: Tuple[int] = ()) -> List[List[List[float]]]:
    total = np.prod(dim)
    return np.random.random(size=total).reshape(dim).tolist()


class EvolutionaryColorSolver(Solver):
    def __init__(self, pop_size) -> None:
        super().__init__(pop_size)

    def _augment(
        self,
        prev_pop: List[sRGBColor],
        prev_grades: int,
    ) -> List[float]:
        new_pop = []
        n = self.pop_size
        previous_best_index = np.argmax(prev_grades)
        # combine colors towards average
        for _ in range(floor(n // 3)):
            t1, t2 = sample(prev_pop, 2)

            new_color_ratio = ((np.asarray(t1) + np.asarray(t2)) / 2).round(3).tolist()
            new_pop.append((new_color_ratio))

        # shift some values up or down
        for _ in range(floor(n // 3)):
            t_color_ratios = choice(prev_pop)

            new_color_ratio = []
            # randomly shift some of the values up or down
            for r in t_color_ratios:
                if np.random.uniform(0, 1) > 0.5:
                    delta = np.random.uniform(-r, 1 - r)
                else:
                    delta = 0

                new_color_ratio.append(round(r + delta, 3))
            new_pop.append(new_color_ratio)

        # generate new randoms
        def _random_init():
            return np.random.rand(4).round(3).tolist()

        for _ in range(len(new_pop), self.pop_size):
            new_pop.append(_random_init())
        new_pop = [x / np.sum(x) for x in new_pop]
        return new_pop

    @staticmethod
    def convert_ratios_to_volumes(
        color_ratios: List[List[Union[sRGBColor, float]]],
        total_volume: float = 275.0,
    ) -> List[List[float]]:
        sanitized_colors = []
        for color in color_ratios:
            if not isinstance(color, sRGBColor):
                sanitized_colors.append(sRGBColor(*color))
            else:
                sanitized_colors.append(color)

        volume_list = []
        for color in sanitized_colors:
            color_ratio = np.asarray(color.get_value_tuple())
            color_ratio /= sum(color_ratio)
            volume_list.append([r * total_volume for r in color_ratio])

        return volume_list


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    show_visual = True
    print_color = True

    target_ratio = [237, 36, 36]
    mixing_colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]
    solver = EvolutionaryColorSolver

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
