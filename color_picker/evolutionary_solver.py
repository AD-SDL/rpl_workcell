import string
from random import sample, choice
from typing import List, Tuple, Union, Optional

from pydantic import BaseModel

import numpy as np

# https://python-colormath.readthedocs.io/en/latest/color_objects.html
from colormath.color_objects import sRGBColor, LabColor
from colormath.color_conversions import convert_color
from colormath.color_diff import delta_e_cie2000

from plate_color_analysis import get_colors


class BestColor(BaseModel):
    color: List[float]
    location: str
    population_index: int
    diff_to_target: float = float("inf")


class EvolutionaryColors:
    def __init__(
        self,
        target: List[float],
        mixing_colors: List[List[float]],
        starting_ratios=None,
        pop_size=96,
        color_diff_threshold=5.0,
    ) -> None:

        self.target = sRGBColor(*target, is_upscaled=True if max(target) > 1 else False)
        self.mixing_colors = [
            sRGBColor(*col, is_upscaled=True if max(col) > 1 else False)
            for col in mixing_colors
        ]
        self.starting_ratios = starting_ratios if starting_ratios else [0.5, 0.5, 0.5]
        self.pop_size = pop_size
        self.color_diff_threshold = color_diff_threshold

        self.population_history = []
        self.current_best_color: BestColor = BestColor(
            color=[0.0, 0.0, 0.0], location="None", population_index=-1
        )

    def run_iteration(
        self,
        experiment_colors: Optional[List[List[float]]] = None,
        return_volumes: bool = True,
        out_dim: Tuple[int] = (96, 3),
    ) -> List[List[float]]:

        if experiment_colors is None and len(self.population_history) == 0:
            c_ratios = make_random_plate(dim=out_dim)
            if return_volumes:
                return self.convert_ratios_to_volumes(c_ratios)
            else:
                return c_ratios

        assert (
            experiment_colors is not None
        ), "Experiment colors not provided for this iteration..."
        # Flatten if not already flattened
        experiment_colors = np.asarray(experiment_colors).reshape((-1, 3)).tolist()
        experiment_colors = [
            sRGBColor(*color_ratio, is_upscaled=True if max(color_ratio) > 1 else False)
            for color_ratio in experiment_colors
        ]
        # Grade population
        population_grades = self._grade_population(experiment_colors, self.target)

        # Store population history and store current best
        self.population_history.append(
            {"colors": experiment_colors, "scores": population_grades}
        )
        self._find_best_color(experiment_colors, population_grades)
        # Augment
        new_population = self._augment(experiment_colors, self.pop_size)

        # Convert to volumes
        if return_volumes:
            return self.convert_ratios_to_volumes(new_population)

        return [c.get_value_tuple() for c in new_population]

    def convert_ratios_to_volumes(
        self,
        color_ratios: List[List[Union[sRGBColor, float]]],
        total_volume: float = 30.0,
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

    def _find_best_color(
        self, experiment_colors: List[sRGBColor], population_grades: List[float]
    ) -> None:
        if min(population_grades) < self.current_best_color.diff_to_target:
            diff = min(population_grades)
            color_ind = np.argmin(np.array(population_grades))
            color_rgb = experiment_colors[color_ind].get_value_tuple()
            location = "".join(
                [
                    string.ascii_lowercase[color_ind // 12],
                    str((color_ind % 12) + 1),
                ]
            )
            self.current_best_color = BestColor(
                color=color_rgb,
                location=location,
                population_index=len(self.population_history) - 1,
                diff_to_target=diff,
            )

    def _grade_population(self, pop_colors: List[sRGBColor], target: sRGBColor):
        diffs = []
        for color in pop_colors:
            diff = self._color_diff(target, color)
            diffs.append(diff)

        return diffs

    def _color_diff(self, color1_rgb: sRGBColor, color2_rgb: sRGBColor) -> float:
        color1_lab = convert_color(color1_rgb, LabColor)
        color2_lab = convert_color(color2_rgb, LabColor)
        delta_e = delta_e_cie2000(color1_lab, color2_lab)
        return delta_e

    def _augment(self, pop: List[sRGBColor], n: int) -> List[sRGBColor]:
        new_pop = []

        # combine colors towards average
        for _ in range(n // 3):
            t1, t2 = sample(pop, 2)
            t1_ratios = t1.get_value_tuple()
            t2_ratios = t2.get_value_tuple()
            new_color_ratio = (
                ((np.asarray(t1_ratios) + np.asarray(t2_ratios)) / 2).round(3).tolist()
            )
            new_pop.append(sRGBColor(*new_color_ratio))

        # shift some values up or down
        for _ in range(n // 3):
            t_color_ratios = choice(pop).get_value_tuple()

            new_color_ratio = []
            # randomly shift some of the values up or down
            for r in t_color_ratios:
                if np.random.uniform(0, 1) > 0.5:
                    delta = np.random.uniform(-r, 1 - r)
                else:
                    delta = 0

                new_color_ratio.append(round(r + delta, 3))
            new_pop.append(sRGBColor(*new_color_ratio))

        # generate new randoms
        def _random_init():
            return sRGBColor(*np.random.rand(3).round(3).tolist())

        for _ in range(len(new_pop), n):
            new_pop.append(_random_init())

        return new_pop

    def read_picture(self, pic):
        # get_colors(pic) returns 11 possible plates for use in an OT2
        # This script assumes only 1 plate outside at OT2, so replace
        # get_colors(pic) with get_colors(pic)[1]
        return get_colors(pic)[1]


def make_random_plate(dim: Tuple[int] = ()) -> List[List[List[float]]]:
    total = np.prod(dim)
    return np.random.random(size=total).reshape(dim).tolist()


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    show_visual = False
    print_color = True

    target_ratio = [237, 36, 36]
    mixing_colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]
    solver = EvolutionaryColors(target=target_ratio, mixing_colors=mixing_colors)

    init_guesses = make_random_plate(dim=(8, 12))
    if show_visual:
        plt.imshow(init_guesses)
        plt.show()

    cur_plate = init_guesses
    best_color = None
    best_diff = float("inf")
    for iter in range(500):
        new_plate = solver.run_iteration(cur_plate, return_volumes=False)
        new_plate = np.asarray(new_plate).reshape((8, 12, 3)).tolist()
        cur_plate = new_plate
        if print_color:
            cur_color = solver.current_best_color
            if cur_color.diff_to_target < best_diff:
                best_color = cur_color
                best_diff = cur_color.diff_to_target
                print(f"{cur_color}, {iter =}")
        if show_visual:
            f, axarr = plt.subplots(1, 2)
            axarr[0].imshow(new_plate)
            axarr[0].set_title("Experiment plate")
            axarr[1].imshow([[target_ratio]])
            axarr[1].set_title("Target Color")

            plt.show()
