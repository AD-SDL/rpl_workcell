from math import floor
from random import sample, choice
from typing import List, Tuple, Union, Optional, Any

from pydantic import BaseModel
import pathlib
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
# https://python-colormath.readthedocs.io/en/latest/color_objects.html
from colormath.color_objects import sRGBColor, LabColor
from colormath.color_conversions import convert_color
from colormath.color_diff import delta_e_cie2000



# def patch_asscalar(a):
    # return a.item()

# setattr(np, "asscalar", patch_asscalar)

class BestColor(BaseModel):
    color: List[float]
    location: str
    population_index: int
    diff_to_target: float = float("inf")


class Solver:
    model_name: str =  'Generic Model'
    def __init__(self) -> None:
        pass
    def run_iteration(
        self,
        target_color: List[float],
        previous_experiment_colors: Optional[List[List[float]]] = None,
        return_volumes: bool = True,
        return_max_volume: float = 275.0,
        out_dim: Tuple[int] = (96, 3),
        pop_size: int = 96,
        prev_best_color: Optional[List[float]] = None
    ) -> List[List[float]]:

        assert pop_size == out_dim[0], "Population size must equal out_dim[0]"

        target_color = sRGBColor(
            *target_color, is_upscaled=True if max(target_color) > 1 else False
        )

        if previous_experiment_colors is None:
            c_ratios = make_random_plate(dim=out_dim)
            if pop_size >= 3:
                c_ratios[0] = sRGBColor(1, 0, 0)
                c_ratios[1] = sRGBColor(0, 1, 0.01)
                c_ratios[2] = sRGBColor(0.01, 0.01, 0.98)
            if return_volumes:
                return Solver.convert_ratios_to_volumes(c_ratios)
            else:
                return c_ratios

        # Flatten if not already flattened
        previous_experiment_colors = (
            np.asarray(previous_experiment_colors).reshape((-1, 3)).tolist()
        )
        previous_experiment_colors = [
            sRGBColor(*color_ratio, is_upscaled=True if max(color_ratio) > 1 else False)
            for color_ratio in previous_experiment_colors
        ]

        # Find population best color
        best_color_position, _ = Solver._find_best_color(
            previous_experiment_colors, target_color
        )

        # Augment
        new_population = self._augment(
            previous_experiment_colors, pop_size, best_color_position, target_color
        )

        # Convert to volumes
        if return_volumes:
            return Solver.convert_ratios_to_volumes(
                new_population, return_max_volume
            )

        return [c.get_value_tuple() for c in new_population]

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

    @staticmethod
    def _find_best_color(
        experiment_colors: List[Union[sRGBColor, List[float]]],
        target_color: List[Union[sRGBColor, List[float]]],
        best_color: Optional[List[Union[sRGBColor, List[float]]]] = None
    ) -> Tuple[int, List[float]]:
        """returns index of best color in population

        Parameters
        ----------
        experiment_colors : List[sRGBColor]
            List of colors in population
        target_color : List[sRGBColor]
            Target color

        Returns
        -------
        int
            Index of best color in population
        List[float]:
            Difference scores for all colors
        """

        if not isinstance(target_color, sRGBColor):
            target_color = sRGBColor(
                *target_color, is_upscaled=True if max(target_color) > 1 else False
            )

        if not all(
            [isinstance(exp_color, sRGBColor) for exp_color in experiment_colors]
        ):
            experiment_colors = [
                sRGBColor(
                    *exp_color,
                    is_upscaled=True if max(exp_color) > 1 else False,
                )
                for exp_color in experiment_colors
            ]
        plate_diffs = np.array(
                Solver._grade_population(
                    experiment_colors, target_color
                )
        )
        return np.argmin(plate_diffs), plate_diffs
        

    @staticmethod
    def _grade_population(
        pop_colors: List[Union[sRGBColor, List[float]]],
        target: Union[sRGBColor, List[float]],
    ):

        if not isinstance(target, sRGBColor):
            target = sRGBColor(*target, is_upscaled=True if max(target) > 1 else False)
        if not all([isinstance(exp_color, sRGBColor) for exp_color in pop_colors]):
            pop_colors = [
                sRGBColor(
                    *exp_color,
                    is_upscaled=True if max(exp_color) > 1 else False,
                )
                for exp_color in pop_colors
            ]

        diffs = []
        for color in pop_colors:
            diff = Solver._color_diff(target, color)
            diffs.append(diff)

        return diffs

    @staticmethod
    def _color_diff(
        color1_rgb: Union[sRGBColor, List[float]],
        color2_rgb: Union[sRGBColor, List[float]],
    ) -> float:

        if not isinstance(color1_rgb, sRGBColor):
            color1_rgb = sRGBColor(
                *color1_rgb, is_upscaled=True if max(color1_rgb) > 1 else False
            )
        if not isinstance(color2_rgb, sRGBColor):
            color2_rgb = sRGBColor(
                *color2_rgb, is_upscaled=True if max(color2_rgb) > 1 else False
            )

        color1_lab = convert_color(color1_rgb, LabColor)
        color2_lab = convert_color(color2_rgb, LabColor)
        delta_e = delta_e_cie2000(color1_lab, color2_lab)
        return delta_e
    
    def _augment(
        self,
        pop: List[sRGBColor],
        new_pop_size: int,
        previous_best_index: Optional[int] = None,
        target_color=None
    ) -> List[sRGBColor]:
        new_pop = []
        n = new_pop_size
        if previous_best_index is not None:
            n -= 1  # Save one spot for the best color
            new_pop.append(pop[previous_best_index])
        # combine colors towards average
        for _ in range(floor(n // 3)):
            t1, t2 = sample(pop, 2)
            t1_ratios = t1.get_value_tuple()
            t2_ratios = t2.get_value_tuple()
            new_color_ratio = (
                ((np.asarray(t1_ratios) + np.asarray(t2_ratios)) / 2).round(3).tolist()
            )
            new_pop.append(sRGBColor(*new_color_ratio))

        # shift some values up or down
        for _ in range(floor(n // 3)):
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

        for _ in range(len(new_pop), new_pop_size):
            new_pop.append(_random_init())
        if previous_best_index is None and len(new_pop) >= 3:
            new_pop[0] = sRGBColor(0.98, 0.01, 0.01)
            new_pop[1] = sRGBColor(0.01, 0.98, 0.01)
            new_pop[2] = sRGBColor(0.01, 0.01, 0.98)
        return new_pop
    
    @staticmethod
    def plot_diffs(
        difflist: List[List[float]],
        exp_folder: Any
    ) -> Any:
        a = []
        print(range(1, len(difflist)+1))
        for i in difflist:
            if True: #a == [] or min(i) < min(a):
                a.append(min(i))
        plt.figure()
        a.sort(reverse=True)
        plt.plot(range(1, len(a)+1), a)
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
    solver = Solver

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
