#!/usr/bin/env python3

import cv2
import numpy as np
from pathlib import Path
import yaml


class BaseOptimizer(object):
    CARDINALITY_GAIN = None
    TIME_GAIN = None

    def load_params(self):
        raise NotImplementedError

    def load_metadata(self):
        raise NotImplementedError

    def run(self):
        raise NotImplementedError

    @classmethod
    def solve(cls, navigable_area, resolution):
        navigable_area *= np.power(resolution, 2)
        
        n_agents = np.ceil(
            np.sqrt(cls.TIME_GAIN * navigable_area / cls.CARDINALITY_GAIN)
        )
        # Minimum 2 agents are required
        n_agents = np.clip(n_agents, a_min=2, a_max=10).astype(dtype=np.uint8)

        print(f"Solved. N_AGENTS: {n_agents}")
        return n_agents
        # return 1


class OptimizerFromImage(BaseOptimizer):
    def __init__(self, map_yaml_path: str, survey_area: int):
        super(OptimizerFromImage, self).__init__()

        self._map_file = map_yaml_path
        self._metadata = None
        
        self._survey_area = survey_area

        self.load_params()
        self.load_metadata()

    def load_params(self):
        OptimizerFromImage.CARDINALITY_GAIN = 100
        OptimizerFromImage.TIME_GAIN = 1

        print(f"Optimizing from image: {self._map_file}")

    def load_metadata(self):
        with open(self._map_file) as fp:
            self._metadata = yaml.safe_load(fp.read())

    def run(self):
        map_img = Path(self._map_file).parent.joinpath(self._metadata["image"])
        assert map_img.exists()
        src = cv2.imread(str(map_img), cv2.IMREAD_GRAYSCALE)

        _, gray = cv2.threshold(
            src, (1 - self._metadata["free_thresh"]) * 255, 255, cv2.THRESH_BINARY
        )
        contours, _ = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        navigable = max(contours, key=lambda c: cv2.contourArea(c))
        area = cv2.contourArea(navigable)

        n_agents = OptimizerFromImage.solve(self._survey_area, self._metadata["resolution"])

        (x, y, w, h) = cv2.boundingRect(navigable)
        cv2.rectangle(gray, (x, y), (x + w, y + h), 255, 1)
        cv2.imwrite("/tmp/optimizer_navigable.png", gray)
        return n_agents


def main():
    OptimizerFromImage()

if __name__ == "__main__":
    main()