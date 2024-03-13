import os
import cv2
import numpy as np


# code to convert jpeg or png image to npy:
img = cv2.imread(os.path.join("C:\\Users\\990sh\\Desktop", "Picture4.png"))
np.save('smaller_circle.npy', img)


class Map:
    def __init__(self, map_: np.array, inflation=0):
        self.map = map_
        self.inflation = inflation
        self.inflated_map = self.inflate_map(inflation)

    def inflate_map(self, inflation):
        original_map = self.map.copy()
        inflated_map = self.map.copy()
        rows, cols, temp = inflated_map.shape
        for j in range(cols):
            for i in range(rows):
                if not original_map[i, j].any():
                    i_min = max(0, i - inflation)
                    i_max = min(rows, i + inflation)
                    j_min = max(0, j - inflation)
                    j_max = min(cols, j + inflation)
                    inflated_map[i_min:i_max, j_min:j_max] = 100
        return inflated_map