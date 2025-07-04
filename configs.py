import numpy as np

CROP_CODE = 5  # Soybeans
CROP_NAME = "Soybeans"
GRID_SIZE = 100
GRID_RES = 30
NUM_DRONES = 2
NUM_CUSTOMERS = 90
BASE_LOC = np.array([GRID_SIZE / 2, GRID_SIZE / 2])
COALITION_PERCENT = 0.03  # 10% of farmland is in the coalition
DRONE_SPEED = 1