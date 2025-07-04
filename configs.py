import numpy as np

config = {
    'CROP_CODE': 5,
    'CROP_NAME': "Soybeans",
    'GRID_SIZE': 100,
    'GRID_RES': 30,
    'NUM_DRONES': 2,
    'NUM_CUSTOMERS': 90,
    'COALITION_PERCENT': 0.03,
    'DRONE_SPEED': 1,
}

# Constants post over-ride
CROP_CODE = config['CROP_CODE']
CROP_NAME = config['CROP_NAME']
GRID_SIZE = config['GRID_SIZE']
GRID_RES = config['GRID_RES']
NUM_DRONES = config['NUM_DRONES']
NUM_CUSTOMERS = config['NUM_CUSTOMERS']
COALITION_PERCENT = config['COALITION_PERCENT']
DRONE_SPEED = config['DRONE_SPEED']
BASE_LOC = np.array([GRID_SIZE / 2, GRID_SIZE / 2])
