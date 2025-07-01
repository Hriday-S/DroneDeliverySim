import sys
import numpy as np
import requests
import rasterio
from io import BytesIO
from zipfile import ZipFile
from rasterio.io import MemoryFile
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QRectF
from PyQt5.QtWidgets import QGraphicsRectItem, QGraphicsLineItem, QGraphicsItemGroup
from DroneSim import DroneSim
from DroneSimGUI import DroneSimGUI
from configs import CROP_CODE, CROP_NAME, GRID_SIZE, GRID_RES, NUM_DRONES, NUM_CUSTOMERS, BASE_LOC, COALITION_PERCENT


def fetch_cdl():
    url = "https://nassgeodata.gmu.edu/nass_data_cache/CDL_2024_clip_20250619203210_1734829252.zip"
    response = requests.get(url)
    if not response.ok:
        raise RuntimeError(f"Failed to fetch CDL data: {response.status_code}")
    with ZipFile(BytesIO(response.content)) as z:
        for name in z.namelist():
            if name.endswith(".tif"):
                with z.open(name) as tif:
                    memfile = MemoryFile(tif.read())
                    return memfile
    raise RuntimeError("No GeoTIFF found in ZIP")

def generate_crop_grid(memfile, crop_code):
    with memfile.open() as src:
        data = src.read(1)
        transform = src.transform
        rows, cols = data.shape
        coords = np.array([
            transform * (c + 0.5, r + 0.5)
            for r in range(rows)
            for c in range(cols)
        ])
        values = data.flatten()
        min_x, min_y = coords.min(axis=0)
        max_x, max_y = coords.max(axis=0)
        extent = np.array([max_x - min_x, max_y - min_y])
        extent[extent == 0] = 1e-5
        norm_coords = (coords - [min_x, min_y]) / extent * GRID_SIZE
        label_grid = np.zeros((GRID_RES, GRID_RES), dtype=bool)
        count_grid = np.zeros((GRID_RES, GRID_RES), dtype=int)
        crop_count_grid = np.zeros((GRID_RES, GRID_RES), dtype=int)
        for pt, val in zip(norm_coords, values):
            gx = int(pt[0] / GRID_SIZE * GRID_RES)
            gy = int(pt[1] / GRID_SIZE * GRID_RES)
            if 0 <= gx < GRID_RES and 0 <= gy < GRID_RES:
                count_grid[gy, gx] += 1
                if val == crop_code:
                    crop_count_grid[gy, gx] += 1
        with np.errstate(divide='ignore', invalid='ignore'):
            frac = crop_count_grid / count_grid
            label_grid = (frac >= 0.5) & (count_grid > 0)

        # Coalition selection logic
        farm_indices = np.argwhere(label_grid)
        num_selected = int(len(farm_indices) * COALITION_PERCENT)
        if num_selected == 0:
            raise ValueError("No fields selected for coalition. Adjust COALITION_PERCENT or verify CDL data.")
        selected_indices = farm_indices[np.random.choice(len(farm_indices), num_selected, replace=False)]
        new_label_grid = np.zeros_like(label_grid)
        for y, x in selected_indices:
            new_label_grid[y, x] = True

        return new_label_grid, crop_count_grid

def random_grid_pos(n):
    return np.random.uniform(0, GRID_SIZE, size=(n, 2))

def main():
    print("Fetching and processing CDL data...")
    memfile = fetch_cdl()
    label_grid, field_counts = generate_crop_grid(memfile, CROP_CODE)
    print("Grid ready.")
    field_coords = []
    cell_size = GRID_SIZE / GRID_RES
    for i in range(GRID_RES):
        for j in range(GRID_RES):
            if label_grid[i, j]:
                field_coords.append([
                    j * cell_size + cell_size / 2,
                    i * cell_size + cell_size / 2
                ])
    app = QtWidgets.QApplication(sys.argv)
    sim = DroneSim(NUM_DRONES, NUM_CUSTOMERS, field_coords)
    gui = DroneSimGUI(label_grid, field_counts, sim)
    gui.show()
    sim.timer.start(50)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
