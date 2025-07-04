import sys
import numpy as np
import requests
import rasterio
from io import BytesIO
from zipfile import ZipFile
from rasterio.io import MemoryFile
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QGraphicsRectItem, QGraphicsLineItem, QGraphicsItemGroup
from DroneSim import DroneSim
from DroneSimGUI import DroneSimGUI
from utils import random_grid_pos
import argparse
import configs
import datetime
import time 

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

def generate_crop_grid(memfile, crop_code, grid_size, grid_res, coalition_percent):
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

        norm_coords = (coords - [min_x, min_y]) / extent * grid_size
        label_grid = np.zeros((grid_res, grid_res), dtype=bool)
        count_grid = np.zeros((grid_res, grid_res), dtype=int)
        crop_count_grid = np.zeros((grid_res, grid_res), dtype=int)

        for pt, val in zip(norm_coords, values):
            gx = int(pt[0] / grid_size * grid_res)
            gy = int(pt[1] / grid_size * grid_res)
            if 0 <= gx < grid_res and 0 <= gy < grid_res:
                count_grid[gy, gx] += 1
                if val == crop_code:
                    crop_count_grid[gy, gx] += 1

        with np.errstate(divide='ignore', invalid='ignore'):
            frac = crop_count_grid / count_grid
            label_grid = (frac >= 0.5) & (count_grid > 0)

        # Coalition selection logic
        farm_indices = np.argwhere(label_grid)
        num_selected = int(len(farm_indices) * coalition_percent)
        if num_selected == 0:
            raise ValueError("No fields selected for coalition. Adjust COALITION_PERCENT or verify CDL data.")
        selected_indices = farm_indices[np.random.choice(len(farm_indices), num_selected, replace=False)]

        new_label_grid = np.zeros_like(label_grid)
        for y, x in selected_indices:
            new_label_grid[y, x] = True

        return new_label_grid, crop_count_grid


def parse_args():
    parser = argparse.ArgumentParser(description="Run CDL DroneSim with overrides")
    parser.add_argument('--crop-code', type=int)
    parser.add_argument('--crop-name', type=str)
    parser.add_argument('--grid-size', type=int)
    parser.add_argument('--grid-res', type=int)
    parser.add_argument('--num-drones', type=int)
    parser.add_argument('--num-customers', type=int)
    parser.add_argument('--coalition-percent', type=float)
    parser.add_argument('--drone-speed', type=float)
    return parser.parse_args()

def override_configs_from_args(args):
    cfg = configs.config

    if args.crop_code is not None: cfg['CROP_CODE'] = args.crop_code
    if args.crop_name is not None: cfg['CROP_NAME'] = args.crop_name
    if args.grid_size is not None: cfg['GRID_SIZE'] = args.grid_size
    if args.grid_res is not None: cfg['GRID_RES'] = args.grid_res
    if args.num_drones is not None: cfg['NUM_DRONES'] = args.num_drones
    if args.num_customers is not None: cfg['NUM_CUSTOMERS'] = args.num_customers
    if args.coalition_percent is not None: cfg['COALITION_PERCENT'] = args.coalition_percent
    if args.drone_speed is not None: cfg['DRONE_SPEED'] = args.drone_speed

    configs.CROP_CODE = cfg['CROP_CODE']
    configs.CROP_NAME = cfg['CROP_NAME']
    configs.GRID_SIZE = cfg['GRID_SIZE']
    configs.GRID_RES = cfg['GRID_RES']
    configs.NUM_DRONES = cfg['NUM_DRONES']
    configs.NUM_CUSTOMERS = cfg['NUM_CUSTOMERS']
    configs.COALITION_PERCENT = cfg['COALITION_PERCENT']
    configs.DRONE_SPEED = cfg['DRONE_SPEED']
    configs.BASE_LOC = np.array([cfg['GRID_SIZE'] / 2, cfg['GRID_SIZE'] / 2])


def main():

    args = parse_args()
    override_configs_from_args(args)

    from configs import (
        CROP_CODE, CROP_NAME, GRID_SIZE, GRID_RES,
        NUM_DRONES, NUM_CUSTOMERS, BASE_LOC, COALITION_PERCENT
    )

    print("Fetching and processing CDL data...")
    memfile = fetch_cdl()

    label_grid, field_counts = generate_crop_grid(
        memfile,
        CROP_CODE,
        GRID_SIZE,
        GRID_RES,
        COALITION_PERCENT
    )
    start_time = time.time()
    start_dt = datetime.datetime.now()
    print(f"Grid ready at {start_dt.strftime('%Y-%m-%d %H:%M:%S')}")


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
    sim = DroneSim(NUM_DRONES, NUM_CUSTOMERS, field_coords, label_grid)
    gui = DroneSimGUI(label_grid, field_counts, sim)
    gui.show()
    sim.timer.start(50)

    def sim_finished():
        sim.timer.stop()
        gui.close()
        app.quit()

        # Logging after sim closes
        end_time = time.time()
        duration = end_time - start_time
        end_dt = datetime.datetime.now()

        log_lines = [
            f"=== Simulation Completed ===",
            f"Start Time: {start_dt.strftime('%Y-%m-%d %H:%M:%S')}",
            f"End Time:   {end_dt.strftime('%Y-%m-%d %H:%M:%S')}",
            f"Duration:   {duration:.2f} seconds",
            "",
            "Parameters:",
            f"  Crop:              {configs.CROP_NAME} (code {configs.CROP_CODE})",
            f"  Grid Size:         {configs.GRID_SIZE}",
            f"  Grid Resolution:   {configs.GRID_RES} x {configs.GRID_RES}",
            f"  Number of Drones:  {configs.NUM_DRONES}",
            f"  Number of Customers: {configs.NUM_CUSTOMERS}",
            f"  Coalition %:       {configs.COALITION_PERCENT:.2%}",
            f"  Drone Speed:       {configs.DRONE_SPEED}",
            "",
            "Delivery Summary:",
            f"  Total Deliveries:  {len(sim.delivered)}",
            f"  Successful:        {sum(sim.delivered)}",
            f"  Failed:            {len(sim.delivered) - sum(sim.delivered)}",
            "",
            "-" * 40,
            ""
        ]

        with open("log.txt", "a") as f:
            f.write("\n".join(log_lines))


    sim.on_finish = sim_finished

    exit_code = app.exec_()

    with open("log.txt", "a") as file:
        file.write("Simulation completed and GUI closed.\n")

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
