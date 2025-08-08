# DroneSim — From CDL Raster to Drone Delivery Simulation

A Streamlit app that processes USDA Cropland Data Layer (CDL) raster data, finds the top-N most suitable farm cells for a chosen crop, places customers on non-farm cells, and simulates multi-drone delivery routes from a central base.

---

## How It Works

### 1. Raster Processing
- Loads a CDL GeoTIFF for Illinois.
- Clips to state boundaries using the shapefile (CRS is matched before masking).
- Marks fill (nodata) cells with a fixed value so masking is unambiguous.

### 2. Grid Aggregation
- The clipped raster is binned into a `GRID_RES x GRID_RES` grid.
- For each grid cell, counts how many valid pixels match the chosen crop code.
- Calculates fraction = (matching pixels) / (total valid pixels in the cell).

### 3. Selecting Farm Cells
- Only considers cells where fraction > 0.
- Picks the top-N by fraction, limited so there’s still space for customer placement.
- If fewer than N candidates exist, the number is reduced automatically.

### 4. Placing Customers
- Randomly samples non-farm cells for customers.
- Uses a deterministic RNG seed based on crop code and grid resolution, so placement is stable between runs.

### 5. Base Location
- Starts at the center of the crop’s bounding box.
- Snaps to the nearest valid in-state cell.

### 6. Drone Simulation
- Each job is “farm → customer.”
- Assigns jobs to drones using a greedy nearest-available strategy.
- Drones move at a fixed speed until all deliveries are complete.

### 7. Visualization
- Background: light gray in-state, white outside.
- Farms: crop color from CDL’s DBF or from `crop_codes.csv`.
- Base: orange circle.
- Customers: blue (light blue when served).
- Pickups: red “X.”
- Drones: black dots.
- Optional gridlines overlay.

---

## Install

```bash
python -m venv .venv
source .venv/bin/activate  # On macOS/Linux
# .venv\Scripts\activate   # On Windows
pip install -U pip
pip install -r requirements.txt
```
## Run
```
streamlit run app.py
```

Configurable Parameters (in app.py)
```
GRID_RES — grid resolution for both raster aggregation and simulation.

FARM_COUNT — number of farm cells to select.

NUM_CUSTOMERS — number of customers to place.

NUM_DRONES — number of drones in the sim.

DRONE_SPEED — movement speed of drones.

DEFAULT_TARGET_CODE — initial crop code.
```
--
## Performance
Processing cost grows with raster size and GRID_RES².

100–300 grid resolution is usually fine; 500+ will be slower.

Uses numpy.add.at for vectorized binning and np.argpartition for efficient top-N selection.
--
## Common Errors
Input shapes do not overlap raster — The shapefile and raster don’t intersect; check CRS and file paths.

No classes present — Clip produced only nodata values.

No farm cells found — Crop too sparse; try a different crop or lower GRID_RES.

No non-farm cells for customers — FARM_COUNT too high; reduce it.

## Data (required, not in repo)

The CDL GeoTIFF is large, so it isn’t tracked here. You must supply it locally.

### Option A — Manual (recommended)
1. Create a `tif/` folder in the project root.
2. Download the 2024 CDL GeoTIFF from USDA NASS, for Illinois https://nassgeodata.gmu.edu/nass_data_cache/byfips/CDL_2024_17.zip

## Data Source
USDA NASS Cropland Data Layer (CDL)
National Agricultural Statistics Service, U.S. Department of Agriculture
https://www.nass.usda.gov/Research_and_Science/Cropland/SARS1a.php
