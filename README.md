# DroneSim Visualizer

A lightweight Streamlit-based simulation for visualizing drone delivery to customers in a gridded farm environment. This tool models the logistics of multi-drone systems operating over agricultural areas, with real-time route planning, pickup point selection, and delivery tracking.

## 🔧 Features

* Simulates delivery from farm grid cells to customers
* Real-time drone routing visualization
* Start/Stop/Reset controls with live animation
* Dynamic grid generation (random or CDL-based)
* Adjustable parameters via the UI or `configs.py`

## 📁 Structure

```
DroneSim/
├── dronesim_app.py    # Main Streamlit UI and simulation logic
├── configs.py        # Centralized simulation parameters (defaults)
├── requirements.txt   # Dependencies
├── README.md         # Project documentation
```

## ⚙️ Configuration

Defaults are defined in `configs.py`, but many parameters can be adjusted at runtime via the app controls:

```python
# configs.py defaults:
USE_RANDOM_GRID = True
CROP_CODE = 5               # Default CDL crop code for soybeans
GRID_SIZE = 50              # Width/height in meters
GRID_RES = 10               # Default grid resolution (cells per side)
NUM_DRONES = 3
NUM_CUSTOMERS = 5           # Default number of customers
COALITION_PERCENT = 0.4     # % of grid marked as farm
DRONE_SPEED = 3             # Movement speed per timestep
BASE_LOC = np.array([25, 25])  # Central base location
```

### Runtime Controls

* **Grid resolution**: Adjust the number of cells per side (defaults to `GRID_RES` from `configs.py`). Changing this resets the simulation.
* **Number of customers**: Set how many delivery points to generate (defaults to `NUM_CUSTOMERS` from `configs.py`). Changing this resets the simulation.
* **Crop type**: Select the CDL crop code for farm cells. Changing this resets the grid and delivery points.
* **GeoTIFF URL**: Optionally load a real CDL GeoTIFF to generate the farm mask.

## 🚀 Usage

Install dependencies:

```bash
pip install -r requirements.txt
```

Run the app:

```bash
streamlit run dronesim_app.py
```

Use the UI sliders and inputs to configure resolution, customer count, and crop type. Then click **Start** to begin the simulation.

## 📊 Visualization Legend

* **Colored cells**: Crop type (per CDL color map)
* **Red X**: Pickup points on matching crop cells
* **Blue dots**: Customer locations
* **Orange dot**: Base station
* **Black dots**: Drones en route

## ❗ Notes

* Customers are never placed on crop (farm) cells.
* Simulation automatically resets and updates when key parameters change.
* Reset the app at any time using the **Reset** button.

## 🧠 TODO

* Path optimization (e.g., TSP approximation)
* Delivery heatmap export / analytics
* UI controls for drone speed, grid size, etc.
