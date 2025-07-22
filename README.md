# DroneSim Visualizer

A lightweight Streamlit-based simulation for visualizing drone delivery to customers in a gridded farm environment. This tool models the logistics of multi-drone systems operating over agricultural areas, with real-time route planning, pickup point selection, and delivery tracking.

## 🔧 Features

- Simulates delivery from farm grid cells to customers
- Real-time drone routing visualization
- Start/Stop controls with live animation
- Dynamic grid generation (random or CDL-based)
- Adjustable parameters via `configs.py`

## 📁 Structure

DroneSim/
├── dronesim_app.py # Main Streamlit UI and simulation logic
├── configs.py # Centralized simulation parameters
├── requirements.txt # Dependencies
├── README.md # You're reading it

makefile
Copy
Edit

## ⚙️ Configuration

Modify `configs.py` to set simulation parameters:

```python
USE_RANDOM_GRID = True
CROP_CODE = 5               # CDL crop code for soybeans (ignored if random)
GRID_SIZE = 50              # Width/height in meters
GRID_RES = 10               # Grid resolution (GRID_RES x GRID_RES cells)
NUM_DRONES = 3
NUM_CUSTOMERS = 5
COALITION_PERCENT = 0.4     # % of grid marked as farm
DRONE_SPEED = 3             # Movement speed per timestep
BASE_LOC = np.array([25, 25])  # Central base location
🚀 Usage
Install dependencies:

bash
Copy
Edit
pip install -r requirements.txt
Run the app:

bash
Copy
Edit
streamlit run dronesim_app.py
Use the "Start Simulation" button to begin drone movement. Stop or reset as needed.

📊 Visualization
Green cells: Farm plots eligible for pickup

Blue dots: Customer locations

Orange dot: Base station

Black dots: Drones en route

❗ Notes
Customers are never placed on farm cells.

Farm cells are randomly selected unless USE_RANDOM_GRID=False (for CDL integration).

Simulation halts automatically when all deliveries complete.

🧠 TODO
Add CDL map support with rasterio

Path optimization (e.g., TSP approximation)

Delivery heatmap export / analytics

UI slider for speed, grid size, etc.

