import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
from configs import USE_RANDOM_GRID, CROP_CODE, GRID_SIZE, GRID_RES, NUM_DRONES, NUM_CUSTOMERS, COALITION_PERCENT, DRONE_SPEED, BASE_LOC

# Session state init
if "sim" not in st.session_state:
    st.session_state.sim = None
if "running" not in st.session_state:
    st.session_state.running = False

# Farm + customer generator
def generate_farm_grid_and_customers(grid_res, farm_percent, num_customers):
    total_cells = grid_res * grid_res
    max_farm_cells = total_cells - num_customers
    if max_farm_cells <= 0:
        raise ValueError("Too many customers or grid too small to allocate non-farm cells.")
    farm_percent = min(farm_percent, max_farm_cells / total_cells)

    label_grid = np.zeros((grid_res, grid_res), dtype=bool)
    num_farm = int(total_cells * farm_percent)
    indices = np.random.choice(total_cells, num_farm, replace=False)
    for idx in indices:
        i, j = divmod(idx, grid_res)
        label_grid[i, j] = True

    allowed = np.argwhere(~label_grid)
    chosen = allowed[np.random.choice(len(allowed), num_customers, replace=False)]
    cell_size = GRID_SIZE / GRID_RES
    customer_pos = np.array([
        [(j + 0.5) * cell_size, (i + 0.5) * cell_size] for i, j in chosen
    ])
    return label_grid, customer_pos

# Sim class
class DroneSim:
    def __init__(self, num_drones, customer_pos, label_grid):
        self.label_grid = label_grid
        self.customer_pos = customer_pos
        self.cell_size = GRID_SIZE / GRID_RES
        self.drone_pos = np.array([BASE_LOC.copy() for _ in range(num_drones)])
        self.pickup_points = np.array([
            self.closest_farm(c) for c in self.customer_pos
        ])
        self.routes = [[self.pickup_points[i], self.customer_pos[i]] for i in range(len(self.customer_pos))]
        self.assignments = self.assign_customers()
        self.drone_route_idx = [0] * num_drones
        self.delivered = [False] * len(self.customer_pos)
        self._finished = False

    def closest_farm(self, customer_pos):
        farm_cells = np.argwhere(self.label_grid)
        farm_coords = np.array([
            [(j + 0.5) * self.cell_size, (i + 0.5) * self.cell_size]
            for i, j in farm_cells
        ])
        dists = np.linalg.norm(farm_coords - customer_pos, axis=1)
        return farm_coords[np.argmin(dists)]

    def assign_customers(self):
        unassigned = set(range(len(self.customer_pos)))
        assign = [[] for _ in self.drone_pos]
        drone_locs = [BASE_LOC.copy() for _ in self.drone_pos]
        while unassigned:
            for d in range(len(assign)):
                if not unassigned:
                    break
                cust_idx = min(unassigned, key=lambda c:
                    np.linalg.norm(drone_locs[d] - self.pickup_points[c]) +
                    np.linalg.norm(self.pickup_points[c] - self.customer_pos[c])
                )
                assign[d].append(cust_idx)
                drone_locs[d] = self.customer_pos[cust_idx].copy()
                unassigned.remove(cust_idx)
        return assign

    def step(self, speed):
        for i in range(len(self.drone_pos)):
            if self.assignments[i]:
                cust_idx = self.assignments[i][0]
                route = self.routes[cust_idx]
                target_idx = self.drone_route_idx[i]
                target = route[target_idx]
                direction = target - self.drone_pos[i]
                dist = np.linalg.norm(direction)
                if dist < speed:
                    self.drone_pos[i] = target.copy()
                    self.drone_route_idx[i] += 1
                    if self.drone_route_idx[i] == len(route):
                        self.assignments[i].pop(0)
                        self.drone_route_idx[i] = 0
                        self.delivered[cust_idx] = True
                else:
                    self.drone_pos[i] += direction / dist * speed
        self._finished = all(self.delivered)
        return self._finished

# UI setup
st.set_page_config(page_title="DroneSim Visualizer", layout="wide")
st.title("Drone Delivery Visualizer")

# Grid + Sim init
label_grid, customer_pos = generate_farm_grid_and_customers(GRID_RES, COALITION_PERCENT, NUM_CUSTOMERS)
if st.session_state.sim is None:
    st.session_state.sim = DroneSim(NUM_DRONES, customer_pos, label_grid)
sim = st.session_state.sim

# Controls
col1, col2 = st.columns(2)
with col1:
    if st.button("Start Simulation") and not st.session_state.running:
        st.session_state.running = True
with col2:
    if st.button("Stop Simulation"):
        st.session_state.running = False

# Plot
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(0, GRID_SIZE)
ax.set_ylim(0, GRID_SIZE)
ax.set_aspect('equal')
for i in range(GRID_RES):
    for j in range(GRID_RES):
        color = 'green' if label_grid[i, j] else 'white'
        ax.add_patch(patches.Rectangle(
            (j * GRID_SIZE / GRID_RES, i * GRID_SIZE / GRID_RES),
            GRID_SIZE / GRID_RES, GRID_SIZE / GRID_RES,
            facecolor=color, edgecolor='gray', linewidth=0.5
        ))

plot_area = st.empty()
while st.session_state.running and not sim._finished:
    sim.step(DRONE_SPEED)

    ax.clear()
    ax.set_xlim(0, GRID_SIZE)
    ax.set_ylim(0, GRID_SIZE)
    ax.set_aspect('equal')

    for i in range(GRID_RES):
        for j in range(GRID_RES):
            color = 'green' if label_grid[i, j] else 'white'
            ax.add_patch(patches.Rectangle(
                (j * GRID_SIZE / GRID_RES, i * GRID_SIZE / GRID_RES),
                GRID_SIZE / GRID_RES, GRID_SIZE / GRID_RES,
                facecolor=color, edgecolor='gray', linewidth=0.5
            ))

    ax.scatter(BASE_LOC[0], BASE_LOC[1], s=80, c='orange', edgecolors='black', label='Base')

    for pos, delivered in zip(sim.customer_pos, sim.delivered):
        color = 'lightblue' if delivered else 'blue'
        ax.plot(pos[0], pos[1], 'o', color=color, markersize=6)

    for i, (x, y) in enumerate(sim.drone_pos):
        # Yellow if heading to pickup, black if heading to deliver
        if sim.assignments[i]:
            cust_idx = sim.assignments[i][0]
            route_idx = sim.drone_route_idx[i]
            if route_idx == 0:
                drone_color = 'yellow'
            else:
                drone_color = 'black'
        else:
            drone_color = 'gray'
        ax.plot(x, y, 'o', color=drone_color, markersize=6)

    ax.set_title("Live Drone Routes")
    plot_area.pyplot(fig)

    if sim._finished:
        st.success("All deliveries completed!")
        st.session_state.running = False

    time.sleep(0.01)
