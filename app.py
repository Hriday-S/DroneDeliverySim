import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import streamlit as st
from configs import USE_RANDOM_GRID, GRID_SIZE, GRID_RES, NUM_DRONES, NUM_CUSTOMERS, DRONE_SPEED, BASE_LOC, COALITION_PERCENT

st.set_page_config(page_title="DroneSim")

# Constants
eps = 1e-6
cell = GRID_SIZE / GRID_RES
_xy = lambda i, j: ((j + 0.5) * cell, (i + 0.5) * cell)
_dist = lambda a, b: ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

# CDL crop code selection dropdown
CROP_CODE_OPTIONS = {
    "Corn (1)": 1,
    "Cotton (2)": 2,
    "Rice (3)": 3,
    "Sorghum (4)": 4,
    "Soybeans (5)": 5,
    "Sunflower (6)": 6,
    "Peanuts (10)": 10,
    "Tobacco (11)": 11,
    "Sweet Corn (12)": 12,
    "Popcorn (13)": 13,
    "Mint (14)": 14,
    "Barley (21)": 21,
    "Durum Wheat (22)": 22,
    "Spring Wheat (23)": 23,
    "Winter Wheat (24)": 24,
    "Other Small Grains (25)": 25,
}
crop_label = st.selectbox("Select crop type:", list(CROP_CODE_OPTIONS.keys()))
CROP_CODE = CROP_CODE_OPTIONS[crop_label]
# Automatically reset simulation state if crop selection changed
prev_crop = st.session_state.get("prev_crop")
if prev_crop != crop_label:
    for key in ["label_grid", "customers", "grid_codes", "sim", "running", "prev_url"]:
        st.session_state.pop(key, None)
    st.session_state.prev_crop = crop_label
    st.rerun()

# Official colors for CDL codes
authoritative_colors = {
    1: "#FFFF64", 2: "#F0F0F0", 3: "#A6CEE3", 4: "#FB9A99",
    5: "#33A02C", 6: "#FFD92F", 10: "#FF7F00", 11: "#B15928",
    12: "#E31A1C", 13: "#FDBF6F", 14: "#CAB2D6", 21: "#6A3D9A",
    22: "#FEB24C", 23: "#FED976", 24: "#FFEDA0", 25: "#B2DF8A",
}

# Helper: ensure unique positions
def unique_positions(positions):
    seen = set()
    unique = []
    for pos in positions:
        if pos not in seen:
            seen.add(pos)
            unique.append(pos)
    return unique

# Generate random grid
def _rand_grid():
    total = GRID_RES * GRID_RES
    num_farms = int(total * COALITION_PERCENT)
    rng = np.random.default_rng()
    perm = rng.permutation(total)
    farm_idx = perm[:num_farms]
    lab = np.zeros((GRID_RES, GRID_RES), dtype=bool)
    grid_codes = np.full((GRID_RES, GRID_RES), -1, dtype=int)
    for k in farm_idx:
        i, j = divmod(k, GRID_RES)
        lab[i, j] = True
        grid_codes[i, j] = CROP_CODE
    open_cells = np.argwhere(~lab)
    if len(open_cells) <= NUM_CUSTOMERS:
        sel = open_cells
    else:
        sel = open_cells[rng.choice(len(open_cells), NUM_CUSTOMERS, replace=False)]
    customers = [tuple(c) for c in sel]
    return lab, customers, grid_codes

# Fetch real grid from GeoTIFF
def _fetch_grid(url):
    import requests, rasterio, tempfile
    from io import BytesIO
    from zipfile import ZipFile
    from collections import Counter

    z = ZipFile(BytesIO(requests.get(url, timeout=30).content))
    tif_name = next(n for n in z.namelist() if n.endswith('.tif'))
    with tempfile.NamedTemporaryFile(delete=False, suffix='.tif') as tmp:
        tmp.write(z.read(tif_name))
        path = tmp.name
    with rasterio.open(path) as src:
        data = src.read(1)
        transform = src.transform

    rows, cols = data.shape
    xs = np.linspace(0.5, cols - 0.5, cols)
    ys = np.linspace(0.5, rows - 0.5, rows)
    xx, yy = np.meshgrid(xs, ys)
    coords = np.vstack([xx.ravel(), yy.ravel()]).T
    world = np.array([tuple(transform * tuple(p)) for p in coords])
    mn = world.min(axis=0)
    span = np.maximum(world.max(axis=0) - mn, eps)
    norm = (world - mn) / span * GRID_SIZE
    gx = np.clip((norm[:, 0] / GRID_SIZE * GRID_RES).astype(int), 0, GRID_RES - 1)
    gy = np.clip((norm[:, 1] / GRID_SIZE * GRID_RES).astype(int), 0, GRID_RES - 1)

    cell_vals = [[[] for _ in range(GRID_RES)] for _ in range(GRID_RES)]
    for x, y, v in zip(gx, gy, data.ravel()):
        cell_vals[y][x].append(v)

    lab = np.zeros((GRID_RES, GRID_RES), dtype=bool)
    grid_codes = np.full((GRID_RES, GRID_RES), -1, dtype=int)
    for i in range(GRID_RES):
        for j in range(GRID_RES):
            if cell_vals[i][j]:
                mc = Counter(cell_vals[i][j]).most_common(1)[0][0]
                grid_codes[i, j] = mc
                lab[i, j] = (mc == CROP_CODE)

    open_cells = np.argwhere(~lab)
    rng = np.random.default_rng()
    if len(open_cells) <= NUM_CUSTOMERS:
        sel = open_cells
    else:
        sel = open_cells[rng.choice(len(open_cells), NUM_CUSTOMERS, replace=False)]
    customers = [tuple(c) for c in sel]
    return lab, customers, grid_codes

# DroneSim class
class DroneSim:
    def __init__(self, n, customers, farms):
        self.farms = farms
        self.cust = customers
        self.base = tuple(BASE_LOC)
        self.dpos = [self.base] * n
        self.done = [False] * len(customers)
        self.pick = [self._farm(c) for c in customers]
        self.routes = [[self.pick[i], customers[i]] for i in range(len(customers))]
        self.assign = self._alloc()
        self.idx = [0] * n
        self.fin = False

    def _farm(self, customer):
        f = np.argwhere(self.farms)
        return tuple(f[np.argmin(np.hypot(f[:,0] - customer[0], f[:,1] - customer[1]))])

    def _alloc(self):
        assignments = [[] for _ in self.dpos]
        left = set(range(len(self.cust)))
        loc = [self.base] * len(self.dpos)
        while left:
            for d in range(len(self.dpos)):
                if not left:
                    break
                best = min(left, key=lambda i: _dist(loc[d], self.pick[i]) + _dist(self.pick[i], self.cust[i]))
                assignments[d].append(best)
                loc[d] = self.cust[best]
                left.remove(best)
        return assignments

    def step(self, spd=DRONE_SPEED):
        for d, pos in enumerate(self.dpos):
            rem = spd
            while rem > eps and self.assign[d]:
                ci = self.assign[d][0]
                tgt = self.routes[ci][self.idx[d]]
                vec = (tgt[0] - pos[0], tgt[1] - pos[1])
                dist = (vec[0]**2 + vec[1]**2)**0.5
                if dist < eps:
                    self.idx[d] += 1
                    if self.idx[d] == 2:
                        self.done[ci] = True
                        self.assign[d].pop(0)
                        self.idx[d] = 0
                    continue
                stepv = min(rem, dist)
                frac = stepv / dist
                pos = (pos[0] + vec[0] * frac, pos[1] + vec[1] * frac)
                rem -= stepv
            self.dpos[d] = pos
        self.fin = all(self.done)
        return self.fin

# UI: load or fetch grid
url = st.text_input("GeoTIFF ZIP URL (optional)")
if "label_grid" not in st.session_state:
    if USE_RANDOM_GRID or not url:
        lab, customers, grid_codes = _rand_grid()
    else:
        lab, customers, grid_codes = _fetch_grid(url)
    st.session_state.label_grid = lab
    st.session_state.customers = customers
    st.session_state.grid_codes = grid_codes
elif url and st.session_state.get("prev_url") != url:
    lab, customers, grid_codes = _fetch_grid(url)
    st.session_state.label_grid = lab
    st.session_state.customers = customers
    st.session_state.grid_codes = grid_codes
    st.session_state.prev_url = url

# Load grid data from session
grid_codes = st.session_state.get("grid_codes")
# Banner if no farms of selected crop
if not np.any(grid_codes == CROP_CODE):
    st.warning(f"No {crop_label} farms found. The Start button is disabled.")

# Controls
# Banner if no farms of selected crop
if not np.any(grid_codes == CROP_CODE):
    st.warning(f"No {crop_label} farms found. The Start button is disabled.")

# Controls
c1, c2, c3 = st.columns(3)
if c1.button("Start"):
    st.session_state.running = True
if c2.button("Stop"):
    st.session_state.running = False
if c3.button("Reset"):
    for key in ["label_grid", "customers", "grid_codes", "sim", "running", "prev_url"]:
        st.session_state.pop(key, None)
    st.rerun()

# Instantiate sim
label_grid = st.session_state.label_grid
customers = st.session_state.customers
grid_codes = st.session_state.grid_codes
if "sim" not in st.session_state:
    st.session_state.sim = DroneSim(NUM_DRONES, customers, label_grid)
sim = st.session_state.sim

# Plot grid
fig, ax = plt.subplots(figsize=(6,6))
ax.set_xlim(0, GRID_SIZE)
ax.set_ylim(0, GRID_SIZE)
ax.set_aspect('equal')
ax.invert_yaxis()
for i in range(GRID_RES):
    for j in range(GRID_RES):
        code = grid_codes[i, j]
        color = authoritative_colors.get(code, 'white')
        ax.add_patch(patches.Rectangle(
            (j*cell, i*cell), cell, cell,
            facecolor=color, edgecolor='gray', linewidth=0.5
        ))
# Draw base
ax.scatter(*_xy(*BASE_LOC), s=80, c='orange', edgecolors='black')
# Draw pick positions only on selected crop
for p in sim.pick:
    if grid_codes[p[0], p[1]] == CROP_CODE:
        ax.plot(*_xy(*p), 'x', color='red', markersize=6)
# Draw unique customers
for p, ok in zip(unique_positions(customers), sim.done):
    ax.plot(*_xy(*p), 'o', color=('blue' if not ok else 'lightblue'), markersize=6)
# Draw drones
for dpos in sim.dpos:
    ax.plot(*_xy(*dpos), 'o', color='black', markersize=6)

st.pyplot(fig)

# Step sim
if st.session_state.get("running") and not sim.fin:
    sim.step()
    time.sleep(0.05)
    st.rerun()
if sim.fin and st.session_state.get("running"):
    st.session_state.running = False
    st.success("All deliveries complete")
