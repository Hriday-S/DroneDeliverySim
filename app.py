import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import streamlit as st
import pandas as pd
from configs import USE_RANDOM_GRID, GRID_SIZE, GRID_RES as DEFAULT_GRID_RES, NUM_DRONES, NUM_CUSTOMERS, DRONE_SPEED, BASE_LOC, COALITION_PERCENT

# Load crop codes and colors from CSV
# Expected CSV columns: code (int), name (string), color (hex string)
crop_df = pd.read_csv("crop_codes.csv")
CROP_CODE_OPTIONS = {f"{row['name']} ({row['code']})": int(row['code']) for _, row in crop_df.iterrows()}
authoritative_colors = {int(row['code']): row['color'] for _, row in crop_df.iterrows()}

st.set_page_config(page_title="DroneSim")
plot_placeholder = st.empty()

# --- User-adjustable resolution input ---
res = st.number_input(
    "Grid resolution (cells per side)",
    min_value=5,
    max_value=500,
    value=DEFAULT_GRID_RES,
    step=1
)
# Reset state if resolution changes
d_prev = st.session_state.get("prev_res")
if d_prev != res:
    for k in ["label_grid", "customers", "grid_codes", "sim", "running", "prev_url"]:
        st.session_state.pop(k, None)
    st.session_state.prev_res = res
    st.rerun()

# Use local resolution
GRID_RES = res
cell = GRID_SIZE / GRID_RES
_xy = lambda i, j: ((j + 0.5) * cell, (i + 0.5) * cell)
_dist = lambda a, b: ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

eps = 1e-6

# --- User-adjustable customer count input ---
num_customers = st.number_input(
    "Number of customers",
    min_value=1,
    max_value=GRID_RES * GRID_RES,
    value=NUM_CUSTOMERS,
    step=1
)
# Reset state if customer count changes
pc_prev = st.session_state.get("prev_cust")
if pc_prev != num_customers:
    for k in ["label_grid", "customers", "grid_codes", "sim", "running", "prev_url"]:
        st.session_state.pop(k, None)
    st.session_state.prev_cust = num_customers
    st.rerun()
CUSTOMERS = num_customers

# --- Crop type selection ---
crop_label = st.selectbox("Select crop type:", list(CROP_CODE_OPTIONS.keys()))
CROP_CODE = CROP_CODE_OPTIONS[crop_label]
# Reset state if crop changes
c_prev = st.session_state.get("prev_crop")
if c_prev != crop_label:
    for k in ["label_grid", "customers", "grid_codes", "sim", "running", "prev_url"]:
        st.session_state.pop(k, None)
    st.session_state.prev_crop = crop_label
    st.rerun()

# Helper for unique customers
def unique_positions(positions):
    seen = set(); unique = []
    for p in positions:
        if p not in seen:
            seen.add(p); unique.append(p)
    return unique

# Random grid generator
def _rand_grid():
    total = GRID_RES * GRID_RES
    num_farms = int(total * COALITION_PERCENT)
    rng = np.random.default_rng()
    perm = rng.permutation(total)
    farms_idx = perm[:num_farms]

    lab = np.zeros((GRID_RES, GRID_RES), dtype=bool)
    grid_codes = np.full((GRID_RES, GRID_RES), -1, dtype=int)
    for k in farms_idx:
        i, j = divmod(k, GRID_RES)
        lab[i, j] = True
        grid_codes[i, j] = CROP_CODE

    open_cells = np.argwhere(~lab)
    if len(open_cells) <= CUSTOMERS:
        sel = open_cells
    else:
        sel = open_cells[rng.choice(len(open_cells), CUSTOMERS, replace=False)]
    customers = [tuple(c) for c in sel]
    return lab, customers, grid_codes

# Fetch real grid logic
def _fetch_grid(url):
    import requests, rasterio, tempfile
    from io import BytesIO
    from zipfile import ZipFile
    from collections import Counter

    z = ZipFile(BytesIO(requests.get(url, timeout=30).content))
    tif = next(n for n in z.namelist() if n.endswith('.tif'))
    with tempfile.NamedTemporaryFile(delete=False, suffix='.tif') as tmp:
        tmp.write(z.read(tif)); path = tmp.name

    with rasterio.open(path) as src:
        data = src.read(1); tr = src.transform
    rows, cols = data.shape
    xs = np.linspace(0.5, cols - 0.5, cols)
    ys = np.linspace(0.5, rows - 0.5, rows)
    xx, yy = np.meshgrid(xs, ys)
    pts = np.vstack([xx.ravel(), yy.ravel()]).T
    world = np.array([tuple(tr * tuple(p)) for p in pts])
    mn = world.min(axis=0); span = np.maximum(world.max(axis=0) - mn, eps)
    norm = (world - mn) / span * GRID_SIZE
    gx = np.clip((norm[:,0] / GRID_SIZE * GRID_RES).astype(int), 0, GRID_RES - 1)
    gy = np.clip((norm[:,1] / GRID_SIZE * GRID_RES).astype(int), 0, GRID_RES - 1)

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
    if len(open_cells) <= CUSTOMERS:
        sel = open_cells
    else:
        sel = open_cells[rng.choice(len(open_cells), CUSTOMERS, replace=False)]
    customers = [tuple(c) for c in sel]
    return lab, customers, grid_codes

# DroneSim class
class DroneSim:
    def __init__(self, n, customers, farms):
        self.farms = farms; self.cust = customers; self.base = tuple(BASE_LOC)
        self.dpos = [self.base] * n; self.done = [False] * len(customers)
        self.pick = [self._farm(c) for c in customers]
        self.routes = [[self.pick[i], customers[i]] for i in range(len(customers))]
        self.assign = self._alloc(); self.idx = [0] * n; self.fin = False
    def _farm(self, c):
        f = np.argwhere(self.farms)
        return tuple(f[np.argmin(np.hypot(f[:,0]-c[0], f[:,1]-c[1]))])
    def _alloc(self):
        A = [[] for _ in self.dpos]; left = set(range(len(self.cust))); loc = [self.base] * len(self.dpos)
        while left:
            for d in range(len(self.dpos)):
                if not left: break
                best = min(left, key=lambda i: _dist(loc[d], self.pick[i]) + _dist(self.pick[i], self.cust[i]))
                A[d].append(best); loc[d] = self.cust[best]; left.remove(best)
        return A
    def step(self, spd=DRONE_SPEED):
        for d, pos in enumerate(self.dpos):
            rem = spd
            while rem > eps and self.assign[d]:
                ci = self.assign[d][0]; tgt = self.routes[ci][self.idx[d]]
                vec = (tgt[0] - pos[0], tgt[1] - pos[1]); dist = (vec[0]**2 + vec[1]**2)**0.5
                if dist < eps:
                    self.idx[d] += 1
                    if self.idx[d] == 2: self.done[ci] = True; self.assign[d].pop(0); self.idx[d] = 0
                    continue
                stepv = min(rem, dist); frac = stepv / dist
                pos = (pos[0] + vec[0]*frac, pos[1] + vec[1]*frac); rem -= stepv
            self.dpos[d] = pos
        self.fin = all(self.done); return self.fin

# Load or fetch grid
url = st.text_input("GeoTIFF ZIP URL (optional)")
if "label_grid" not in st.session_state:
    if USE_RANDOM_GRID or not url: lab, customers, grid_codes = _rand_grid()
    else: lab, customers, grid_codes = _fetch_grid(url)
    st.session_state.label_grid = lab; st.session_state.customers = customers; st.session_state.grid_codes = grid_codes
elif url and st.session_state.get("prev_url") != url:
    lab, customers, grid_codes = _fetch_grid(url); st.session_state.label_grid = lab; st.session_state.customers = customers; st.session_state.grid_codes = grid_codes; st.session_state.prev_url = url

# Warning if no farms
if not np.any(st.session_state.grid_codes == CROP_CODE):
    st.warning(f"No {crop_label} farms found. Start disabled.")

# Controls
c1, c2, c3 = st.columns(3)
if c1.button("Start"): st.session_state.running = True
if c2.button("Stop"): st.session_state.running = False
if c3.button("Reset"): [st.session_state.pop(k, None) for k in ["label_grid", "customers", "grid_codes", "sim", "running", "prev_url"]]; st.rerun()

# Instantiate sim
label_grid = st.session_state.label_grid; customers = st.session_state.customers; grid_codes = st.session_state.grid_codes
if "sim" not in st.session_state: st.session_state.sim = DroneSim(NUM_DRONES, customers, label_grid)
sim = st.session_state.sim

# Plot grid
fig, ax = plt.subplots(figsize=(6,6)); ax.set_xlim(0, GRID_SIZE); ax.set_ylim(0, GRID_SIZE); ax.set_aspect('equal'); ax.invert_yaxis()
for i in range(GRID_RES):
    for j in range(GRID_RES):
        code = grid_codes[i,j]; color = authoritative_colors.get(code, 'white')
        ax.add_patch(patches.Rectangle((j*cell, i*cell), cell, cell, facecolor=color, edgecolor='gray', linewidth=0.5))
ax.scatter(*_xy(*BASE_LOC), s=80, c='orange', edgecolors='black')
for p in sim.pick:
    if grid_codes[p[0],p[1]] == CROP_CODE: ax.plot(*_xy(*p), 'x', color='red', markersize=6)
for p, ok in zip(unique_positions(customers), sim.done): ax.plot(*_xy(*p), 'o', color=('blue' if not ok else 'lightblue'), markersize=6)
for dpos in sim.dpos: ax.plot(*_xy(*dpos), 'o', color='black', markersize=6)
st.pyplot(fig)

# Step sim
if st.session_state.get("running") and not sim.fin: sim.step(); time.sleep(0.05); st.rerun()
if sim.fin and st.session_state.get("running"): st.session_state.running=False; st.success("All deliveries complete")
