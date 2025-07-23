import time, numpy as np, matplotlib.pyplot as plt, matplotlib.patches as patches, streamlit as st
from configs import USE_RANDOM_GRID, CROP_CODE, GRID_SIZE, GRID_RES, NUM_DRONES, NUM_CUSTOMERS, DRONE_SPEED, BASE_LOC, COALITION_PERCENT

st.set_page_config(page_title="DroneSim")

cell = GRID_SIZE / GRID_RES
_xy = lambda i, j: ((j + .5) * cell, (i + .5) * cell)
_dist = lambda a, b: ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** .5

def _rand_grid():
    total_cells = GRID_RES * GRID_RES
    num_farms = int(total_cells * COALITION_PERCENT)
    rng = np.random.default_rng()
    all_idx = rng.permutation(total_cells)
    farm_idx = all_idx[:num_farms]
    cust_idx = all_idx[num_farms:num_farms + NUM_CUSTOMERS]
    lab = np.zeros((GRID_RES, GRID_RES), bool)
    for k in farm_idx:
        i, j = divmod(k, GRID_RES)
        lab[i, j] = True
    customers = [divmod(k, GRID_RES) for k in cust_idx]
    return lab, customers

def _fetch_grid(url):
    import requests, rasterio, tempfile
    from io import BytesIO
    from zipfile import ZipFile
    z = ZipFile(BytesIO(requests.get(url, timeout=30).content))
    name = next(n for n in z.namelist() if n.endswith(".tif"))
    with tempfile.NamedTemporaryFile(delete=False, suffix=".tif") as f:
        f.write(z.read(name))
        path = f.name
    with rasterio.open(path) as src:
        data, tr = src.read(1), src.transform
    rows, cols = data.shape
    xs = np.linspace(.5, cols - .5, cols)
    ys = np.linspace(.5, rows - .5, rows)
    world = np.stack(np.meshgrid(xs, ys)[::-1], -1).reshape(-1, 2)
    world = np.array([tr * tuple(p) for p in world])
    mn, span = world.min(0), np.maximum(world.max(0) - world.min(0), 1e-5)
    norm = (world - mn) / span * GRID_SIZE
    gx = np.clip((norm[:, 0] / GRID_SIZE * GRID_RES).astype(int), 0, GRID_RES - 1)
    gy = np.clip((norm[:, 1] / GRID_SIZE * GRID_RES).astype(int), 0, GRID_RES - 1)
    lab = np.zeros((GRID_RES, GRID_RES), bool)
    flat = data.ravel()
    for x, y, v in zip(gx, gy, flat):
        if v == CROP_CODE:
            lab[y, x] = True
    open_cells = np.argwhere(~lab)
    if len(open_cells) <= NUM_CUSTOMERS:
        cust_sel = open_cells
    else:
        cust_sel = open_cells[np.random.choice(len(open_cells), NUM_CUSTOMERS, replace=False)]
    return lab, [tuple(c) for c in cust_sel]

class DroneSim:
    def __init__(self, n, cust, farms):
        self.farms, self.cust = farms, cust
        self.base = tuple(BASE_LOC)
        self.dpos = [self.base] * n
        self.done = [False] * len(cust)
        self.pick = [self._farm(c) for c in cust]
        self.routes = [[self.pick[i], cust[i]] for i in range(len(cust))]
        self.assign = self._alloc()
        self.idx = [0] * n
        self.fin = False
    def _farm(self, c):
        f = np.argwhere(self.farms)
        return tuple(f[np.argmin(np.hypot(f[:, 0] - c[0], f[:, 1] - c[1]))])
    def _alloc(self):
        A, left, loc = [[] for _ in self.dpos], set(range(len(self.cust))), [self.base] * len(self.dpos)
        while left:
            for d in range(len(self.dpos)):
                if not left: break
                best = min(left, key=lambda i: _dist(loc[d], self.pick[i]) + _dist(self.pick[i], self.cust[i]))
                A[d].append(best)
                loc[d] = self.cust[best]
                left.remove(best)
        return A
    def step(self, spd=DRONE_SPEED):
        for d, pos in enumerate(self.dpos):
            rem = spd
            while rem > 1e-9 and self.assign[d]:
                ci = self.assign[d][0]
                tgt = self.routes[ci][self.idx[d]]
                vec = (tgt[0] - pos[0], tgt[1] - pos[1])
                dist = (vec[0] ** 2 + vec[1] ** 2) ** .5
                if dist < 1e-9:
                    self.idx[d] += 1
                    if self.idx[d] == 2:
                        self.done[ci] = True
                        self.assign[d].pop(0)
                        self.idx[d] = 0
                    continue
                step = min(rem, dist)
                k = step / dist
                pos = (pos[0] + vec[0] * k, pos[1] + vec[1] * k)
                rem -= step
            self.dpos[d] = pos
        self.fin = all(self.done)
        return self.fin

url_input = st.text_input("GeoTIFF ZIP URL (optional)")
if "label_grid" not in st.session_state:
    st.session_state.label_grid, st.session_state.customers = (_rand_grid() if not url_input else _fetch_grid(url_input))
if url_input and st.session_state.get("prev_url") != url_input:
    st.session_state.label_grid, st.session_state.customers = _fetch_grid(url_input)
    st.session_state.prev_url = url_input
label_grid = st.session_state.label_grid
customers = st.session_state.customers

if "sim" not in st.session_state:
    st.session_state.sim = DroneSim(NUM_DRONES, customers, label_grid)
sim = st.session_state.sim

c1, c2 = st.columns(2)
if c1.button("Start", disabled=st.session_state.get("running", False)):
    st.session_state.running = True
if c2.button("Stop"):
    st.session_state.running = False

fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(0, GRID_SIZE); ax.set_ylim(0, GRID_SIZE); ax.set_aspect('equal'); ax.invert_yaxis()
for i in range(GRID_RES):
    for j in range(GRID_RES):
        ax.add_patch(patches.Rectangle((j * cell, i * cell), cell, cell,
                                       facecolor=('green' if label_grid[i, j] else 'white'),
                                       edgecolor='gray', linewidth=.5))
ax.scatter(*_xy(*BASE_LOC), s=80, c='orange', edgecolors='black')
for p in sim.pick: ax.plot(*_xy(*p), 'x', color='red', markersize=6)
for p, ok in zip(sim.cust, sim.done): ax.plot(*_xy(*p), 'o', color=('lightblue' if ok else 'blue'), markersize=6)
for p in sim.dpos: ax.plot(*_xy(*p), 'o', color='black', markersize=6)
st.pyplot(fig)

if st.session_state.get("running") and not sim.fin:
    sim.step(); time.sleep(.05); st.rerun()
if sim.fin and st.session_state.get("running"):
    st.session_state.running = False; st.success("All deliveries complete")
