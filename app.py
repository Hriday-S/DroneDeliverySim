# app.py — top-N farms, centered base, stable state, metrics, gridlines
import os, glob, time
import numpy as np
import streamlit as st
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb

from DroneSim import DroneSim

# --- config
CDL_TIF_PATH   = "tif/CDL_2024_17.tif"      # il cdl 2024
STATE_SHP_PATH = "IL_BNDY_State/IL_BNDY_State_Py.shp"   # il boundary
DBF_GLOB       = "tif/*.dbf"                # cdl vat dbf (colors)

GRID_SIZE        = 100
DEFAULT_GRID_RES = 300
NUM_DRONES       = 3
NUM_CUSTOMERS    = 80
DRONE_SPEED      = 5.0

DEFAULT_TARGET_CODE = 54        # default crop
FARM_COUNT          = 250       # pick top-N farm cells
MIN_NONFARM_BUFFER  = 2 * NUM_CUSTOMERS  # leave space for customers

st.set_page_config(page_title="DroneSim", layout="centered")

# --- color utils
@st.cache_data(show_spinner=False, max_entries=3)
def _norm_hex(h):
    if h is None: return None
    s = str(h).strip()
    if not s: return None
    if s.lower().startswith('0x'): s = s[2:]
    if s.startswith('#'): s = s[1:]
    if len(s) == 8: s = s[-6:]  # drop alpha
    if len(s) != 6 or any(c not in "0123456789abcdefABCDEF" for c in s): return None
    return '#' + s.upper()

def _hex_to_u8(h: str) -> np.ndarray:
    from matplotlib.colors import to_rgb as _to_rgb
    return (np.array(_to_rgb(h)) * 255).astype(np.uint8)

@st.cache_data(show_spinner=False, max_entries=3)
def load_colors_and_names(dbf_glob_pattern: str, tif_path: str, csv_path="crop_codes.csv"):
    colors, names = {}, {}
    # dbf first if present
    try:
        from dbfread import DBF
        files = sorted(glob.glob(dbf_glob_pattern))
        if files:
            base = os.path.basename(tif_path).split('.')[0]
            preferred = [p for p in files if base in os.path.basename(p)]
            dbf_path = preferred[0] if preferred else files[0]
            for rec in DBF(dbf_path, load=True, char_decode_errors='ignore'):
                if 'Value' not in rec: continue
                v = int(rec['Value'])
                c = _norm_hex(rec.get('Color')) if rec.get('Color') else None
                if c is None and all(k in rec for k in ('Red','Green','Blue')):
                    try:
                        r,g,b = int(rec['Red']), int(rec['Green']), int(rec['Blue'])
                        c = '#%02X%02X%02X' % (r,g,b)
                    except Exception:
                        c = None
                if c: colors[v] = c
                nm = rec.get('Class_Name') or rec.get('ClassName') or rec.get('CLASS_NAME') or ''
                if nm: names[v] = str(nm)
    except Exception:
        pass
    # csv overrides
    if os.path.isfile(csv_path):
        try:
            import pandas as pd
            df = pd.read_csv(csv_path)
            for _, row in df.iterrows():
                try:
                    v = int(str(row['code']).strip(), 0)  # dec or 0x
                except Exception:
                    continue
                c = _norm_hex(row.get('color'))
                if c: colors[v] = c
                nm = row.get('name')
                if isinstance(nm, str) and nm.strip():
                    names[v] = nm.strip()
        except Exception:
            pass
    if not colors:
        colors[0] = "#CCCCCC"
    return colors, names

def color_for_code(code: int, colors: dict) -> str:
    if code in colors: return colors[code]
    return ['#CC3333','#3366CC','#33AA55','#AA33AA','#CC9933','#008888'][abs(int(code)) % 6]

# --- clip raster to state (crs-safe)
@st.cache_data(show_spinner=False, max_entries=3)
def clip_cdl_to_state(tif_path: str, shp_path: str, fill_value: int = 255):
    import rasterio
    import fiona
    from shapely.geometry import shape, mapping
    from shapely.ops import transform as shp_transform
    from rasterio.mask import mask as rio_mask
    from pyproj import Transformer

    with rasterio.open(tif_path) as src:
        r_crs = src.crs
        with fiona.open(shp_path) as shp:
            s_crs = shp.crs
            geoms_src = [shape(feat["geometry"]) for feat in shp if feat["geometry"]]
        if not geoms_src:
            raise ValueError("state shapefile has no geometry.")
        if s_crs and r_crs and s_crs != r_crs:
            tfm = Transformer.from_crs(s_crs, r_crs, always_xy=True)
            reproj = lambda x, y, z=None: tfm.transform(x, y)
            geoms = [shp_transform(reproj, g) for g in geoms_src]
        else:
            geoms = geoms_src
        arr, _ = rio_mask(src, [mapping(g) for g in geoms], crop=True, filled=True, nodata=fill_value)
        arr = arr[0]
    return arr, fill_value

@st.cache_data(show_spinner=False, max_entries=3)
def present_codes_from_array(arr: np.ndarray, fill_value: int):
    u = np.unique(arr)
    u = u[u != fill_value]
    return [int(x) for x in u.tolist()]

# --- per-cell fraction (no threshold yet)
@st.cache_data(show_spinner=False, max_entries=3)
def fraction_field_for_code(arr: np.ndarray, fill_value: int, grid_res: int, target_code: int):
    H, W = arr.shape
    valid = arr != fill_value
    if not valid.any():
        frac = np.zeros((grid_res, grid_res), dtype=np.float32)
        state_mask = np.zeros((grid_res, grid_res), dtype=bool)
        return frac, state_mask, (0, grid_res-1, 0, grid_res-1)

    gx_1d = np.clip(((np.arange(W) + 0.5) / W * grid_res).astype(np.int32), 0, grid_res - 1)
    gy_1d = np.clip(((np.arange(H) + 0.5) / H * grid_res).astype(np.int32), 0, grid_res - 1)
    gx = np.tile(gx_1d, H)
    gy = np.repeat(gy_1d, W)

    vals  = arr.ravel().astype(np.int32)
    vflat = valid.ravel().astype(np.int32)

    totals = np.zeros((grid_res, grid_res), dtype=np.int32)
    hits   = np.zeros((grid_res, grid_res), dtype=np.int32)
    np.add.at(totals, (gy, gx), vflat)
    np.add.at(hits,   (gy, gx), (vals == int(target_code)).astype(np.int32))

    with np.errstate(divide='ignore', invalid='ignore'):
        frac = np.where(totals > 0, hits / totals, 0.0).astype(np.float32)

    state_mask = totals > 0
    ys, xs = np.where(state_mask)
    bbox = (int(ys.min()), int(ys.max()), int(xs.min()), int(xs.max())) if ys.size else (0, grid_res-1, 0, grid_res-1)
    return frac, state_mask, bbox

# --- make background image (just highlight selected crop cells)
def build_rgb_image_single(farms: np.ndarray, state_mask: np.ndarray, target_hex: str):
    h, w = state_mask.shape
    img = np.full((h, w, 3), 255, dtype=np.uint8)
    interior = np.array([240, 240, 240], np.uint8)
    img[state_mask] = interior
    img[farms] = _hex_to_u8(target_hex)
    return img

# --- small helpers
def _safe_base(base_guess, mask):
    bi, bj = map(int, base_guess)
    h, w = mask.shape
    if 0 <= bi < h and 0 <= bj < w and mask[bi, bj]:
        return (bi, bj)
    cells = np.argwhere(mask)
    if cells.size == 0:
        return (0, 0)
    di = cells[:, 0] - bi
    dj = cells[:, 1] - bj
    k = int(np.argmin(np.hypot(di, dj)))
    return tuple(map(int, cells[k]))

# --- ui
res = st.number_input("grid resolution (cells per side)", min_value=80, max_value=500, value=DEFAULT_GRID_RES, step=20)
show_gridlines = st.checkbox("show grid lines", value=False)

# reset on res change
if st.session_state.get("prev_res") != res:
    for k in ["frac", "state_mask", "bbox", "farms", "customers", "sim", "running", "safe_base", "crop_name_cache"]:
        st.session_state.pop(k, None)
    st.session_state.prev_res = int(res)
    st.rerun()
GRID_RES = int(res)

# --- load & prep
try:
    arr_clip, FILL = clip_cdl_to_state(CDL_TIF_PATH, STATE_SHP_PATH, fill_value=255)
except Exception as e:
    st.error(f"clip failed: {e}")
    st.stop()

colors, names = load_colors_and_names(DBF_GLOB, CDL_TIF_PATH)
present_codes = present_codes_from_array(arr_clip, FILL)
if not present_codes:
    st.error("no classes inside the clip.")
    st.stop()

# crop selection
if "selected_code" not in st.session_state:
    st.session_state.selected_code = DEFAULT_TARGET_CODE if DEFAULT_TARGET_CODE in present_codes else present_codes[0]
options = [f"{names.get(c, 'Class '+str(c))} ({c})" for c in present_codes]
default_idx = max(0, present_codes.index(st.session_state.selected_code))
selected_label = st.selectbox("target crop:", options, index=default_idx)
try:
    TARGET_CODE = int(selected_label.split("(")[-1][:-1])
except Exception:
    TARGET_CODE = st.session_state.selected_code

if st.session_state.get("selected_code") != TARGET_CODE:
    st.session_state.selected_code = TARGET_CODE
    for k in ["frac", "state_mask", "bbox", "farms", "customers", "sim", "running", "safe_base", "crop_name_cache"]:
        st.session_state.pop(k, None)
    st.rerun()

# per-cell fractions (cached)
if "frac" not in st.session_state:
    frac, state_mask, bbox = fraction_field_for_code(arr_clip, FILL, GRID_RES, TARGET_CODE)
    st.session_state.frac = frac
    st.session_state.state_mask = state_mask
    st.session_state.bbox = bbox
frac = st.session_state.frac
state_mask = st.session_state.state_mask
bbox = st.session_state.bbox

# --- choose farms: top-N with a buffer for customers
if "farms" not in st.session_state:
    farms = np.zeros_like(state_mask, dtype=bool)
    positive = (frac > 0) & state_mask
    pos_count = int(positive.sum())
    state_cells = int(state_mask.sum())

    max_farms_allowed = max(0, state_cells - MIN_NONFARM_BUFFER)
    desired_k = min(FARM_COUNT, pos_count, max_farms_allowed)
    if desired_k < FARM_COUNT:
        st.info(f"FARM_COUNT {FARM_COUNT} -> {desired_k} to leave space for customers.", icon="⚠️")

    if desired_k > 0:
        scores = np.where(positive, frac, -np.inf).ravel()
        idx = np.argpartition(scores, -desired_k)[-desired_k:]
        farms.flat[idx] = True

    if not farms.any():
        st.error("no farm cells for this crop (frac > 0). try another crop or tweak resolution.")
        st.stop()

    st.session_state.farms = farms
farms = st.session_state.farms

# --- customers (fixed per crop+res)
if "customers" not in st.session_state:
    seed = (int(TARGET_CODE) * 73856093) ^ (int(GRID_RES) * 19349663)
    rng = np.random.default_rng(np.uint64(seed))
    cands = np.argwhere(state_mask & ~farms)
    if len(cands) == 0:
        st.error("no non-farm cells for customers. reduce FARM_COUNT or resolution.")
        st.stop()
    num_customers = min(NUM_CUSTOMERS, len(cands))
    sel_idx = rng.choice(len(cands), size=num_customers, replace=False)
    st.session_state.customers = [tuple(map(int, p)) for p in cands[sel_idx]]
customers = st.session_state.customers

# --- base (center then snap)
imin, imax, jmin, jmax = bbox
center_guess = ((imin + imax) // 2, (jmin + jmax) // 2)
if "safe_base" not in st.session_state:
    st.session_state.safe_base = _safe_base(center_guess, state_mask)
safe_base = st.session_state.safe_base

# --- sim (stable)
if "sim" not in st.session_state:
    st.session_state.sim = DroneSim(
        n=NUM_DRONES,
        customers=customers,
        farms=farms,
        base_loc=safe_base,
        speed=float(DRONE_SPEED),
        eps=1e-6,
    )
sim = st.session_state.sim

# --- plot
bbox_h = (imax - imin + 1); bbox_w = (jmax - jmin + 1)
PLOT_W = GRID_SIZE; PLOT_H = GRID_SIZE * (bbox_h / bbox_w)
cell_w = PLOT_W / bbox_w; cell_h = PLOT_H / bbox_h
def _xy(i, j): return ((j - jmin + 0.5) * cell_w, (i - imin + 0.5) * cell_h)

target_hex = color_for_code(TARGET_CODE, colors)
img_full = build_rgb_image_single(farms, state_mask, target_hex)
img = img_full[imin:imax+1, jmin:jmax+1, :]

fig, ax = plt.subplots(figsize=(6, 6))
ax.imshow(img, origin='upper', extent=[0, PLOT_W, PLOT_H, 0], interpolation='nearest')
ax.set_xlim(0, PLOT_W); ax.set_ylim(PLOT_H, 0); ax.set_aspect('equal'); ax.axis('off')

if show_gridlines:
    for gi in range(imin, imax + 2):
        y = (gi - imin) * cell_h
        ax.plot([0, PLOT_W], [y, y], color='black', linewidth=0.3, alpha=0.25)
    for gj in range(jmin, jmax + 2):
        x = (gj - jmin) * cell_w
        ax.plot([x, x], [0, PLOT_H], color='black', linewidth=0.3, alpha=0.25)

ax.scatter(*_xy(*safe_base), s=60, c='orange', edgecolors='black', linewidths=0.5, zorder=5)

for p in sim.pick:
    if imin <= p[0] <= imax and jmin <= p[1] <= jmax and farms[p[0], p[1]]:
        ax.plot(*_xy(*p), 'x', color='red', markersize=5, linewidth=1.0, zorder=6)

seen = set()
for i, p in enumerate(customers):
    if p in seen: continue
    seen.add(p)
    ax.plot(*_xy(*p), 'o', color=('blue' if not sim.done[i] else 'lightblue'), markersize=4, zorder=5)

for dpos in sim.dpos:
    ax.plot(*_xy(*dpos), 'o', color='black', markersize=4, zorder=6)

st.pyplot(fig, clear_figure=True)
plt.close(fig)

# --- tiny metrics
crop_name = st.session_state.get("crop_name_cache") or ""
if not crop_name:
    crop_name = names.get(TARGET_CODE, f"Class {TARGET_CODE}")
    st.session_state.crop_name_cache = crop_name
total_cells = int(state_mask.sum())
farm_cells = int(farms.sum())
avg_frac = float(frac[farms].mean()) if farm_cells > 0 else 0.0
st.caption(f"target: **{crop_name} ({TARGET_CODE})** · farms: **{farm_cells}** / {total_cells} · "
           f"avg frac: **{avg_frac:.3f}** · customers: **{len(customers)}** · drones: **{NUM_DRONES}**")

# --- controls
c1, c2, c3 = st.columns(3)
if c1.button("start"): st.session_state.running = True
if c2.button("stop"):  st.session_state.running = False
if c3.button("reset"):
    for k in ["frac", "state_mask", "bbox", "farms", "customers", "sim", "running", "safe_base", "crop_name_cache"]:
        st.session_state.pop(k, None)
    st.rerun()

# --- stepper
if st.session_state.get("running") and not sim.fin:
    for _ in range(8):
        if sim.step(): break
    time.sleep(0.01)
    st.rerun()

if sim.fin and st.session_state.get("running"):
    st.session_state.running = False
    st.success("all deliveries complete")
