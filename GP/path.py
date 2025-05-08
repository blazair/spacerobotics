import os
import shutil
import re

# ─── Paths ────────────────────────────────────────────────────────────────
STATIONARY_ROOT = r"C:\ASU\Semester 2\space robotics and ai\codeyy\GP\results"
NONSTAT_ROOT   = r"C:\ASU\Semester 2\space robotics and ai\codeyy\GP\results(ns)"
TEST_ROOT       = r"C:\ASU\Semester 2\space robotics and ai\codeyy\GP\test"

# ─── Your sensor variables ─────────────────────────────────────────────────
sensor_variables = [
    "Chlorophyll_ug_L",
    "Conductivity_uS_cm",
    "Depth_Sonar",
    "Dissolved_Oxygen_Concentration_mg_L",
    "Dissolved_Oxygen_Saturation",
    "pH",
    "Temperature_°C"
]

# ─── Helper to make a comparable string ────────────────────────────────────
def sanitize(name: str) -> str:
    # replace non-alphanumerics with underscore, collapse repeats, lowercase
    s = re.sub(r'[^0-9A-Za-z]', '_', name)
    s = re.sub(r'_+', '_', s)
    return s.lower().strip('_')

# ─── 1) Recreate test/ ─────────────────────────────────────────────────────
if os.path.exists(TEST_ROOT):
    shutil.rmtree(TEST_ROOT)
os.makedirs(TEST_ROOT)

# ─── 2) Loop over dates ───────────────────────────────────────────────────
for date in os.listdir(STATIONARY_ROOT):
    date_src = os.path.join(STATIONARY_ROOT, date)
    if not os.path.isdir(date_src):
        continue

    date_dst = os.path.join(TEST_ROOT, date)
    os.makedirs(date_dst, exist_ok=True)

    # ─── 3) For each sensor variable ────────────────────────────────────────
    for var in sensor_variables:
        var_src = os.path.join(date_src, var)
        if not os.path.isdir(var_src):
            print(f"[WARN] No stationary folder for {date}/{var}")
            continue

        # replicate variable folder
        var_dst = os.path.join(date_dst, var)
        os.makedirs(var_dst, exist_ok=True)

        # copy stationary kernels
        kernels_src = os.path.join(var_src, 'kernels')
        kernels_dst = os.path.join(var_dst, 'kernels')
        if not os.path.isdir(kernels_src):
            print(f"[WARN] Missing kernels/ for {date}/{var}")
            continue
        shutil.copytree(kernels_src, kernels_dst)

        # create ns subfolder
        ns_dst = os.path.join(kernels_dst, 'ns')
        os.makedirs(ns_dst, exist_ok=True)

        # ─── 4) Find matching NS folder name by fuzzy match ────────────────
        base_key   = sanitize(var)
        ns_date_dir = os.path.join(NONSTAT_ROOT, date)
        if not os.path.isdir(ns_date_dir):
            print(f"[WARN] No results(ns) date folder for {date}")
            continue

        # look for any folder where sanitize(folder) contains base_key
        candidates = [
            vf for vf in os.listdir(ns_date_dir)
            if os.path.isdir(os.path.join(ns_date_dir, vf))
            and base_key in sanitize(vf)
        ]
        if not candidates:
            print(f"[WARN] No NS match for {date}/{var} (sanitized='{base_key}')")
            continue

        # pick the first match
        ns_folder = candidates[0]
        ns_src_base = os.path.join(ns_date_dir, ns_folder)

        # copy all .pngs
        for fname in os.listdir(ns_src_base):
            if fname.lower().endswith('.png'):
                shutil.copy2(
                    os.path.join(ns_src_base, fname),
                    os.path.join(ns_dst, fname)
                )

print("✅ Done! Your `test/` tree now mirrors `results/` with an added `kernels/ns` containing the real NS images.")
