from flask import Flask, render_template_string, request, send_from_directory, abort
import os, pathlib, re

BASE_DIR     = pathlib.Path(__file__).resolve().parent
RESULTS_ROOT = BASE_DIR / 'results'
STAT_ROOT    = RESULTS_ROOT / 'stationary'
NS_ROOT      = RESULTS_ROOT / 'nonstationary'

app = Flask(__name__)

# ─── Static image route ─────────────────────────────────────────────────────
@app.route('/img/<path:filename>')
def send_img(filename):
    full = RESULTS_ROOT / filename
    if not full.is_file():
        abort(404)
    return send_from_directory(RESULTS_ROOT, filename)

# ─── Helpers ────────────────────────────────────────────────────────────────
def list_dirs(path, ignore=None):
    ignore = set(ignore or [])
    return sorted(p.name for p in path.iterdir() if p.is_dir() and p.name not in ignore)

def pretty(name: str) -> str:
    if name.lower() == 'nonstationary':
        return 'Non-Stationary'
    return re.sub(r'_', ' ', name).title()

def collect_images(date, var):
    out = {}
    # Stationary
    sroot = STAT_ROOT / date / var
    if sroot.exists():
        for k in list_dirs(sroot):
            mn = sroot / k / 'mean.png'
            ut = sroot / k / 'uncert.png'
            out[k] = {
                'mean':   f'/img/stationary/{date}/{var}/{k}/mean.png'   if mn.is_file() else None,
                'uncert': f'/img/stationary/{date}/{var}/{k}/uncert.png' if ut.is_file() else None
            }
    # Non-stationary
    nroot = NS_ROOT / date / var
    if nroot.exists():
        m = u = None
        for fn in os.listdir(nroot):
            if fn.endswith('_mean.png'):
                m = f'/img/nonstationary/{date}/{var}/{fn}'
            elif fn.endswith('_uncert.png'):
                u = f'/img/nonstationary/{date}/{var}/{fn}'
        out['nonstationary'] = {'mean': m, 'uncert': u}
    return out

# ─── Main page ──────────────────────────────────────────────────────────────
@app.route('/', methods=['GET'])
def index():
    # Dates
    dates = sorted(set(list_dirs(STAT_ROOT) + list_dirs(NS_ROOT)))
    date  = request.args.get('date', dates[0] if dates else '')
    if dates and date not in dates:
        date = dates[0]

    # Variables
    vs = []
    if (STAT_ROOT / date).exists(): vs += list_dirs(STAT_ROOT / date)
    if (NS_ROOT  / date).exists(): vs += list_dirs(NS_ROOT  / date)
    variables = sorted(set(vs))
    var       = request.args.get('var', variables[0] if variables else '')
    if variables and var not in variables:
        var = variables[0]

    # Images & kernels
    imgs    = collect_images(date, var)
    kernels = list(imgs.keys())  # e.g. ['RBF','Exponential',...,'nonstationary']

    # UI state
    mode         = request.args.get('mode', 'View')            # View | Compare
    view_type    = request.args.get('view_type', 'Stationary') # Stationary | Non-Stationary
    compare_type = request.args.get('compare_type', 'single')  # single | all
    sel_kernel   = request.args.get('kernel', kernels[0] if kernels else '')
    if kernels and sel_kernel not in kernels:
        sel_kernel = kernels[0]
    show_uncert  = request.args.get('uncert','off') == 'on'

    # Build compare selection
    if mode == 'Compare':
        if compare_type == 'single':
            cmp_sel = []
            if 'nonstationary' in imgs:
                cmp_sel.append('nonstationary')
            if sel_kernel != 'nonstationary':
                cmp_sel.append(sel_kernel)
        else:
            cmp_sel = kernels
        kernel = sel_kernel
    else:
        # View mode
        kernel = sel_kernel if view_type=='Stationary' else 'nonstationary'
        cmp_sel = [kernel]

    return render_template_string(TPL,
        dates=dates,
        variables=variables,
        kernels=[k for k in kernels if k!='nonstationary'],
        date=date,
        var=var,
        mode=mode,
        view_type=view_type,
        compare_type=compare_type,
        kernel=kernel,
        cmp_sel=cmp_sel,
        show_uncert=show_uncert,
        imgs=imgs,
        pretty=pretty
    )

# ─── HTML template ─────────────────────────────────────────────────────────
TPL = r"""<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<title>GP Explorer</title>
<link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/css/bootstrap.min.css" rel="stylesheet">
<style>
  body{margin:0;font-family:sans-serif;background:#121212;color:#f0f0f0}
  #sidebar{width:260px;background:#1e1e1e;height:100vh;position:fixed;overflow-y:auto;padding:1rem}
  #sidebar h2{color:#f0a020}#sidebar label{color:#f0a020;font-weight:500}
  #sidebar select,#sidebar input[type=checkbox]+label{background:#333;color:#fff;border:1px solid #f0a020}
  #content{margin-left:260px;padding:1rem;min-height:100vh}
  .btn-primary{background:#f0a020;border:#f0a020;color:#121212}.btn-primary:hover{background:#ffca70}
  .img-thumb{border:2px solid #f0a020;cursor:pointer;max-height:400px;margin:auto;display:block}
  .card-custom{background:#1a1a1a;border:1px solid #f0a020}.card-header{background:#f0a020;color:#121212;font-weight:600}
</style></head><body>

<!-- SIDEBAR -->
<div id="sidebar"><h2>Controls</h2><form>
  <!-- Date -->
  <div class="mb-3"><label>Date</label>
    <select name="date" class="form-select form-select-sm" onchange="this.form.submit()">
      {% for d in dates %}<option {% if d==date %}selected{% endif %}>{{d}}</option>{% endfor %}
    </select>
  </div>
  <!-- Variable -->
  <div class="mb-3"><label>Variable</label>
    <select name="var" class="form-select form-select-sm" onchange="this.form.submit()">
      {% for v in variables %}<option {% if v==var %}selected{% endif %}>{{v}}</option>{% endfor %}
    </select>
  </div>
  <!-- Mode -->
  <div class="mb-3"><label>Mode</label><br>
    <div class="form-check form-check-inline">
      <input class="form-check-input" type="radio" name="mode" value="View" {% if mode=='View' %}checked{% endif %} onchange="this.form.submit()">
      <label class="form-check-label">View</label>
    </div>
    <div class="form-check form-check-inline">
      <input class="form-check-input" type="radio" name="mode" value="Compare" {% if mode=='Compare' %}checked{% endif %} onchange="this.form.submit()">
      <label class="form-check-label">Compare</label>
    </div>
  </div>

  {% if mode=='View' %}
    <!-- Type -->
    <div class="mb-3"><label>Type</label><br>
      <div class="form-check form-check-inline">
        <input class="form-check-input" type="radio" name="view_type" value="Stationary" {% if view_type=='Stationary' %}checked{% endif %} onchange="this.form.submit()">
        <label class="form-check-label">Stationary</label>
      </div>
      <div class="form-check form-check-inline">
        <input class="form-check-input" type="radio" name="view_type" value="Non-Stationary" {% if view_type=='Non-Stationary' %}checked{% endif %} onchange="this.form.submit()">
        <label class="form-check-label">Non-Stationary</label>
      </div>
    </div>
    <!-- Kernel dropdown always visible -->
    <div class="mb-3"><label>Kernel</label>
      <select name="kernel" class="form-select form-select-sm">
        {% if view_type=='Non-Stationary' %}
          <option value="nonstationary" {% if kernel=='nonstationary' %}selected{% endif %}>Non-Stationary</option>
        {% else %}
          {% for k in kernels %}<option value="{{k}}" {% if k==kernel %}selected{% endif %}>{{pretty(k)}}</option>{% endfor %}
        {% endif %}
      </select>
    </div>
  {% else %}
    <!-- Compare controls -->
    <div class="mb-3"><label>Compare With</label><br>
      <div class="form-check form-check-inline">
        <input class="form-check-input" type="radio" name="compare_type" value="single" {% if compare_type=='single' %}checked{% endif %}>
        <label class="form-check-label">One Kernel</label>
      </div>
      <div class="form-check form-check-inline">
        <input class="form-check-input" type="radio" name="compare_type" value="all" {% if compare_type=='all' %}checked{% endif %}>
        <label class="form-check-label">All Kernels</label>
      </div>
    </div>
    {% if compare_type=='single' %}
      <div class="mb-3"><label>Select Kernel</label>
        <select name="kernel" class="form-select form-select-sm">
          {% for k in kernels %}<option value="{{k}}" {% if k==kernel %}selected{% endif %}>{{pretty(k)}}</option>{% endfor %}
        </select>
      </div>
    {% endif %}
  {% endif %}

  <div class="mb-3 form-check">
    <input class="form-check-input" type="checkbox" name="uncert" {% if show_uncert %}checked{% endif %}>
    <label class="form-check-label">Show Uncertainty</label>
  </div>
  <button type="submit" class="btn btn-primary w-100">Apply</button>
</form></div>

<!-- CONTENT -->
<div id="content">
  {% if mode=='View' %}
    <h4 class="text-warning mb-3">{{ pretty(kernel) }}</h4>
    <div class="row g-4">
      <div class="col-md-6 text-center">
        <h6>Mean</h6>
        {% if imgs[kernel]['mean'] %}
          <img src="{{imgs[kernel]['mean']}}" class="img-thumb img-fluid" data-bs-toggle="modal" data-bs-target="#m" data-src="{{imgs[kernel]['mean']}}">
        {% else %}
          <p class="text-danger">Missing</p>
        {% endif %}
      </div>
      {% if show_uncert %}
      <div class="col-md-6 text-center">
        <h6>Uncertainty</h6>
        {% if imgs[kernel]['uncert'] %}
          <img src="{{imgs[kernel]['uncert']}}" class="img-thumb img-fluid" data-bs-toggle="modal" data-bs-target="#m" data-src="{{imgs[kernel]['uncert']}}">
        {% else %}
          <p class="text-danger">Missing</p>
        {% endif %}
      </div>
      {% endif %}
    </div>

  {% else %}
    <h4 class="text-warning mb-3">Compare</h4>
    <div class="row g-4">
      {% for k in cmp_sel %}
        <div class="col-md-{{ 12 // (cmp_sel|length if cmp_sel|length<=4 else 4) }}">
          <div class="card card-custom">
            <div class="card-header text-center">{{ pretty(k) }}</div>
            <div class="card-body text-center">
              <p class="mb-1">Mean</p>
              {% if imgs[k]['mean'] %}
                <img src="{{imgs[k]['mean']}}" class="img-thumb mb-2 img-fluid" data-bs-toggle="modal" data-bs-target="#m" data-src="{{imgs[k]['mean']}}">
              {% else %}
                <p class="text-danger small">Missing</p>
              {% endif %}
              {% if show_uncert %}
                <p class="mt-2 mb-1">Uncertainty</p>
                {% if imgs[k]['uncert'] %}
                  <img src="{{imgs[k]['uncert']}}" class="img-thumb img-fluid" data-bs-toggle="modal" data-bs-target="#m" data-src="{{imgs[k]['uncert']}}">
                {% else %}
                  <p class="text-danger small">Missing</p>
                {% endif %}
              {% endif %}
            </div>
          </div>
        </div>
      {% endfor %}
    </div>

  {% endif %}
</div>

<!-- Modal Viewer -->
<div class="modal fade" id="m" tabindex="-1">
  <div class="modal-dialog modal-xl modal-dialog-centered">
    <div class="modal-content bg-dark border-0">
      <img id="modalImg" src="#" class="img-fluid">
    </div>
  </div>
</div>
<script src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/js/bootstrap.bundle.min.js"></script>
<script>
  document.addEventListener('click', e => {
    if (e.target.dataset.src) {
      document.getElementById('modalImg').src = e.target.dataset.src;
    }
  });
</script>
</body></html>"""

if __name__=='__main__':
    app.run(debug=True)
