/**
 * app.js – Frontend logic for HSLM Dynamic Analysis Web App
 */
'use strict';

const App = (() => {
  const DYNAMIC_SWEEP_ENABLED = false;
  // ── Train list ──────────────────────────────────────────────────────────
  const TRAINS = ['A1','A2','A3','A4','A5','A6','A7','A8','A9','A10'];
  let _selectedTrains = new Set(TRAINS);
  let _sweepJobId = null;
  let _pollTimer = null;
  let _beamData = {}; // Stores typical beam parameters from CSV

  // ── Init ────────────────────────────────────────────────────────────────
  function init() {
    _buildTrainCheckboxes();
    _bindTabs();
    document.getElementById('btn-select-all').addEventListener('click', _toggleSelectAll);
    document.getElementById('btn-run-vibration').addEventListener('click', runVibration);
    document.getElementById('btn-run-sweep').addEventListener('click', runSweep);
    document.getElementById('btn-stop-sweep').addEventListener('click', stopSweep);
    document.getElementById('beam_type').addEventListener('change', _applyBeamType);

    _loadBeamPropCSV();

    if (!DYNAMIC_SWEEP_ENABLED) {
      const sweepBtn = document.getElementById('btn-run-sweep');
      const stopBtn = document.getElementById('btn-stop-sweep');
      sweepBtn.disabled = true;
      sweepBtn.textContent = 'Sweep Disabled';
      stopBtn.disabled = true;
      _showError('Dynamic Sweep đã bị tắt bởi quản trị viên để tránh quá tải server.');
    }

    toggleProfileInputs();
  }

  // ── Profile UI Toggle ───────────────────────────────────────────────────
  function toggleProfileInputs() {
    const type = document.getElementById('profile_type').value;
    const psdDiv = document.getElementById('psd-inputs');
    const bumpDiv = document.getElementById('bump-inputs');

    if (!psdDiv || !bumpDiv) return;

    psdDiv.classList.add('hidden');
    bumpDiv.classList.add('hidden');

    if (type === '1') {
      psdDiv.classList.remove('hidden');
    } else if (type === '3') {
      bumpDiv.classList.remove('hidden');
    }
  }

  // ── Beam Properties CSV ────────────────────────────────────────────────
  async function _loadBeamPropCSV() {
    console.log("App: Loading beam_prop.csv...");
    try {
      const resp = await fetch('/static/beam_prop.csv');
      if (!resp.ok) {
        console.warn('App: Could not find beam_prop.csv (Status:', resp.status, ')');
        return;
      }
      const text = await resp.text();
      const lines = text.trim().split(/\r?\n/);
      if (lines.length < 2) return;

      const headers = lines[0].split(',').map(h => h.trim());
      const select = document.getElementById('beam_type');
      if (!select) return;

      for (let i = 1; i < lines.length; i++) {
        const values = lines[i].split(',').map(v => v.trim());
        if (values.length < headers.length) continue;

        const row = {};
        headers.forEach((h, idx) => {
          row[h] = values[idx];
        });

        _beamData[row.id] = row;

        // Add to dropdown
        const opt = document.createElement('option');
        opt.value = row.id;
        opt.textContent = row.name;
        select.appendChild(opt);
      }
      console.log(`App: Successfully loaded ${Object.keys(_beamData).length} typical sections.`);
    } catch (err) {
      console.error('App: Error loading beam_prop.csv:', err);
    }
  }

  function _applyBeamType() {
    const id = document.getElementById('beam_type').value;
    if (id === 'custom') return;
    const data = _beamData[id];
    if (!data) return;

    // List of numeric fields to update
    const fields = [
      'L', 'E', 'Ixx', 'Iyy', 'Ixy', 'G', 'J', 
      'I_theta', 'rho', 'I', 'damping_pct', 'ele_per_spacing'
    ];

    fields.forEach(f => {
      const el = document.getElementById(f);
      if (el && data[f] !== undefined) {
        el.value = data[f];
        // Add a subtle highlight effect
        el.classList.add('field-highlight');
        setTimeout(() => el.classList.remove('field-highlight'), 1000);
      }
    });
  }

  // ── Tabs ────────────────────────────────────────────────────────────────
  function _bindTabs() {
    document.querySelectorAll('.tab-btn').forEach(btn => {
      btn.addEventListener('click', () => goToTab(btn.dataset.tab));
    });
  }

  function goToTab(tabId) {
    document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.getElementById(tabId).classList.add('active');
    document.querySelector(`[data-tab="${tabId}"]`).classList.add('active');
  }

  // ── Train checkboxes ────────────────────────────────────────────────────
  function _buildTrainCheckboxes() {
    const container = document.getElementById('train-checkboxes');
    if (!container) return;
    container.innerHTML = '';
    TRAINS.forEach(name => {
      const label = document.createElement('label');
      label.className = 'train-check-label checked';
      label.dataset.train = name;

      const cb = document.createElement('input');
      cb.type = 'checkbox';
      cb.value = name;
      cb.checked = true;
      cb.addEventListener('change', () => {
        if (cb.checked) {
          _selectedTrains.add(name);
          label.classList.add('checked');
        } else {
          _selectedTrains.delete(name);
          label.classList.remove('checked');
        }
      });

      label.appendChild(cb);
      label.appendChild(document.createTextNode(name));
      container.appendChild(label);
    });
  }

  function _toggleSelectAll() {
    const allChecked = _selectedTrains.size === TRAINS.length;
    _selectedTrains = allChecked ? new Set() : new Set(TRAINS);
    document.querySelectorAll('.train-check-label').forEach(label => {
      const name = label.dataset.train;
      const cb = label.querySelector('input');
      if (_selectedTrains.has(name)) {
        label.classList.add('checked');
        cb.checked = true;
      } else {
        label.classList.remove('checked');
        cb.checked = false;
      }
    });
    document.getElementById('btn-select-all').textContent =
      _selectedTrains.size === 0 ? 'Tích chọn tất cả' : 'Bỏ chọn tất cả';
  }

  // ── Collect bridge params from form ─────────────────────────────────────
  function _getBridgeParams() {
    return {
      L:               parseFloat(document.getElementById('L').value)              || 50,
      E:               parseFloat(document.getElementById('E').value)              || 3.5e10,
      Ixx:             parseFloat(document.getElementById('Ixx').value)            || 51.3,
      Iyy:             parseFloat(document.getElementById('Iyy').value)            || 25.0,
      Ixy:             parseFloat(document.getElementById('Ixy').value)            || 0.0,
      G:               parseFloat(document.getElementById('G').value)              || 1.35e10,
      J:               parseFloat(document.getElementById('J').value)              || 5.0,
      I_theta:         parseFloat(document.getElementById('I_theta').value)        || 80000,
      I:               parseFloat(document.getElementById('I').value)              || 51.3,
      rho:             parseFloat(document.getElementById('rho').value)            || 69000,
      damping_pct:     parseFloat(document.getElementById('damping_pct').value)    || 2.0,
      ele_per_spacing: parseInt(document.getElementById('ele_per_spacing').value)  || 2,
      track_type:      document.getElementById('track_type').value,
      profile: {
        type:     parseInt(document.getElementById('profile_type').value) || 0,
        psd_type: document.getElementById('psd_type').value,
        seed:     parseInt(document.getElementById('profile_seed').value) || 0,
        amp:      parseFloat(document.getElementById('profile_amp').value) || 0,
        length:   parseFloat(document.getElementById('profile_length').value) || 0,
      }
    };
  }

  // ── Window 2: Free Vibration ─────────────────────────────────────────────
  async function runVibration() {
    const btn   = document.getElementById('btn-run-vibration');
    const label = document.getElementById('btn-vib-label');
    const res   = document.getElementById('vib-results');
    res.classList.add('hidden');
    btn.disabled = true;
    label.textContent = '⏳ Đang tính toán...';

    const body = { bridge: _getBridgeParams(), n_modes: 3 };

    try {
      const resp = await fetch('/api/check-vibration', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      if (!resp.ok) throw new Error(`Server error ${resp.status}`);
      const data = await resp.json();
      _renderVibrationResults(data);
    } catch (err) {
      alert('Lỗi khi gọi API: ' + err.message);
    } finally {
      btn.disabled = false;
      label.textContent = '▶ Run Eigenvalue Analysis';
    }
  }

  function _renderVibrationResults(data) {
    const tbody = document.getElementById('vib-table-body');
    tbody.innerHTML = '';
    const rows = data.mode_rows || [];

    for (let i = 0; i < rows.length; i += 1) {
      const m = rows[i];
      const typeLabel = m.mode_type.charAt(0).toUpperCase() + m.mode_type.slice(1);
      const modeLabel = `Mode ${m.mode_idx} - ${typeLabel}`;
      
      const imgTag = m.img_mode_shape
        ? `<img class="mode-inline-img" alt="${modeLabel}" src="data:image/png;base64,${m.img_mode_shape}" />`
        : '<span class="hint-text">Không có dữ liệu</span>';
        
      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td style="white-space: nowrap;"><strong>${modeLabel}</strong></td>
        <td style="color: var(--accent-2);"><strong>${m.freq_hz.toFixed(4)}</strong></td>
        <td>${imgTag}</td>
      `;
      tbody.appendChild(tr);
    }

    const verdict = document.getElementById('vib-verdict');
    if (data.verdict === 'warning') {
      verdict.className = 'verdict-box warning';
      verdict.innerHTML = `<span class="verdict-icon">🚨</span><span>${data.verdict_text}<br/>f1 governing = ${data.governing_f1_hz.toFixed(3)} Hz</span>`;
    } else {
      verdict.className = 'verdict-box ok';
      verdict.innerHTML = `<span class="verdict-icon">✅</span><span>${data.verdict_text}<br/>f1 governing = ${data.governing_f1_hz.toFixed(3)} Hz</span>`;
    }

    document.getElementById('vib-results').classList.remove('hidden');
  }

  // ── Window 3: Dynamic Sweep ──────────────────────────────────────────────
  async function runSweep() {
    if (!DYNAMIC_SWEEP_ENABLED) {
      _showError('Dynamic Sweep đang bị tắt bởi quản trị viên.');
      return;
    }

    if (_selectedTrains.size === 0) {
      alert('Vui lòng chọn ít nhất một loại tàu HSLM.');
      return;
    }

    const sweepBtn = document.getElementById('btn-run-sweep');
    const stopBtn = document.getElementById('btn-stop-sweep');
    sweepBtn.disabled = true;
    stopBtn.disabled = false;

    document.getElementById('results-dashboard').classList.add('hidden');
    document.getElementById('sweep-error').classList.add('hidden');
    _showProgress(true);

    const numCoaches = parseInt(document.getElementById('num_coaches').value) || null;
    const body = {
      bridge:      _getBridgeParams(),
      train_names: [..._selectedTrains],
      num_coaches: numCoaches,
      v_min_kmh:   parseFloat(document.getElementById('v_min').value) || 250,
      v_max_kmh:   parseFloat(document.getElementById('v_max').value) || 350,
      v_step_kmh:  parseFloat(document.getElementById('v_step').value) || 10,
    };

    try {
      const resp = await fetch('/api/run-dynamic', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      if (!resp.ok) throw new Error(`Server error ${resp.status}`);
      const data = await resp.json();
      _sweepJobId = data.job_id;
      _startPolling(sweepBtn, stopBtn);
    } catch (err) {
      _showError('Lỗi khởi động phân tích: ' + err.message);
      sweepBtn.disabled = false;
      stopBtn.disabled = true;
    }
  }

  async function stopSweep() {
    if (!_sweepJobId) return;
    const stopBtn = document.getElementById('btn-stop-sweep');
    stopBtn.disabled = true;
    try {
      const resp = await fetch(`/api/stop-dynamic/${_sweepJobId}`, { method: 'POST' });
      if (!resp.ok) throw new Error(`Server error ${resp.status}`);
      _updateProgressUI(0, 'Đang gửi yêu cầu dừng...');
    } catch (err) {
      _showError('Không thể dừng tác vụ: ' + err.message);
      stopBtn.disabled = false;
    }
  }

  function _startPolling(sweepBtn, stopBtn) {
    clearInterval(_pollTimer);
    _pollTimer = setInterval(() => _pollStatus(sweepBtn, stopBtn), 1500);
  }

  async function _pollStatus(sweepBtn, stopBtn) {
    if (!_sweepJobId) return;
    try {
      const resp = await fetch(`/api/status/${_sweepJobId}`);
      if (!resp.ok) {
        if (resp.status === 404) {
          clearInterval(_pollTimer);
          _pollTimer = null;
          _sweepJobId = null;
          _showProgress(false);
          _showError('Tác vụ đã hết hạn hoặc server vừa khởi động lại. Vui lòng chạy lại phân tích.');
          sweepBtn.disabled = false;
          stopBtn.disabled = true;
        }
        return;
      }
      const data = await resp.json();

      _updateProgressUI(data.progress, data.status_text);

      if (data.status === 'done') {
        clearInterval(_pollTimer);
        _pollTimer = null;
        _sweepJobId = null;
        _updateProgressUI(1.0, 'Hoàn thành! ✅');
        _renderSweepResults(data);
        sweepBtn.disabled = false;
        stopBtn.disabled = true;
      } else if (data.status === 'cancelled') {
        clearInterval(_pollTimer);
        _pollTimer = null;
        _sweepJobId = null;
        _showProgress(false);
        _showError('Đã dừng tính toán theo yêu cầu.');
        sweepBtn.disabled = false;
        stopBtn.disabled = true;
      } else if (data.status === 'error') {
        clearInterval(_pollTimer);
        _pollTimer = null;
        _sweepJobId = null;
        _showProgress(false);
        _showError('Lỗi tính toán: ' + (data.error_msg || 'Unknown error'));
        sweepBtn.disabled = false;
        stopBtn.disabled = true;
      }
    } catch (err) {
      // Network hiccup – keep polling
    }
  }

  function _showProgress(show) {
    document.getElementById('progress-idle').classList.toggle('hidden', show);
    document.getElementById('progress-bar-wrap').classList.toggle('hidden', !show);
  }

  function _updateProgressUI(progress, text) {
    const pct = Math.round(progress * 100);
    document.getElementById('progress-bar-fill').style.width = pct + '%';
    document.getElementById('progress-label').textContent = pct + '%';
    document.getElementById('progress-status').textContent = text;
  }

  function _renderSweepResults(data) {
    const setImg = (id, b64) => {
      if (!b64) return;
      const el = document.getElementById(id);
      if (el) el.src = 'data:image/png;base64,' + b64;
    };
    setImg('img-disp',  data.img_disp);
    setImg('img-acc',   data.img_acc);
    setImg('img-worst', data.img_worst);
    setImg('img-freq',  data.img_freq);
    document.getElementById('results-dashboard').classList.remove('hidden');
  }

  function _showError(msg) {
    const box = document.getElementById('sweep-error');
    if (box) {
      box.textContent = '⚠ ' + msg;
      box.classList.remove('hidden');
    }
  }

  return { 
    init,
    goToTab,
    runSweep,
    stopSweep,
    toggleProfileInputs
  };
})();

// Bootstrap
document.addEventListener('DOMContentLoaded', App.init);
