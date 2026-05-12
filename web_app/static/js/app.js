/**
 * app.js – Frontend logic for HSLM Dynamic Analysis Web App (v2)
 * Window 3: Single Time-History Simulation (1 train × 1 velocity)
 */
'use strict';

const App = (() => {
  let _beamData = {};

  // ── Init ────────────────────────────────────────────────────────────────
  function init() {
    _bindTabs();
    _loadBeamPropCSV();
    _bindTrainSelect();

    document.getElementById('btn-run-vibration').addEventListener('click', runVibration);

    toggleProfileInputs();
  }

  // ── Profile UI Toggle ───────────────────────────────────────────────────
  function toggleProfileInputs() {
    const type = document.getElementById('profile_type').value;
    const psdDiv  = document.getElementById('psd-inputs');
    const bumpDiv = document.getElementById('bump-inputs');
    if (!psdDiv || !bumpDiv) return;
    psdDiv.classList.add('hidden');
    bumpDiv.classList.add('hidden');
    if (type === '1') psdDiv.classList.remove('hidden');
    else if (type === '3') bumpDiv.classList.remove('hidden');
  }

  // ── Beam Properties CSV ────────────────────────────────────────────────
  async function _loadBeamPropCSV() {
    try {
      const resp = await fetch('/static/beam_prop.csv');
      if (!resp.ok) return;
      const text = await resp.text();
      const lines = text.trim().split(/\r?\n/);
      if (lines.length < 2) return;
      const headers = lines[0].split(',').map(h => h.trim());
      const select  = document.getElementById('beam_type');
      if (!select) return;
      for (let i = 1; i < lines.length; i++) {
        const values = lines[i].split(',').map(v => v.trim());
        if (values.length < headers.length) continue;
        const row = {};
        headers.forEach((h, idx) => { row[h] = values[idx]; });
        _beamData[row.id] = row;
        const opt = document.createElement('option');
        opt.value = row.id;
        opt.textContent = row.name;
        select.appendChild(opt);
      }
    } catch (err) {
      console.error('App: Error loading beam_prop.csv:', err);
    }
  }

  function _applyBeamType() {
    const id = document.getElementById('beam_type').value;
    if (id === 'custom') return;
    const data = _beamData[id];
    if (!data) return;
    ['L','E','Ixx','Iyy','Ixy','G','J','I_theta','rho','I','damping_pct','ele_per_spacing'].forEach(f => {
      const el = document.getElementById(f);
      if (el && data[f] !== undefined) {
        el.value = data[f];
        el.classList.add('field-highlight');
        setTimeout(() => el.classList.remove('field-highlight'), 1000);
      }
    });
  }

  // ── Train select sync (Window 1 → Window 3 display) ────────────────────
  function _bindTrainSelect() {
    const sel = document.getElementById('train_name');
    if (!sel) return;
    sel.addEventListener('change', _syncTrainDisplay);
    const beamSel = document.getElementById('beam_type');
    if (beamSel) beamSel.addEventListener('change', _applyBeamType);
  }

  function _syncTrainDisplay() {
    const sel     = document.getElementById('train_name');
    const display = document.getElementById('sim-train-display');
    if (!sel || !display) return;
    const opt = sel.options[sel.selectedIndex];
    display.textContent = opt ? opt.textContent : sel.value;
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
    // Sync train display when navigating to tab-sweep
    if (tabId === 'tab-sweep') _syncTrainDisplay();
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

    try {
      const resp = await fetch('/api/check-vibration', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ bridge: _getBridgeParams(), n_modes: 3 }),
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
    for (const m of rows) {
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

  function onTrainChange(val) {
    const p = document.getElementById('custom-train-panel');
    if (!p) return;
    if (val === 'Custom') p.classList.remove('hidden');
    else p.classList.add('hidden');
    _syncTrainDisplay();
  }

  // ── Window 3: Time-History Simulation ────────────────────────────────────
  async function runSweep() {
    const btn      = document.getElementById('btn-run-sweep');
    const simLabel = document.getElementById('btn-sim-label');
    const errBox   = document.getElementById('sweep-error');

    btn.disabled = true;
    if (simLabel) simLabel.textContent = '⏳ Đang tính toán...';
    errBox.classList.add('hidden');
    document.getElementById('results-dashboard').classList.add('hidden');
    document.getElementById('sim-stats').classList.add('hidden');
    _showProgress(true);
    _updateProgressUI(0.3, 'Đang chạy mô phỏng Newmark-β...');

    const trainName  = document.getElementById('train_name').value;
    const velKmh     = parseFloat(document.getElementById('sim_vel').value) || 300;
    const numCoaches = parseInt(document.getElementById('num_coaches').value) || null;
    const fastMode   = true;

    const body = {
      bridge:      _getBridgeParams(),
      train_name:  trainName,
      num_coaches: numCoaches,
      vel_kmh:     velKmh,
      fast_mode:   fastMode,
    };

    if (trainName === 'Custom') {
      body.custom_train_params = {
        m_body:  parseFloat(document.getElementById('cust_m_body').value)  || 40000,
        L_body:  parseFloat(document.getElementById('cust_l_body').value)  || 15.0,
        m_bogie: parseFloat(document.getElementById('cust_m_bogie').value) || 3000,
        m_wheel: parseFloat(document.getElementById('cust_m_wheel').value) || 1500,
      };
    }

    try {
      const resp = await fetch('/api/run-dynamic', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });

      if (!resp.ok) {
        const detail = await resp.json().catch(() => ({}));
        throw new Error(detail.detail || `Server error ${resp.status}`);
      }

      const data = await resp.json();
      _updateProgressUI(1.0, data.status_text || 'Hoàn thành! ✅');
      _renderSimResults(data);
    } catch (err) {
      _showError('Lỗi mô phỏng: ' + err.message);
      _showProgress(false);
    } finally {
      btn.disabled = false;
      if (simLabel) simLabel.textContent = '▶ Run Time-History Simulation';
    }
  }

  function _renderSimResults(data) {
    // Update charts
    const setImg = (id, b64) => {
      if (!b64) return;
      const el = document.getElementById(id);
      if (el) el.src = 'data:image/png;base64,' + b64;
    };
    setImg('img-disp', data.img_disp_time);
    setImg('img-acc',  data.img_acc_time);

    // Update stats panel
    const statDisp = document.getElementById('stat-disp');
    const statAcc  = document.getElementById('stat-acc');
    const statVerdict = document.getElementById('stat-verdict');

    if (statDisp) statDisp.textContent = `${data.max_disp_mm.toFixed(2)} mm`;
    if (statAcc)  statAcc.textContent  = `${data.max_acc_ms2.toFixed(3)} m/s²`;

    // EN 1991-2 acceleration verdict
    if (statVerdict) {
      if (data.max_acc_ms2 > 3.5) {
        statVerdict.style.cssText = 'background:rgba(255,107,107,0.15);border:1px solid rgba(255,107,107,0.4);color:#ff6b6b;';
        statVerdict.textContent = `🚨 Gia tốc vượt giới hạn EN 1991-2 (3.5 m/s²)! Cần xem xét lại kết cấu.`;
      } else {
        statVerdict.style.cssText = 'background:rgba(0,255,120,0.1);border:1px solid rgba(0,255,120,0.3);color:#00ff78;';
        statVerdict.textContent = `✅ Gia tốc thỏa mãn EN 1991-2 (${data.max_acc_ms2.toFixed(3)} < 3.5 m/s²)`;
      }
    }

    document.getElementById('sim-stats').classList.remove('hidden');
    document.getElementById('results-dashboard').classList.remove('hidden');
    _showProgress(false);
  }

  function stopSweep() {
    // No-op for single simulation (not needed)
  }

  function _showProgress(show) {
    document.getElementById('progress-idle').classList.toggle('hidden', show);
    document.getElementById('progress-bar-wrap').classList.toggle('hidden', !show);
  }

  function _updateProgressUI(progress, text) {
    const pct = Math.round(progress * 100);
    document.getElementById('progress-bar-fill').style.width = pct + '%';
    document.getElementById('progress-label').textContent = text || (pct + '%');
    const statusEl = document.getElementById('progress-status');
    if (statusEl) statusEl.textContent = '';
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
    toggleProfileInputs,
    onTrainChange,
    _syncTrainDisplay,
  };
})();

// Bootstrap
document.addEventListener('DOMContentLoaded', App.init);
