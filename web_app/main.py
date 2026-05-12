"""
main.py – FastAPI entry point for the HSLM Dynamic Analysis Web Application.

Run with:
    cd Python_Version/web_app
    uvicorn main:app --reload --host 127.0.0.1 --port 8000
"""
from __future__ import annotations

import traceback
import threading
import queue
import multiprocessing as mp
import uuid
from pathlib import Path
from typing import Dict

from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

from web_app.schemas import (
    VibrationCheckRequest,
    VibrationCheckResponse,
    ModeResult,
    ModeDisplayRow,
    DynamicSimRequest,
    DynamicSimResponse,
    SweepRequest,
    SweepJobResponse,
    JobStatusResponse,
)
from web_app.core_worker import check_free_vibration, run_single_simulation, run_dynamic_sweep

# ---------------------------------------------------------------------------
# App setup
# ---------------------------------------------------------------------------
app = FastAPI(title="HSLM Dynamic Analysis", version="2.0.0")

STATIC_DIR = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

# Heavy multi-train sweep is disabled on cloud to avoid overload.
# The new single-run simulation endpoint is always enabled.
DYNAMIC_SWEEP_ENABLED = False

# ---------------------------------------------------------------------------
# In-memory job store (legacy sweep – kept for local use)
# ---------------------------------------------------------------------------
_jobs: Dict[str, dict] = {}


def _sweep_worker_entry(params: dict, progress_queue):
    try:
        def _progress(prog, text):
            progress_queue.put({"type": "progress", "progress": float(prog), "status_text": str(text)})
        output = run_dynamic_sweep(params, update_cb=_progress)
        progress_queue.put({
            "type": "done",
            "img_disp": output["img_disp"], "img_acc": output["img_acc"],
            "img_worst": output["img_worst"], "img_freq": output["img_freq"],
        })
    except Exception:
        progress_queue.put({"type": "error", "error_msg": traceback.format_exc()})


def _monitor_sweep_process(job_id: str):
    job = _jobs.get(job_id)
    if job is None:
        return
    proc = job.get("_process")
    progress_queue = job.get("_queue")
    if proc is None or progress_queue is None:
        return

    job["status"] = "running"
    job["status_text"] = "Đang khởi động tính toán..."

    while True:
        if job.get("status") == "cancelled":
            break
        try:
            msg = progress_queue.get(timeout=0.5)
        except queue.Empty:
            if not proc.is_alive():
                if job.get("status") in {"queued", "running"}:
                    job["status"] = "error"
                    job["status_text"] = "Tiến trình tính toán dừng bất thường."
                    job["error_msg"] = f"Worker exited with code {proc.exitcode}"
                break
            continue

        mtype = msg.get("type")
        if mtype == "progress":
            if job.get("status") != "cancelled":
                job["status"] = "running"
                job["progress"] = float(msg.get("progress", 0.0))
                job["status_text"] = msg.get("status_text", "Đang tính toán...")
        elif mtype == "done":
            job.update({"status": "done", "progress": 1.0, "status_text": "Hoàn thành!",
                        "img_disp": msg.get("img_disp"), "img_acc": msg.get("img_acc"),
                        "img_worst": msg.get("img_worst"), "img_freq": msg.get("img_freq")})
            break
        elif mtype == "error":
            job.update({"status": "error", "status_text": "Lỗi xảy ra trong quá trình tính toán.",
                        "error_msg": msg.get("error_msg", "Unknown worker error")})
            break

    if proc.is_alive():
        proc.join(timeout=1.0)


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------

@app.get("/", response_class=FileResponse)
async def index():
    return FileResponse(str(STATIC_DIR / "index.html"))


@app.post("/api/check-vibration", response_model=VibrationCheckResponse)
async def api_check_vibration(body: VibrationCheckRequest):
    """Synchronous – fast enough to run in-request (< 1 ms)."""
    bridge_p = body.bridge.model_dump()
    result = check_free_vibration(bridge_p, n_modes=body.n_modes)
    return VibrationCheckResponse(
        vertical_modes=[ModeResult(**m) for m in result["vertical_modes"]],
        lateral_modes=[ModeResult(**m) for m in result["lateral_modes"]],
        torsion_modes=[ModeResult(**m) for m in result["torsion_modes"]],
        mode_rows=[ModeDisplayRow(**m) for m in result["mode_rows"]],
        modes=[ModeResult(**m) for m in result["modes"]],
        verdict=result["verdict"],
        verdict_text=result["verdict_text"],
        governing_f1_hz=result["governing_f1_hz"],
        img_vertical_modes=result["img_vertical_modes"],
        img_lateral_modes=result["img_lateral_modes"],
        img_torsion_modes=result["img_torsion_modes"],
        f1_hz=result["f1_hz"],
    )


@app.post("/api/run-dynamic", response_model=DynamicSimResponse)
async def api_run_dynamic(body: DynamicSimRequest):
    """
    Synchronous single-train Time-History simulation.
    Runs in a background thread to avoid blocking the async event loop.
    """
    import asyncio

    params = {
        "bridge_p":    body.bridge.model_dump(),
        "track_type":  body.bridge.track_type,
        "train_name":  body.train_name,
        "num_coaches": body.num_coaches,
        "vel_kmh":     body.vel_kmh,
        "fast_mode":   True,
    }

    loop = asyncio.get_event_loop()
    try:
        # run_in_executor with None = default ThreadPoolExecutor
        result = await loop.run_in_executor(None, run_single_simulation, params)
    except Exception:
        raise HTTPException(status_code=500, detail=traceback.format_exc())

    return DynamicSimResponse(
        img_disp_time=result["img_disp_time"],
        img_acc_time=result["img_acc_time"],
        max_disp_mm=result["max_disp_mm"],
        max_acc_ms2=result["max_acc_ms2"],
        duration_s=result["duration_s"],
        status_text=result["status_text"],
    )


@app.post("/api/run-sweep", response_model=SweepJobResponse)
async def api_run_sweep(body: SweepRequest):
    """Legacy heavy sweep – disabled on cloud deployment."""
    if not DYNAMIC_SWEEP_ENABLED:
        raise HTTPException(status_code=503, detail="Dynamic sweep is disabled on this server.")

    job_id = str(uuid.uuid4())
    _jobs[job_id] = {
        "status": "queued", "progress": 0.0, "status_text": "Đang xếp hàng...",
        "img_disp": None, "img_acc": None, "img_worst": None, "img_freq": None,
        "error_msg": None, "_process": None, "_queue": None,
    }
    params = {
        "bridge_p": body.bridge.model_dump(), "track_type": body.bridge.track_type,
        "train_names": body.train_names, "num_coaches": body.num_coaches,
        "v_min_kmh": body.v_min_kmh, "v_max_kmh": body.v_max_kmh, "v_step_kmh": body.v_step_kmh,
        "max_workers": min(20, max(1, (mp.cpu_count() or 2) - 1)),
    }
    ctx = mp.get_context("spawn")
    pq = ctx.Queue()
    proc = ctx.Process(target=_sweep_worker_entry, args=(params, pq))
    proc.start()
    _jobs[job_id]["_process"] = proc
    _jobs[job_id]["_queue"] = pq
    threading.Thread(target=_monitor_sweep_process, args=(job_id,), daemon=True).start()
    return SweepJobResponse(job_id=job_id)


@app.post("/api/stop-dynamic/{job_id}")
async def api_stop_dynamic(job_id: str):
    if not DYNAMIC_SWEEP_ENABLED:
        raise HTTPException(status_code=503, detail="Sweep is disabled.")
    job = _jobs.get(job_id)
    if job is None:
        raise HTTPException(status_code=404, detail="Job not found")
    if job["status"] in {"done", "error", "cancelled"}:
        return {"job_id": job_id, "status": job["status"]}
    proc = job.get("_process")
    if proc and proc.is_alive():
        proc.terminate()
        proc.join(timeout=2.0)
    job["status"] = "cancelled"
    job["status_text"] = "Đã dừng tính toán theo yêu cầu người dùng."
    return {"job_id": job_id, "status": "cancelled"}


@app.get("/api/status/{job_id}", response_model=JobStatusResponse)
async def api_job_status(job_id: str):
    job = _jobs.get(job_id)
    if job is None:
        raise HTTPException(status_code=404, detail="Job not found")
    return JobStatusResponse(
        job_id=job_id, status=job["status"], progress=job["progress"],
        status_text=job["status_text"], img_disp=job.get("img_disp"),
        img_acc=job.get("img_acc"), img_worst=job.get("img_worst"),
        img_freq=job.get("img_freq"), error_msg=job.get("error_msg"),
    )
