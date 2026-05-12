# TECHNICAL SPECIFICATION & AGENT HANDOFF PLAN
**Project:** Hybrid Web-Local Application for HSLM Dynamic Analysis
**Architecture:** FastAPI (Backend) + Vanilla HTML/CSS/JS (Frontend - Premium Glassmorphism UI)
**Core Engine:** `A00_Run_HSLM_Sweep.py`

Chào Agent phụ trách thi công, đây là bản thiết kế hệ thống chi tiết đã duyệt bởi Sếp. Bạn hãy đọc thật kỹ và triển khai code bám sát theo từng Step và Specification dưới đây. Đặc biệt phần **CHI TIẾT MÀN HÌNH** là kim chỉ nam để bạn code HTML/CSS.

---

## 1. UX/UI DESIGN SPECIFICATION (BẢN VẼ CHI TIẾT 3 MÀN HÌNH)

Toàn bộ ứng dụng sẽ nằm trong 1 trang duy nhất, sử dụng hệ thống **Tabs (hoặc Sidebar menu)** để chuyển qua lại giữa 3 Window.

### 🪟 WINDOW 1: CORE INPUTS (Cấu hình Mô hình)
*Nhiệm vụ: Form nhập liệu thông số Cầu và cấu hình hệ thống.*

**Chia làm 2 nhóm thẻ Card (Glassmorphism layout):**

+ **Card 1: Bridge & Track Parameters (Thông số cầu đường)**
  - `Dropdown (List Box)`: Kiểu cấu trúc đường (Track Type) - *Tùy chọn: "Có Ballast" (With Ballast) hoặc "Không Ballast"*. Sẽ tự động map vào các hàm cấp phát thông số như `TrackProp_Zhai_WithBallastOnBridge` ở lõi Python.
  - `Trường nhập [Number]`: Nhịp L (m) - *Mặc định: 50*
  - `Trường nhập [Number]`: Mô đun đàn hồi E (N/m2) - *Mặc định: 3.5e10*
  - `Trường nhập [Number]`: Mô men quán tính I (m4) - *Mặc định: 51.3*
  - `Trường nhập [Number]`: Khối lượng phân bố $\rho$ (kg/m) - *Mặc định: 69000*
  - `Trường nhập [Number]`: Tỉ lệ cản Damping (%) - *Mặc định: 2.0*
  - `Trường nhập [Number]`: Lưới phần tử (Elements per spacing) - *Mặc định: 2*
+ **Card 2: Train Configuration (Cấu hình đoàn tàu)**
  - `Danh sách Checkbox`: Danh sách các loại tàu HSLM A1 đến HSLM A10. Có nút [Tích chọn tất cả].
  - `Trường nhập [Number]`: Số lượng toa xe (Number of coaches) - *Mặc định: Để trống (theo tiêu chuẩn)*

---

### 🪟 WINDOW 2: FREE VIBRATION CHECK (Kiểm tra Phân tích động)
*Nhiệm vụ: Chạy tính Mode dao động trước xem có phải tốn công chạy động lực học ở Window 3 hay không.*

- **Vùng thực thi:** 
  - Nút bấm lớn gradient: **[Run Eigenvalue Analysis]**
- **Vùng Hiển thị Kết quả:**
  - `Bảng (Table)`: Cột 1 = "Mode 1, Mode 2, Mode 3" | Cột 2 = "Tần số tự nhiên Hz".
  - `Khung Phán Quyết (Status Box)`: Nhận xét dao động theo tiêu chuẩn (Eurocode EN 1991-2).
    - Nếu Tần số Mode 1 nằm trong giới hạn phải chạy: Khung bo góc **màu Đỏ cam (Warning)**, icon 🚨, Text: *"Kết cấu yêu cầu phân tích động lực học!"*
    - Nếu Tần số nằm ngoài vùng nguy hiểm: Khung bo góc **màu Xanh lá (Success)**, icon ✅, Text: *"Kết cấu thỏa mãn an toàn cộng hưởng. Không bắt buộc phân tích động."*

---

### 🪟 WINDOW 3: DYNAMIC SWEEP & PLOTTING (Phân tích Động lực)
*Nhiệm vụ: Cấu hình dải vận tốc, chạy mô phỏng siêu nặng, và show kết quả Biểu diễn.*

- **Khung Input (Dải vận tốc tính toán):**
  - `Trường nhập [Number]`: V_min (km/h) - *Mặc định: 250*
  - `Trường nhập [Number]`: V_max (km/h) - *Mặc định: 350*
  - `Trường nhập [Number]`: Bước quét V_step - *Mặc định: 10*
- **Vùng Thực thi & Tiến độ:**
  - Nút Action: **[Run Dynamic Sweep Analysis]**. Hover sáng lóe.
  - Sau khi bấm, hiện thanh **Progress Bar** chạy dài cùng dòng note trạng thái *"Đang xử lý tổ hợp Tàu x Gió..."* (Fetch polling Status API).
- **Vùng Báo cáo Kết quả (Dashboard Layout):**
  - Khung Canvas chứa Grid hiển thị 4 bức ảnh Base64 trả về:
    1. Biểu đồ Displacement (Võng).
    2. Biểu đồ Acceleration (Gia tốc dầm cầu).
    3. Critical Train Plot (Tàu nguy hiểm nhất ở từng vận tốc).
    4. Frequency check plot.

---

## 2. CẤU TRÚC THƯ MỤC CẦN TẠO
Hãy set up code theo cây thư mục sau:

```text
Python_Version/
├── web_app/
│   ├── main.py
│   ├── schemas.py             
│   ├── core_worker.py         
│   └── static/                
│       ├── index.html         
│       ├── css/
│       │   └── style.css      
│       └── js/
│           └── app.js         
```

---

## 3. CÁC BƯỚC THỰC THI CHO AGENT BACKEND & FRONTEND

### Step 1: Refactor Logic (`core_worker.py`)
- Mở `A00_Run_HSLM_Sweep.py`, bê hàm xử lý sang `core_worker.py`. 
- Viết hàm `check_free_vibration(bridge_props, track_type) -> dict`. (Logic kiểm tra EN 1991-2 tạo rule sơ bộ).
- Viết hàm `run_dynamic_sweep_task(...) -> dict`. Đảm bảo khởi tạo chuẩn property `TrackProp_Zhai_WithBallastOnBridge` nếu User chọn Track có Ballast. Ảnh biểu đồ thay vì `plt.savefig()` ra HDD thì chuyển sang `io.BytesIO() -> base64`.

### Step 2: Code Backend (`main.py`)
- Làm API `POST /api/check-vibration`.
- Làm API `POST /api/run-dynamic` với cơ chế BackgroundTask hoặc Thread và trả output về client.

### Step 3: Dựng UI/UX Cực Phẩm (`static`)
- CSS: Bắt sáng Glassmorphism (nền blur gradient tối màu, viền neon mảnh, nút ấn glow). Font "Inter".
- JS: Thu thập Form Data (bao gồm cả Dropdown list chọn cấu trúc đường) > Fetch call > Render ảnh imgBase64 cực căng nét vào `Window 3`.

>> THI CÔNG NGAY THEO BẢN THIẾT KẾ NÀY! KHÔNG HỎI LẠI! MỤC TIÊU DUY NHẤT: TRẢI NGHIỆM VÀ ĐỘ CHÍNH XÁC KHI SẾP SỬ DỤNG.
$env:PYTHONPATH = "d:\Tools\DynamicAnalysis\SOFTX-D-22-00221\Python_Version"; python -m uvicorn web_app.main:app --host 0.0.0.0 --port 8002