# Tài liệu Lý thuyết: Phân tích Động lực học Hệ Tàu - Đường - Cầu (TTB-2D)

## 1. Giới thiệu chung

Ứng dụng của chúng ta được xây dựng dựa trên cốt lõi là hệ thống **TTB-2D (Train-Track-Bridge in 2D)**. Đây là một công cụ mạnh mẽ dùng để mô phỏng và phân tích tương tác động lực học phức tạp giữa các thành phần: Phương tiện (Tàu), Đường ray, Lớp đá Ballast và Kết cấu Cầu.

Hiểu một cách đơn giản, khi một đoàn tàu di chuyển qua cầu, nó không chỉ tạo ra tĩnh tải mà còn gây ra các dao động động lực học. TTB-2D giúp chúng ta tính toán chính xác phản ứng của cầu, cũng như sự tương tác qua lại giữa bánh tàu và đường ray.

![Tổng quan mô hình TTB-2D](./docs/images/fig_1_ttb2d_overview.png)
*Hình 1: Tổng quan về mô hình TTB-2D*

Mô hình xem xét một đoàn tàu (gồm nhiều toa xe) di chuyển trên đường ray từ trái sang phải với một vận tốc không đổi. Hệ thống đường ray bao gồm các thành phần từ trên xuống dưới: ray, đệm cao su (pad), tà vẹt (sleeper), đá ballast và lớp dưới ballast (sub-ballast). Một phần của đường ray được đặt trên cầu (mô phỏng như một dầm).

---

## 2. Mô tả các mô hình thành phần

Để đạt được sự cân bằng giữa độ chính xác và khối lượng tính toán, bài toán được chia thành các hệ thống con.

### 2.1. Mô hình Phương tiện (Vehicle Model)

Đoàn tàu được định nghĩa là một chuỗi các toa xe (vehicle) ghép nối với nhau. Mỗi toa xe được mô phỏng bằng một mô hình tập trung khối lượng (lumped masses), kết hợp với các thanh cứng, lò xo và bộ cản dịu (dashpots).

![Mô hình Phương tiện](./docs/images/fig_2_vehicle_model.png)
*Hình 2: Chi tiết mô hình toán học của một toa xe*

Các thành phần chính:
- **Bánh xe (Wheels):** Được xem như các khối lượng tập trung trượt trên ray.
- **Hệ thống treo sơ cấp (Primary suspension):** Liên kết giữa bánh xe và giá chuyển hướng (bogie), bao gồm lò xo và bộ cản dịu (đặc trưng bởi độ cứng $k_p$ và hệ số cản $c_p$).
- **Hệ thống treo thứ cấp (Secondary suspension):** Liên kết giữa giá chuyển hướng và thân toa xe chính ($k_s, c_s$).
- **Thân xe và Giá chuyển hướng:** Được mô phỏng là các thanh cứng có khối lượng ($m$) và mô men quán tính ($I$).

### 2.2. Mô hình Kết cấu Cầu (Bridge Model)

Cầu được mô phỏng như một dầm Euler-Bernoulli sử dụng phương pháp Phần tử hữu hạn (FEM). 

![Mô hình Cầu](./docs/images/fig_3_bridge_model.png)
*Hình 3: Mô hình dầm cầu Euler-Bernoulli*

Các thông số cơ bản bao gồm:
- **$L$:** Nhịp cầu
- **$E, I$:** Độ cứng uốn của mặt cắt dầm
- **$\mu$:** Khối lượng trên một đơn vị chiều dài
- **$\eta$:** Tỷ số cản (Damping ratio)

Ngoài ra, ứng dụng cho phép linh hoạt thiết lập các điều kiện biên. Cầu có thể là dầm giản đơn, dầm liên tục hoặc có các gối tựa đàn hồi.

![Các gối tựa của dầm](./docs/images/fig_4_beam_supports.png)
*Hình 4: Dầm với nhiều gối tựa. Mỗi gối tựa có thể thiết lập độ cứng dọc trục ($k_V$) và độ cứng xoay ($k_R$)*

### 2.3. Mô hình Đường ray (Track Model)

Đường ray được mô phỏng là một dầm liên tục đặt trên các hệ thống khối lượng - lò xo phân bố tuần hoàn. Dầm đại diện cho ray, còn các khối lượng đại diện cho tà vẹt và lớp đá ballast.

![Mô hình Đường ray](./docs/images/fig_5_track_model.png)
*Hình 5: Mô hình chi tiết của hệ thống đường ray*

Hệ thống bao gồm các lớp liên kết:
- **Ray:** Mô phỏng bằng phần tử dầm (thông số $E_R, I_R, \mu_R$).
- **Đệm (Pads):** Lò xo và cản dịu thẳng đứng ($k_P, c_P$).
- **Tà vẹt (Sleepers):** Các khối lượng tập trung ($m_S$) đặt cách nhau một khoảng $L_S$.
- **Lớp đá Ballast & Sub-ballast:** Cung cấp độ cứng ($k_{BA}, k_{SB}$) và sự cản dịu từ nền đường.

### 2.4. Liên kết Đường ray trên Cầu (Track on Bridge)

Khi đường ray chạy ngang qua cầu, mô hình là sự kết hợp giữa mô hình Cầu và mô hình Đường ray. Ứng dụng hỗ trợ hai trường hợp thực tế:

**Trường hợp 1: Có lớp đá Ballast trên cầu**  
Khối lượng đại diện cho ballast sẽ nằm trực tiếp lên trên dầm cầu. Bề dày của lớp ballast trên cầu thường khác với trên nền đất cứng, do đó các thông số cơ học ($k^*_{BA}, c^*_{BA}$) cũng sẽ được hiệu chỉnh cho phù hợp.

![Đường ray trên cầu có ballast](./docs/images/fig_6_track_on_bridge_ballast.png)
*Hình 6: Mô hình đường ray trên cầu (có ballast)*

**Trường hợp 2: Không có đá Ballast (Bản mặt cầu chạy trực tiếp / Chân đế cố định)**  
Trong trường hợp này, tà vẹt được đặt trực tiếp lên dầm cầu thông qua một lớp đệm lót dưới tà vẹt (Pad Under sleeper - PU) với độ cứng $k_{PU}$ và cản dịu $c_{PU}$.

![Đường ray trên cầu không có ballast](./docs/images/fig_7_track_on_bridge_no_ballast.png)
*Hình 7: Mô hình đường ray trên cầu (không có ballast)*

---

## 3. Phương trình Chủ đạo và Nguyên lý Giải số

### 3.1. Phương trình Vi phân Tổng quát

Phương trình vi phân chuyển động của một hệ cơ học có nhiều bậc tự do (MDOF) biểu diễn sự tương tác động lực học được viết dưới dạng ma trận như sau:

$$[M]\{\ddot{x}\} + [C]\{\dot{x}\} + [K]\{x\} = \{F(t)\}$$

Trong đó:
- $[M]$: Ma trận khối lượng của hệ thống (Mass matrix).
- $[C]$: Ma trận cản (Damping matrix), đặc trưng cho sự tiêu tán năng lượng.
- $[K]$: Ma trận độ cứng (Stiffness matrix).
- $\{x\}, \{\dot{x}\}, \{\ddot{x}\}$: Lần lượt là véc-tơ chuyển vị, vận tốc và gia tốc tại các bậc tự do của hệ thống.
- $\{F(t)\}$: Véc-tơ tải trọng tác dụng phụ thuộc vào thời gian $t$.

### 3.2. Tương tác Tàu - Hạ tầng (Vehicle - Infrastructure Interaction)

Khi tàu chạy trên đường ray hoặc cầu, hệ thống tổng thể được chia thành hai hệ thống con (Subsystems) tương tác qua lại lẫn nhau thông qua **lực tiếp xúc (Contact forces - $F_c$)** tại vị trí bánh xe. Phương trình chuyển động của từng hệ thống con được định nghĩa:

**Đối với hệ thống Phương tiện (Tàu):**
$$[M_v]\{\ddot{x}_v\} + [C_v]\{\dot{x}_v\} + [K_v]\{x_v\} = \{F_{ext,v}\} - \{F_c\}$$

**Đối với hệ thống Hạ tầng (Đường ray / Cầu):**
$$[M_b]\{\ddot{x}_b\} + [C_b]\{\dot{x}_b\} + [K_b]\{x_b\} = \{F_{ext,b}\} + \{F_c\}$$

Trong đó:
- Chỉ số $v$ (vehicle) và $b$ (bridge/track) đại diện cho các ma trận của tàu và kết cấu bên dưới.
- Lực tương tác $\{F_c\}$ không phải là hằng số. Nó phụ thuộc trực tiếp vào khối lượng không treo của bánh tàu, độ cứng điểm tiếp xúc, chuyển vị tương đối giữa bánh tàu và mặt ray, yếu tố hình học của biên dạng ray (profile irregularities), và vận tốc tàu $V$. 

Sự di chuyển của lực tiếp xúc $\{F_c\}$ dọc theo chiều dài cầu chính là nguyên nhân làm cho bài toán trở nên **phi tuyến (non-linear)** và **phụ thuộc thời gian (time-dependent)**, mặc dù bản thân vật liệu và kết cấu của từng hệ thống con được giả định là tuyến tính.

### 3.3. Giải pháp Tích phân Số (Numerical Solution)

Để tính toán sự kết hợp (coupled system) của hai phương trình trên, TTB-2D gộp chúng lại thành một hệ phương trình ma trận đồ sộ:

$$ \begin{bmatrix} M_v & 0 \\ 0 & M_b \end{bmatrix} \begin{Bmatrix} \ddot{x}_v \\ \ddot{x}_b \end{Bmatrix} + \begin{bmatrix} C_v & C_{v,b} \\ C_{b,v} & C_b \end{bmatrix} \begin{Bmatrix} \dot{x}_v \\ \dot{x}_b \end{Bmatrix} + \begin{bmatrix} K_v & K_{v,b} \\ K_{b,v} & K_b \end{bmatrix} \begin{Bmatrix} x_v \\ x_b \end{Bmatrix} = \begin{Bmatrix} F_v \\ F_b \end{Bmatrix} $$

Vì ma trận cản và ma trận độ cứng tương tác ($C_{v,b}, C_{b,v}, K_{v,b}, K_{b,v}$) thay đổi liên tục theo vị trí của tàu, cốt lõi giải thuật của ứng dụng sẽ:
1. **Cập nhật ma trận hệ thống** ở mỗi bước thời gian nhỏ (time step $\Delta t$).
2. **Tích phân trực tiếp** các phương trình chuyển động bằng thuật toán **Newmark-$\beta$** (một phương pháp giải số vô điều kiện ổn định thường dùng trong động lực học kết cấu).

Kết quả xuất ra sẽ bao gồm chuyển vị tại các nút, lực tiếp xúc (contact forces), biến dạng, biểu đồ mô men uốn, lực cắt và gia tốc. Các số liệu này đóng vai trò quan trọng giúp kỹ sư đánh giá đầy đủ an toàn của cấu trúc cầu dưới tác dụng của tải trọng đoàn tàu cao tốc.

---

## 4. Hướng dẫn sử dụng Phần mềm (Web App)

Trang phân tích động lực học trực tuyến cung cấp một giao diện trực quan và thân thiện, được chia làm 3 bước (tab) tương ứng với quy trình tính toán động lực học tiêu chuẩn.

### 4.1. Core Inputs - Cấu hình Mô hình

Đây là bước thiết lập các thông số cơ bản cho Cầu, Đường ray và Tàu.

![Core Inputs](./docs/images/app_tab_1_inputs.png)
*Hình 8: Giao diện thiết lập thông số mô hình (Core Inputs)*

**Thông số Cầu (Bridge Parameters):**
- **Loại dầm điển hình:** Cho phép chọn các mẫu dầm có sẵn (như dầm hộp HSR, dầm thép) giúp điền tự động các giá trị.
- **Nhịp L (m) / Mô đun E (N/m²):** Chiều dài nhịp cầu và mô đun đàn hồi của vật liệu.
- **Ixx, Iyy, Ixy, G, J, v.v...:** Các đặc trưng hình học và xoắn của mặt cắt ngang dầm. 
- **Khối lượng $\rho$ (kg/m):** Khối lượng phân bố đều trên một mét dài của cầu.
- **Tỉ lệ cản (%):** Tỷ lệ cản (Damping ratio), thường chọn từ 1% đến 2% cho cầu bê tông cốt thép.

**Thông số Đường ray (Track Parameters):**
- **Kiểu đường (Track Type):** Chọn loại đường có đá ballast hoặc không có đá ballast.
- **Loại Profile:** Lựa chọn biên dạng bề mặt ray (phẳng, gờ nhân tạo, hoặc theo phổ ngẫu nhiên PSD).
- **Tiêu chuẩn phổ PSD:** (Chỉ áp dụng khi chọn phổ PSD) Chọn tiêu chuẩn đường sắt tương ứng (Chinese HSR, Eurocode, FRA, v.v...).

**Thông số Tàu (Train Configuration):**
- Chọn các mô hình tàu HSLM-A tiêu chuẩn theo Eurocode EN 1991-2. Bạn có thể chọn một hoặc nhiều cấu hình tàu để tính toán song song.

### 4.2. Free Vibration - Kiểm tra Phân tích động

Sau khi thiết lập cấu hình, hệ thống sẽ tính toán các Tần số tự nhiên (Natural Frequencies) của cầu.

![Free Vibration](./docs/images/app_tab_2_vibration.png)
*Hình 9: Giao diện kiểm tra tần số dao động (Free Vibration)*

**Ý nghĩa các thao tác:**
- Nút **Run Eigenvalue Analysis:** Nhấn để hệ thống giải bài toán trị riêng (Eigenvalue) và tìm ra các tần số dao động riêng.
- **Bảng kết quả:** Hiển thị các mode dao động tương ứng với tần số $f$ (Hz). Hệ thống sẽ tự động so sánh tần số cơ bản ($n_0$) với các giới hạn trong tiêu chuẩn EN 1991-2 để đưa ra kết luận (Verdict) xem cây cầu này **có bắt buộc** phải phân tích động lực học (Dynamic Analysis) hay không.

### 4.3. Dynamic Sweep - Phân tích Động lực học

Đây là bước chạy mô phỏng cốt lõi. Hệ thống sẽ cho các tàu chạy qua cầu với nhiều vận tốc khác nhau (quét vận tốc - Speed Sweep) để tìm ra vận tốc cộng hưởng nguy hiểm nhất.

![Dynamic Sweep](./docs/images/app_tab_3_sweep.png)
*Hình 10: Giao diện quét vận tốc và vẽ biểu đồ (Dynamic Sweep)*

**Các lựa chọn tính toán:**
- **V_min / V_max (km/h):** Giới hạn dải vận tốc tàu muốn quét. Ví dụ từ 250 km/h đến 350 km/h.
- **Bước V_step (km/h):** Khoảng cách giữa mỗi lần chạy mô phỏng. Chọn càng nhỏ thì biểu đồ càng mịn nhưng thời gian tính toán sẽ lâu hơn (thông thường chọn 10 km/h).

**Biểu đồ kết quả (Results Dashboard):**
Sau khi hoàn tất, hệ thống sẽ xuất ra 4 biểu đồ quan trọng:
1. **Võng giữa nhịp (Displacement):** Độ võng lớn nhất của cầu tại từng vận tốc mô phỏng.
2. **Gia tốc dầm (Acceleration):** Gia tốc lớn nhất của mặt cầu tại từng vận tốc (tiêu chí cực kỳ quan trọng để đánh giá độ an toàn chạy tàu).
3. **Tàu nguy hiểm nhất (Critical Train):** So sánh hiệu ứng động lực học lớn nhất giữa các mác tàu HSLM-A khác nhau để tìm ra đoàn tàu bất lợi nhất.
4. **Kiểm tra tần số (Frequency Check):** Biểu đồ thể hiện mức độ nguy hiểm của các hiện tượng cộng hưởng (Resonance) dựa trên chuỗi tần số quét.
