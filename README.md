# ROS 2 Web Bridge Monitoring System

*Capstone Project - Integrated Real-time IoT System*

Repositori ini adalah modul penghubung utama (*core bridge*) untuk sistem monitoring terintegrasi. Proyek ini dibangun berdasarkan kolaborasi lintas divisi teknis untuk menciptakan pipeline data yang mengalir dari sensor fisik hingga ke antarmuka pengguna berbasis web.

Sistem ini menerapkan komunikasi **Linear Stream (Simplex)**, di mana data mengalir satu arah secara real-time dari perangkat keras ke dashboard tanpa hambatan *feedback loop* yang kompleks.

---

## 👥 Struktur Tim & Tanggung Jawab

Proyek ini dikembangkan oleh 6 anggota tim dengan spesialisasi masing-masing:

| Nama Anggota | Divisi | Peran Utama |
| :--- | :--- | :--- |
| **Dimas Humam** | Sistem Cerdas (AI) | Analisis prediktif & pemrosesan model cerdas. |
| **Favian Qintara** | IoT | Perancangan hardware & akuisisi data sensor. |
| **Muhammad Ervanzha** | IoT | Implementasi protokol komunikasi & kalibrasi. |
| **Alfareza Giovani** | Rekayasa Data | Pengembangan *Bridge Node* & pipeline data ROS 2. |
| **Delviano Arie P** | Rekayasa Data | Manajemen struktur data & optimasi aliran pesan. |
| **Albihan** | Rekayasa Perangkat Lunak (RPL) | Pengembangan Web Dashboard & Integrasi Sistem. |

---

## 🔄 Alur Kerja Sistem (Single Stream Architecture)

Sistem dirancang dengan pendekatan modular yang berjalan di perangkat terpisah namun saling terhubung melalui jaringan lokal. Alur data berjalan secara **Simplex** (Satu Arah) sebagai berikut:

1.  **Tahap Akuisisi (Tim IoT - Favian & Ervanzha)**
    * Sensor membaca parameter fisik (Suhu, Kelembapan, Cahaya).
    * Data dikirim via Micro-ROS menuju Gateway.

2.  **Tahap Bridging & Konversi (Tim Data - Alfareza & Delviano)**
    * Menerima raw data dari IoT.
    * Node `bridge_data` mengonversi data menjadi format JSON String standar.
    * Data dipublikasikan ke topik ROS 2 (`/suhu`, `/kelembapan`, `/ldr`).

3.  **Tahap Pemrosesan Cerdas (Tim AI - Dimas)**
    * (Opsional/Parallel) Node AI berlangganan ke data untuk melakukan inferensi atau deteksi anomali.

4.  **Tahap Visualisasi (Tim RPL - Albihan)**
    * Web Dashboard menerima data JSON melalui WebSocket (Rosbridge).
    * Menampilkan status terkini ke pengguna akhir.

---

🚀 Instalasi & Penggunaan (Modul Data Bridge)

Instruksi ini untuk menjalankan *Node Bridge* yang dikelola oleh Tim Rekayasa Data.

### Prasyarat
* ROS 2 Jazzy
* Python 3.10+

### 1. Build Paket
Pastikan Anda berada di root workspace.
```bash
colcon build --packages-select bridge_data --symlink-install
source install/setup.bash

### 2. Menjalankan Sistem
Gunakan perintah berikut untuk mengaktifkan jembatan data:

Bash
# Menjalankan Node Konversi Data (Data Engineering)
./install/bridge_data/lib/bridge_data/iot_bridge
Output yang diharapkan: [INFO] [iot_data_publisher]: IoT Data Bridge Node started.

### 3. Integrasi Web
Untuk menghubungkan ke Dashboard (RPL), pastikan Rosbridge aktif:

Bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
📡 Interface Data (JSON Output)
Tim RPL (Dashboard) akan menerima data dalam format berikut:

Topic: /suhu, /kelembapan, /ldr

Format: std_msgs/String

Payload:

JSON
{
  "name": "suhu",
  "value": 29.5,
  "unit": "C",
  "timestamp": "2026-02-04T15:00:00"
}

### Langkah 3: Commit dan Push Perubahan
Setelah menyimpan file (Ctrl+O, Enter, Ctrl+X), kirim perubahan tersebut ke GitHub:

```bash
cd ~/ros2_web_monitoring/ros2_ws
git add README.md
git commit -m "Update: Add Team Members and Simplify Architecture Flow"
git push origin main
