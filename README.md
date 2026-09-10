# 5-DoF Robotic ARM For Lab Studies

Purwarupa lengan robot 5-Derajat Kebebasan (5-DoF) yang dikembangkan oleh **Kelompok Studi Robotika (KSR)**, Fakultas Teknologi Industri, Universitas Atma Jaya Yogyakarta (UAJY), sebagai instrumen pembelajaran dan praktikum untuk mata kuliah **Otomasi Industri dan Robotika**.

Proyek ini menjembatani teori kinematika ruang tiga dimensi dengan penerapan nyata pada perangkat keras fisik, melalui tiga tahap utama: **pemodelan**, **pembuatan purwarupa**, dan **pengujian pergerakan sendi**.

## Fitur Utama

- **Forward Kinematics (FK)** — melacak posisi koordinat ujung lengan (end-effector) secara presisi berdasarkan input sudut sendi, menggunakan parameter Denavit-Hartenberg (DH) hybrid yang diekstraksi langsung dari desain URDF.
- **Inverse Kinematics (IK)** — diselesaikan dengan algoritma analitik-numerik **Damped Least Squares (DLS)** untuk menstabilkan pergerakan di area singularitas mekanis, tanpa memerlukan pelatihan AI.
- **Simulasi Virtual (MATLAB)** — lingkungan simulasi berbasis URDF sebagai lapisan perlindungan sebelum program dieksekusi ke mesin fisik, termasuk simulasi Monte Carlo (30.000 kombinasi sudut sendi acak) untuk memetakan titik rawan tabrakan.
- **Interlock Keamanan** — sistem kendali Python memblokir eksekusi trajektori yang berpotensi menabrak bidang kerja (meja).

## Struktur Perangkat Keras

- Base link, tiga tautan ekstensi menengah, dan end-effector — dicetak 3D menggunakan filamen PETG pada printer Creality Ender 3 V3 Plus.
- Desain komponen dibuat menggunakan Autodesk Fusion.
- Lima motor servo **Dynamixel AX-12A** yang dirangkai seri, menyediakan engsel putar independen untuk base, bahu, siku, dan pergelangan tangan ganda.
- Kendali menggunakan mikrokontroler dengan komunikasi serial pada 57600 bps.

## Struktur Repositori

```
├── 5DofARM.py              # Skrip kendali utama (FK, IK/DLS, komunikasi serial ke Dynamixel)
├── Simulasi_URDF_GUI.m     # GUI simulasi lengan robot berbasis URDF di MATLAB
├── Test_Collision.m        # Simulasi Monte Carlo untuk deteksi area rawan tabrakan
├── meshes/                 # File mesh (.stl) komponen fisik lengan robot
├── LICENSE
└── README.md
```

## Dasar Matematika

### Forward Kinematics — Transformasi Homogen Denavit-Hartenberg

Setiap frame *i* dihubungkan ke frame sebelumnya melalui matriks transformasi homogen 4x4:

```
        [ cosθᵢ   -sinθᵢ·cosαᵢ    sinθᵢ·sinαᵢ    aᵢ·cosθᵢ ]
Tᵢ  =   [ sinθᵢ    cosθᵢ·cosαᵢ   -cosθᵢ·sinαᵢ    aᵢ·sinθᵢ ]
        [   0         sinαᵢ          cosαᵢ          dᵢ    ]
        [   0            0              0            1    ]
```

Keterangan:
- `Tᵢ` : matriks transformasi homogen
- `θᵢ` : sudut sendi (joint angle)
- `αᵢ` : pilin link (link twist)
- `aᵢ` : panjang link (link length)
- `dᵢ` : offset link (link offset)

Posisi akhir end-effector diperoleh dari perkalian berurutan seluruh matriks transformasi frame (8 frame, kombinasi sendi dinamis dan frame statis hasil ekstraksi geometri fisik robot).

**Parameter DH robot ini:**

| Frame (i) | aᵢ (m) | αᵢ (°) | dᵢ (m) | θᵢ (°) | Keterangan |
|---|---|---|---|---|---|
| 1 | 0.0 | 0.0 | 0.021 | θ₁* | Base joint dinamis (Yaw) |
| 2 | 0.0075 | -90.0 | 0.006448 | 90.0 | Transisi orientasi sumbu bahu |
| 3 | 0.036878 | 0.0 | 0.0215 | θ₂* | Shoulder joint dinamis (Pitch) |
| 4 | 0.120266 | 0.0 | -0.0051 | 16.3429 | Geometri statis Link 1 |
| 5 | 0.036878 | 0.0 | 0.005 | θ₃* | Elbow joint dinamis (Pitch) |
| 6 | 0.008 | 90.0 | 0.015 | 102.529 | Geometri statis Link 2 |
| 7 | 0.036878 | 0.0 | 0.0735 | θ₄* | Wrist joint dinamis (Pitch) |
| 8 | 0.008 | 90.0 | 0.0465 | 102.529 | Jarak absolut ke End-Effector |

\* `θₓ*` merepresentasikan variabel sudut dinamis dari aktuator, yang berubah real-time sesuai hasil perhitungan inverse kinematics.

### Inverse Kinematics — Damped Least Squares (DLS)

Untuk menghindari ketidakstabilan pada titik singularitas mekanis (kelemahan metode pseudo-invers konvensional), perubahan sudut sendi per iterasi dihitung dengan menambahkan faktor redaman:

```
Δθₖ = Jᵀ (J·Jᵀ + λ²·I)⁻¹ · eₖ
```

Keterangan:
- `Δθₖ` : vektor perubahan sudut sendi
- `J` : matriks Jacobian numerik (dihitung dengan diferensiasi maju berhingga)
- `Jᵀ` : transpos matriks Jacobian
- `λ` : skalar redaman (damping factor)
- `I` : matriks identitas
- `eₖ` : vektor error spasial (spatial error)

Iterasi berlangsung hingga vektor error spasial konvergen menuju target koordinat, tanpa mekanisme *joint limit clamping* (dinonaktifkan untuk menghindari minima lokal).

## Hasil Eksperimen (Ringkasan)

| Aspek | Hasil |
|---|---|
| Deviasi rata-rata Forward Kinematics | 0,0555 m (X: 0,0315 m, Y: 0,0358 m, Z: 0,0153 m) |
| Error residual Inverse Kinematics (DLS) | 1–4 mm (0,25%–1,02% dari radius jangkauan) |
| Batas keamanan vertikal bawah | 0,025 m |

## Persyaratan

- Python 3.x dengan pustaka `dynamixel_sdk`
- MATLAB (untuk simulasi URDF dan uji tabrakan)
- Port serial (COM) untuk komunikasi dengan motor Dynamixel AX-12A

## Cara Menjalankan

1. Hubungkan motor Dynamixel AX-12A ke port serial (default: `COM6`, baudrate `57600`).
2. Jalankan `5DofARM.py` untuk mengendalikan lengan robot secara langsung.
3. Gunakan `Simulasi_URDF_GUI.m` di MATLAB untuk menjalankan simulasi virtual sebelum eksekusi ke perangkat fisik.
4. Gunakan `Test_Collision.m` untuk menjalankan simulasi Monte Carlo dan memvalidasi batas ruang kerja aman.

## Publikasi Terkait

Lie, J., Prasmada, D. R., Bawono, B., & Pamosoaji, A. K. (2026). *Pengembangan Robot Lengan untuk Pembelajaran Kuliah Otomasi dan Robotika di UAJY*. SENAPAS 2026.

## Kontributor

Kelompok Studi Robotika (KSR), Fakultas Teknologi Industri, Universitas Atma Jaya Yogyakarta.

## Lisensi

Proyek ini dilisensikan di bawah [MIT License](LICENSE).