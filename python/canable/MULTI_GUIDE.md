# Panduan `rs05_multi_gui.py`: 4 motor sekaligus di satu CANable

GUI ini menjalankan sampai 4 motor RobStride (RS05 / RS02), masing-masing dengan CAN ID sendiri, di **satu
bus CAN**. Tiap motor punya urutan target, mode, dan batas waktunya sendiri.

GUI ini dipakai untuk dua tujuan:

1. **Membuktikan setting ID benar.** Tiap ID dijawab tepat satu motor, tidak ada ID lain yang tertinggal,
   dan perintah ke ID *n* benar-benar menggerakkan sendi *n*.
2. **Menguji kabel dan konektor dalam kondisi operasional (arus tinggi).** Feedback yang putus, error
   kirim, VBUS yang turun, dan fault direkam per motor.

> Untuk satu motor (tuning, MIT detail, friction, ganti ID) pakai `rs05_gui.py`. Lihat
> [MIT_GUIDE.md](MIT_GUIDE.md) dan [README.md](README.md). Kedua GUI memakai file batas sendi yang sama
> (`rs05_gui_config.json`).

---

## 1. Menjalankan

```powershell
cd rs05_hal\python\canable
python rs05_multi_gui.py --sim          # latihan: 4 motor simulasi, tanpa hardware
python rs05_multi_gui.py -p COM9        # motor asli lewat CANable (slcan)
```

**Syarat sebelum memakai motor asli:**

| Cek | Kenapa |
|---|---|
| Semua motor memakai **private protocol** | Motor yang tersangkut di mode MIT atau CANopen tidak menjawab. Cek dengan `python rs05_protocol.py -p COM9` atau tombol *Detect protocol* di `rs05_gui.py` |
| Tiap motor punya **ID unik** (misalnya 1, 2, 3, 4) | Motor baru semuanya 0x7F. Ganti ID **satu per satu** di `rs05_gui.py` → Setup → D |
| Bitrate 1 Mbit/s, terminasi 120 Ω di **kedua ujung** bus | Tanpa terminasi, frame hilang saat bus ramai |
| Batas arus PSU sudah di-set, saklar bisa dijangkau | iq yang ditampilkan adalah arus fasa, bukan arus PSU |
| Sendi bebas bergerak di seluruh rentang tes | |

---

## 2. Tampilan jendela

```
┌ Interface [slcan] Port [COM9] Bitrate [1000000] Host [0xfd] [Connect] [Check IDs] [STOP ALL (Esc)] ┐
├ Plan: satu baris per motor ─────────────────────────────────────────────────────────────────────┤
│ On Name ID Mode Steps Dwell Cycles Delay I lim Vmax Acc Kp Kd Ramp Rel End Model | limits Free  │
│ [MIN][MAX][Zero][Identify]                                                       (x4 baris)     │
├ Time limit s [60]  Stop all if VBUS < V [40]  ☑ stop all if one motor trips  ☑ record           │
│ [▶ RUN TEST] [Save plan] [progress ███░░] running 12 / 60 s                                     │
├ Tabel status per motor │ Plot: posisi (°), iq (A), VBUS minimum (V)                             │
│ Events / ID check      │                                                                        │
└──────────────────────────────────────────────────────────────────────────────────────────────────┘
```

- **Esc** = STOP ALL, kapan saja, termasuk saat fokus ada di kotak isian.
- Plan disimpan otomatis ke `rs05_multi_plan.json` saat RUN, *Save plan*, atau saat jendela ditutup.

---

## 3. Mengisi plan (satu baris = satu motor)

| Kolom | Arti | Contoh |
|---|---|---|
| **On** | Ikut di-connect dan dites atau tidak | ☑ |
| **Name** | Nama sendi (dipakai di laporan dan CSV) | J1 |
| **ID** | CAN ID motor (desimal atau `0x..`) | 0x1 |
| **Mode** | `pos-pp` (posisi dengan profil trapesium dari motor), `pos-csp` (posisi, kecepatan dibatasi), `vel` (kecepatan), `mit` (impedansi) | pos-csp |
| **Steps** | Urutan target `nilai[:detik], ...`. Derajat untuk pos dan mit, rad/s untuk vel | `0:2, 30:1.5, -30:1.5` |
| **Dwell s** | Lama tiap step yang tidak diberi `:detik` | 2 |
| **Cycles** | Berapa kali urutan diulang. **0 = ulang terus sampai time limit** | 0 |
| **Delay s** | Motor mulai sekian detik setelah RUN | 0 |
| **I lim A** | Batas arus fasa (pos dan vel, maks 11 A). **Tidak berlaku untuk MIT** | 2 |
| **Vmax** | pos: kecepatan maksimum (rad/s). vel: batas atas step (step di atas Vmax ditolak) | 3 |
| **Acc** | Percepatan (rad/s²) untuk pos-pp dan vel | 10 |
| **Kp / Kd** | Kekakuan (Nm/rad) dan redaman (Nm·s/rad), hanya untuk MIT | 10 / 0.5 |
| **Ramp** | Kecepatan pergeseran target MIT (rad/s), supaya tidak melompat | 2 |
| **Rel** | ☑ = step relatif terhadap posisi saat motor mulai, ☐ = absolut (terhadap zero) | ☐ |
| **End** | Setelah cycle terakhir: `hold` (tetap menahan sampai time limit) atau `stop` (lemas) | stop |
| **Model** | `RS05` (EduLite 05) atau `RS02`. **Wajib benar.** RS02 memakai skala ±17 Nm / ±44 rad/s | RS02 |

**Catatan:**
- Step diberi jeda waktu, bukan "tunggu sampai sampai". Kalau waktunya terlalu pendek, motor pindah ke
  target berikutnya sebelum mencapai target sebelumnya. Error di akhir tiap step dicatat di laporan.
- Untuk motor yang tidak dipakai lama, pilih `End = stop`. Motor yang di-*hold* tetap enabled dan pelan-pelan
  menghangat.
- Jangan "hold" di mode `vel` pada 0 rad/s. Untuk menahan posisi, pakai mode `pos-*`.

### Contoh plan

**Tes ID dan gerak ringan** (aman, arus kecil):

| Name | ID | Mode | Steps | Cycles | I lim | Vmax | Acc | End |
|---|---|---|---|---|---|---|---|---|
| J1 | 1 | pos-csp | `0:2, 15:2, -15:2` | 2 | 1.5 | 2 | 5 | stop |
| J2 | 2 | pos-csp | `0:2, -15:2, 15:2` | 2 | 1.5 | 2 | 5 | stop |
| J3 | 3 | vel | `0:1, 1:1.5, -1:1.5, 0:1` | 2 | 1.5 | 2 | 5 | stop |
| J4 | 4 | mit | `0:2, 10:2, -10:2` | 2 | - | - | - | stop |

**Stress kabel dan konektor** (arus tinggi, semua motor menarik arus bersamaan):

| Name | ID | Mode | Steps | Cycles | Delay | I lim | Vmax | Acc |
|---|---|---|---|---|---|---|---|---|
| J1 | 1 | pos-pp | `40:0.8, -40:0.8` | 0 | 0 | 6 | 8 | 60 |
| J2 | 2 | pos-pp | `-40:0.8, 40:0.8` | 0 | 0 | 6 | 8 | 60 |
| J3 | 3 | pos-pp | `40:0.8, -40:0.8` | 0 | 0 | 6 | 8 | 60 |
| J4 | 4 | pos-pp | `-40:0.8, 40:0.8` | 0 | 0 | 6 | 8 | 60 |

Arus naik saat percepatan dan pengereman, jadi yang membuat arus tinggi adalah **Acc** besar dan
**bolak-balik cepat**, bukan sudut yang besar. Naikkan I lim dan Acc **bertahap** (misalnya 2 → 4 → 6 A),
satu run per langkah, lalu bandingkan laporannya.

---

## 4. Alur kerja

### Langkah 1: Connect
Pilih Interface dan Port, lalu tekan **Connect**. Hanya baris dengan **On** ☑ yang di-connect.

- Baris tabel status berwarna **merah** dengan `LOST` berarti ID itu tidak mengirim feedback. Kemungkinannya:
  ID salah, kabel atau konektor, power, atau motor masih di protokol lain.
- Model per motor diambil dari kolom **Model** saat Connect. Kalau model diubah setelah Connect, model baru
  dipakai saat RUN.

### Langkah 2: Check IDs
Tekan **Check IDs**. Tiga pemeriksaan dijalankan, dan hasilnya muncul di popup dan di panel *Events*:

1. **Satu ID, satu motor.** Tiap ID di plan harus menjawab dengan **tepat satu UID**.
   `DUPLICATE` berarti dua motor memakai ID yang sama. Ganti ID salah satunya **sendirian di bus**.
2. **Tidak ada ID asing.** ID 1..127 di-scan. `UNEXPECTED IDs` berarti ada motor dengan ID yang tidak
   ada di plan, misalnya masih 0x7F dari pabrik.
3. **Alamat benar.** Satu parameter (mechPos, VBUS) dibaca dari tiap ID dan harus dijawab ID itu sendiri.

Hasil harus **PASS** sebelum lanjut.

### Langkah 3: Identify (ID mana = sendi mana)
Tekan **Identify** di tiap baris. Motor itu bergoyang ±5° (MIT lembut, Kp 5) lalu berhenti. Lihat sendi
mana yang bergerak. Kalau yang bergerak bukan sendi yang dimaksud, tukar ID di plan, atau ganti ID motornya.

### Langkah 4: Batas sendi
Tes **tidak mau jalan** sampai tiap motor punya batas. Ada dua cara:

- **Ajarkan batasnya.** Pastikan motor dalam keadaan STOP (lemas). Gerakkan sendi dengan tangan ke ujung
  bawah lalu tekan **MIN**, ke ujung atas lalu tekan **MAX**. Batas otomatis dipasang 5° di dalam ujung
  fisik. Kolom *joint limits* lalu menampilkan `[min, max]°`.
- **Free.** Centang ini hanya kalau **tidak ada apa pun** terpasang di poros. Batasnya menjadi ±700°.

**Zero** menjadikan posisi sekarang 0°. Hanya koordinatnya yang berubah, motor tidak bergerak. Batas
sendi ikut digeser. Untuk mode posisi, posisi di motor (mechPos) dan feedback harus cocok. Kalau ditolak
dengan pesan "re-zero", tekan Zero dulu.

### Langkah 5: Pengaturan tes
| Isian | Arti |
|---|---|
| **Time limit s** | Batas waktu total. Semua motor berhenti saat waktunya habis, apa pun kondisinya |
| **Stop all if VBUS < V** | Semua motor berhenti kalau VBUS terendah turun di bawah nilai ini (0 = mati). Sesuaikan dengan suplai: untuk **24 V** isi sekitar 20, untuk **48 V** sekitar 40 |
| **stop all if one motor trips** | Satu motor trip → semua berhenti (disarankan untuk tes kabel) |
| **record CSV + report** | Tulis `logs/multi_<waktu>.csv` dan `logs/multi_<waktu>_report.txt` |

### Langkah 6: RUN TEST
Tekan **▶ RUN TEST**. Sebelum mulai, ada konfirmasi berisi ringkasan tiap motor dan jumlah batas arus.

Tes ditolak sebelum ada yang bergerak kalau:
- batas sendi belum diajarkan;
- tidak ada feedback;
- ada fault aktif;
- sendi berada di luar batas;
- ada step vel di atas Vmax;
- ada ID ganda.

Selama tes berjalan:
- Tabel status diperbarui 10× per detik.
- Plot menampilkan posisi dan iq per motor (warna sesuai baris) serta VBUS terendah.
- Progress bar menunjukkan waktu yang sudah berjalan.

Tes berakhir kalau salah satu terjadi:
- time limit habis;
- semua motor selesai dengan `End = stop`;
- satu motor trip (kalau opsinya dicentang);
- VBUS di bawah ambang;
- STOP ALL atau Esc ditekan.

Di akhir tes, **semua motor di-stop** dan jendela laporan muncul.

---

## 5. Tabel status

| Kolom | Arti | Waspada kalau |
|---|---|---|
| fb Hz | Jumlah feedback per detik dari motor | turun jauh dari biasanya, atau `LOST` |
| mode / test | Mode controller dan state tes (`wait`, `run sX cY`, `done`, `trip`, `off`) | `trip` |
| setpoint, pos°, vel | Target, posisi, kecepatan. vel adalah median 5 frame, jadi spike encoder ±0.085 rad/s tidak tampil | |
| τ Nm, iq A, iq pk | Torsi estimasi motor, arus fasa (di-poll 50 Hz), puncak iq selama tes | iq pk mendekati I lim terus-menerus |
| T °C | Suhu motor. Controller berhenti di atas `temp max` (default 70 °C) | naik terus saat hold |
| VBUS | Tegangan bus di motor itu | turun saat arus naik |
| gaps | Jumlah jeda feedback > 50 ms selama tes | > 0 |
| fault / trip | Fault dari motor atau alasan trip | ada isinya |

Baris **merah** berarti feedback hilang, ada fault, atau motor trip.

---

## 6. Membaca laporan (tes kabel dan konektor)

Contoh:

```
motor   ID  mode    state cyc  fb n  gap max gaps>50ms iq pk A iq rms tq pk Nm T max C VBUS min VBUS max tx err faults
J1       1  pos-pp  off   12   5980    14ms        0    5.80   2.10    1.74    41.0    46.90    48.02      0  -
J3       3  pos-pp  trip   7   3410   310ms        3    5.95   2.30    1.80    40.5    43.10    48.01      2  undervoltage
...
bus: error frames 0, rx errors 0, frames per ID {1: 9012, 2: 9020, 3: 5120, 4: 9005}
```

| Tanda | Kemungkinan penyebab |
|---|---|
| `gaps>50ms` / `gap max` besar **hanya di satu motor**, makin sering saat arus naik | Konektor CAN atau power motor itu longgar atau kontaknya buruk |
| `VBUS min` satu motor jauh lebih rendah dari yang lain | Kabel power ke motor itu terlalu kecil atau terlalu panjang, atau konektornya panas |
| `VBUS min` semua motor turun bersamaan | PSU (current limit, ripple) atau kabel induk |
| fault `undervoltage` | VBUS sempat jatuh di bawah batas motor. Lihat dua baris di atas |
| `tx err` > 0, `error frames` > 0 | Bus CAN bermasalah: terminasi, kabel CAN, EMI dari kabel power |
| `frames per ID` satu motor jauh lebih sedikit | Motor itu kehilangan frame, atau sempat trip lebih awal |
| `end-of-step error` besar | Step terlalu pendek untuk Vmax/Acc, I lim terlalu kecil, atau ada beban berlebih |

**Cara membandingkan yang adil:**
- Jalankan plan yang sama dengan I lim / Acc yang naik bertahap.
- Lalu **tukar kabel atau konektor antar motor**. Kalau masalahnya ikut pindah, sumbernya kabel. Kalau
  tetap di motor yang sama, sumbernya motor atau driver.

**CSV** (`logs/multi_*.csv`, 50 Hz) berisi per motor: `state, step, setpoint, pos_deg, vel, torque, iq,
temp, vbus, fault`. Bisa dibuka di Excel atau pandas untuk melihat saat persis VBUS turun atau feedback
putus.

---

## 7. Pengaman yang aktif selama tes

Semua pengaman dari `rs05_gui.py` berjalan untuk **tiap motor** (supervisor 100 Hz):

- **Batas sendi.**
  - pos: target dipotong ke batas, STOP kalau sendi lewat lebih dari 5°.
  - vel: kecepatan dipaksa 0 sebelum batas.
  - MIT: dinding virtual, STOP kalau lewat lebih dari 12°.
- **Feedback hilang** lebih dari 0.3 s → STOP.
- **Fault motor** → STOP.
- **Suhu** di atas `temp max` → STOP.
- MIT punya **rem overspeed** (`mit_vel_max`, default 8 rad/s).
- Tingkat tes: time limit, VBUS minimum, dan stop-all-on-trip.

> **I lim tidak membatasi MIT.** Di MIT, torsi = Kp·error + Kd·kecepatan dan hanya dibatasi torsi
> puncak motor. Untuk tes arus yang terkontrol, pakai mode `pos-*` atau `vel`.

---

## 8. Masalah umum

| Gejala | Solusi |
|---|---|
| Connect: baris `LOST` / "No feedback" | Cek power, CAN H/L, terminasi, bitrate, ID. Jalankan `python rs05_canable_test.py -p COM9 scan --deep` atau `python rs05_protocol.py -p COM9` |
| Check IDs: `DUPLICATE` | Dua motor ber-ID sama. Pasang satu motor saja di bus, ganti ID-nya di `rs05_gui.py` |
| Check IDs: `UNEXPECTED 0x7F` | Ada motor yang belum diberi ID (atau baris plan-nya belum dicentang On) |
| RUN ditolak: "Teach the joint limits first" | MIN/MAX belum diajarkan, atau Free belum dicentang |
| RUN ditolak: "mechPos ... re-zero" | Tekan **Zero** di baris itu (motor dalam keadaan stop) |
| Tes langsung berhenti "VBUS ... below" | Ambang VBUS lebih tinggi dari suplai (misalnya 40 V untuk PSU 24 V). Turunkan |
| Torsi RS02 terlihat kecil sekali | Kolom **Model** belum di-set ke RS02 |
| Motor menghangat saat diam | Motor masih enabled (`End = hold`). Pakai `stop` atau STOP ALL saat tidak dipakai |
| Grafik kecepatan di `rs05_gui.py` terlihat "jitter" | Garis abu-abu adalah data mentah (encoder ±1 count). Lihat garis biru (median) |

---

## 9. Tanpa GUI (headless)

```powershell
python rs05_multi.py -p COM9 --check-ids --ids 1 2 3 4    # hanya cek ID
python rs05_multi.py -p COM9 -y --duration 120            # jalankan plan dari rs05_multi_plan.json
python rs05_multi.py --sim -y --duration 10               # latihan di simulator
```

Plan headless dibaca dari `rs05_multi_plan.json`, file yang sama dengan yang disimpan GUI. Jadi
plan bisa disusun di GUI lalu dijalankan dari command line.
