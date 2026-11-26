# Tes RS05 langsung dari PC via CANable v2

```
PC --USB--> CANable v2 (firmware slcan, COMx) --CAN 1 Mbit/s--> RS05 (ID 0x7F)
```
Tanpa STM32. Dipakai untuk memastikan motor, wiring, dan parameter benar sebelum lewat firmware STM32H723.

### Lewat STM32 (opsional): interface `stm`
```
PC --USB--> STM32H723 (Firmware_Gateway) --SPI--> 2x MCP2515 --CAN1 / CAN2--> J1,J2 / J3,J4
```
Semua tool di folder ini bisa memakai STM32 sebagai pengganti CANable: pilih **Interface = `stm`** dan
**Port = COM port USB STM**. Untuk CLI, pakai `--iface stm -p COMx`.
- Python tetap mengerjakan semuanya: mode, target, gain, safety, dan tes. STM hanya mengirim frame ke
  CAN1/CAN2 sesuai urutan, meneruskan semua frame yang diterima, dan menjalankan watchdog (STOP semua motor
  kalau Python diam lebih dari 0.3 s).
- `stm-sim` menjalankan STM simulasi dengan motor simulasi, untuk latihan tanpa hardware.
- Jalur CANable langsung (`slcan`, `gs_usb`) tidak berubah.

Detail firmware, cara memasukkannya ke project CubeIDE, dan protokolnya ada di
[../../Firmware_Gateway/README.md](../../Firmware_Gateway/README.md). Kodenya di `rs05_stm.py` (bus
python-can) dan `rs05_stm_emu.py` (simulasi).

## Setup
```powershell
cd rs05_hal
.venv\Scripts\Activate.ps1
pip install -r python\requirements.txt
cd python\canable
```
- CANable v2 harus pakai firmware **slcan** (muncul sebagai COM port). Kalau pakai candleLight: `--iface gs_usb -p 0`.
- CAN H/L ke motor, **terminasi 120 Ω di kedua ujung** (CANable punya jumper/switch terminasi).
- PSU 48 V (RS05: 15–60 V), mulai dengan current limit PSU 1–2 A.

## GUI (disarankan)
```powershell
python rs05_gui.py --sim     # latihan dulu tanpa hardware (motor simulasi, hard stop di ±120°)
python rs05_gui.py           # motor asli: pilih COM port, Connect
```
| Tab | Isi |
|---|---|
| 1 Setup | **Zero** di pose referensi · **ajari batas sendi** dengan menggerakkan tangan ke ujung lalu tekan *Set MIN/MAX here* (dipasang `inset` derajat sebelum titik tabrakan) · current limit, suhu maks · **ganti CAN ID** motor |
| 2 Position | PP / CSP, slider target yang dibatasi oleh limit, jog ±1/5/10°, go 0 / mid |
| 3 Velocity | slider speed, tombol tahan-untuk-jog, speed otomatis jadi 0 sebelum batas |
| 4 MIT / Impedance | slider Kp (kekakuan), Kd (redaman), τff, target; preset; dinding virtual di batas |
| 5 Friction | sweep kecepatan bolak-balik di antara batas → fit `τ = Fc·sign(ω) + B·ω + offset`, plus tes breakaway (static friction) |

Kiri: tombol **STOP (Esc)**, nilai live, bar rentang sendi (hijau = boleh, merah = terlarang), log event.
Bawah: plot live posisi / kecepatan / torsi + iq. *Record CSV* menyimpan semuanya ke `logs/`.

### Setup CAN ID tiap motor (sekali saja)
Motor baru semua ber-ID 0x7F (127); dua motor dengan ID sama di satu bus tidak bisa dibedakan.
1. Pasang **hanya satu** motor ber-ID 127 di bus → Connect (ID 0x7F).
2. Tab 1 → *D. Motor CAN ID* → isi ID baru (mis. J1=1, J2=2, J3=3, J4=4) → **Change ID**.
   GUI mengecek dulu ID itu belum dipakai, mengirim type 7, memverifikasi motor menjawab di ID baru,
   lalu reconnect; limit/zero setting ikut pindah ke ID baru.
3. Tambah motor berikutnya (masih 127) ke bus → Connect ke 0x7F → ganti ke ID berikutnya. Ulangi.
4. Power-cycle semua motor, **Scan bus**: harus muncul 1, 2, 3, 4. Pilih motor lewat dropdown *Motor ID*.
CLI: `python rs05_canable_test.py --id 0x7F setid --new 1`

### Motor dengan ID tidak diketahui (hilang / tidak tercatat)
*Scan IDs* biasa hanya mencari ID 1..127 di 1 Mbit/s. Kalau motor tidak muncul, pakai **Deep scan**:
- GUI: tab 1 → *D. Motor CAN ID* → **Deep scan (lost ID)…** (centang *all bitrates* kalau masih tidak ketemu).
- CLI: `python rs05_canable_test.py -p COM9 scan --deep` atau `scan --all-bitrates`.

Deep scan: dengar pasif 1 s, kirim *get ID* (type 0) ke **ID 0..255**, lalu *stop* (type 4) ke ID yang belum
menjawab (motor selalu membalas stop dengan feedback; ini juga mematikan motor yang sedang jalan), dan
mendekode setiap frame yang diterima. Hasilnya: ID, bitrate, UID MCU (untuk membedakan motor fisik), dan
"other traffic" kalau ada frame lain. Frame 11-bit di sana berarti motor diubah ke protokol MIT/CANopen;
kembalikan ke protokol private dengan software RobStride.
Setelah ketemu: GUI otomatis mengisi *Motor ID* (dan bitrate) → Connect → **Change ID** ke ID yang tercatat.

Proteksi tabrakan (`rs05_controller.py`, 100 Hz, di semua mode):
- gerak **ditolak** sampai batas diajarkan (atau *free shaft* dicentang);
- posisi: target di-clamp ke batas; STOP kalau lewat >5°;
- kecepatan: speed dipotong ke 0 saat jarak pengereman `v²/2a` menyentuh batas;
- MIT: zona redam + dinding pegas-redam di batas, rem overspeed, STOP kalau lewat >12°;
- feedback hilang >0.3 s, fault motor, suhu lebih → STOP.
Batas & setting disimpan per motor ID di `rs05_gui_config.json`. Setelah zero, batas otomatis digeser.
Ini **bukan** sistem keselamatan: tetap pegang saklar PSU.

### Pilihan MIT / posisi untuk friction dan tuning
**Tab 5 Friction → *Sweep drive*.** Pilih cara membuat lintasan berkecepatan konstan:
- **velocity loop**: cara referensi (spd_ref).
- **position CSP**: posisi dengan `limit_spd = v`.
- **MIT**: θd naik dengan kecepatan v, ωd = v.

Untuk MIT, pakai **Kp kecil (≈2) dan Kd ≈1**. θd dikirim 100 Hz sebagai tangga, jadi Kp besar membuat
riak torsi yang lebih besar dari gesekannya sendiri, dan sendi mengalami stick-slip. Uji breakaway selalu
memakai ramp torsi MIT. Kalau drive = MIT, perpindahan ke tengah rentang juga memakai MIT, bukan CSP.

Di simulator (Fc 0.060 / B 0.012 asli):

| Drive | Fc | B |
|---|---|---|
| velocity | 0.060 | 0.012 |
| CSP | 0.060 | 0.012 |
| MIT | 0.060 | 0.0099 (sekitar 17 % rendah karena riak) |

**Tab 6 Tuning (step).** Tes step: start → start+step → start, diulang. Kontrolnya bisa **MIT (Kp/Kd/ramp)**,
**Position PP**, atau **Position CSP**. Hasil per step:
- rise time 10–90 %;
- overshoot;
- settling (pita ±band %, minimal 0.2°);
- error steady-state;
- torsi puncak.

Ada juga plot respons ternormalisasi dan torsi. Tombol *Save results* menulis `logs/step_<mode>_<waktu>.json`
dan `.csv`.

Kalau step berakhir di **luar** pita, itu error statis karena gesekan atau gravitasi dibanding kekakuan.
Naikkan Kp / loc_kp, atau tambah τff; dwell yang lebih lama tidak membantu.

**Motor loop gains** (`loc_kp`, `spd_kp`, `spd_ki`, `spd_filt_gain`):
- Read, Write (RAM), dan Restore read.
- *Save ALL params to motor flash…* menyimpan **semua** parameter motor, bukan hanya gain, dan hanya boleh
  saat motor STOP.
- Gain ini mengatur loop internal PP / CSP / velocity, dan **tidak berpengaruh di MIT**.
- Ubah satu gain sekali, ±20–30 %, lalu bandingkan respons step-nya.
- Default gain RS05/RS02 asli belum diukur di sini. Selalu tekan Read dulu.

## CLI (tanpa GUI): urutan tes yang disarankan
| # | Perintah | Yang dicek |
|---|---|---|
| 1 | `python rs05_canable_test.py scan` | ID motor (default 0x7F) |
| 2 | `python rs05_canable_test.py monitor` | **posisi**, speed, torsi, iq, VBUS, suhu, rate feedback (putar poros dengan tangan) |
| 3 | `python rs05_canable_test.py params` | semua parameter penting terbaca |
| 4 | `python rs05_canable_test.py zero` | **zeroing**: posisi sekarang = 0, diverifikasi lewat `mechPos` |
| 5 | `python rs05_canable_test.py speed --vel 2 5 -5 --hold 3` | **speed control**: error tracking + arus |
| 6 | `python rs05_canable_test.py pos --mode pp --deg 90 -90 0` | **position control** PP (profil trapesium motor) |
| 7 | `python rs05_canable_test.py pos --mode csp --deg 45 0 --vmax 2` | position control CSP |
| 8 | `python rs05_canable_test.py mit --deg 30 -30 0 --kp 10 --kd 0.5` | **MIT control**: step posisi dengan ramp |
| 9 | `python rs05_canable_test.py mit --spring --kp 5 --kd 0.2 --time 15` | MIT sebagai pegas: dorong poros dengan tangan |
| 10 | `python rs05_canable_test.py current` | **current draw** idle (iq + VBUS) |
| 11 | `python rs05_canable_test.py current --iq 0.3 0.6` | current mode → torsi & Kt (**poros dikunci/dibebani!**) |

Tanpa sub-perintah (`python rs05_canable_test.py`) → menu interaktif, koneksi tetap terbuka.

Opsi umum: `-p COM9 --id 0x7F --host 0xFD --bitrate 1000000 -y` (`-y` = lewati konfirmasi).
Lihat semua opsi: `python rs05_canable_test.py <tes> -h`.

Setiap tes menulis CSV ke `logs/` (t, setpoint, pos, vel, torque, iq, VBUS, suhu, state, fault) dan mencetak ringkasan.

## Catatan protokol (private protocol RobStride)
- **MIT di sini = komunikasi type 1 (operation control, run_mode 0)** di protokol private. Tidak perlu
  mengganti protokol motor ke "MIT protocol" (type 25) lalu power-cycle seperti script lama.
  Kalau motor kamu sempat di-switch ke MIT protocol oleh script lama, kembalikan ke private dulu
  (software resmi RobStride), karena tool ini tidak akan mendapat feedback.
- Skala RS05: posisi ±4π rad, kecepatan ±50 rad/s, **torsi ±5.5 Nm**, Kp 0–500, Kd 0–5.
- Ganti run_mode selalu: stop → tulis run_mode → enable → tulis limit → tulis target.
- Arus: `iqf` (0x701A) = arus fasa sumbu-q (A), bukan arus dari PSU. VBUS dari 0x701C.
- Zero (type 6) — cek setelah power-cycle; kalau hilang, jalankan `zero --save` (type 22).
- Batas aman default: arus 2 A, speed ≤ 20 rad/s, step posisi ≤ 360°, MIT kp ≤ 50, tff ≤ 1 Nm, suhu 70 °C.
  Naikkan dengan opsi `--max-*` / `--cur` secara sadar.

## 4 motor sekaligus (tes ID + tes kabel/konektor arus tinggi)
Panduan lengkap: [MULTI_GUIDE.md](MULTI_GUIDE.md)

```powershell
python rs05_multi_gui.py --sim     # 4 motor simulasi
python rs05_multi_gui.py           # motor asli, semua di satu CANable
```
1. **Tabel plan** (satu baris per motor): ID, mode (`pos-pp`/`pos-csp`/`vel`/`mit`), *steps*
   `nilai[:detik], ...` (derajat; rad/s untuk vel), dwell, cycles (0 = sampai batas waktu), delay start,
   I lim, Vmax, Acc, Kp/Kd/Ramp (MIT), Rel, End (hold/stop). Disimpan di `rs05_multi_plan.json`.
2. **Connect** → **Check IDs**: tiap ID menjawab dengan tepat satu UID (dua UID = ID dobel), scan 1..127
   (ID tak terduga, mis. sisa 0x7F), dan baca parameter per ID. **Identify** = goyang ±5° supaya kelihatan
   sendi mana yang punya ID itu.
3. Batas sendi per motor: **MIN/MAX** (gerakkan dengan tangan) atau **Free**; sama dengan `rs05_gui.py`.
4. **RUN TEST**: semua motor jalan bersamaan. Berhenti otomatis saat batas waktu, VBUS < ambang,
   atau (opsional) satu motor trip. **STOP ALL / Esc** kapan saja.
5. Laporan per motor: feedback gap maks / jumlah gap > 50 ms, iq puncak & RMS, torsi puncak, suhu,
   VBUS min/max, tx error, fault, error akhir step. CSV + report di `logs/multi_*`.
   Gap / tx error / dip VBUS yang naik seiring arus → curigai kabel/konektor motor itu.

Untuk stress arus: naikkan *I lim* dan *Acc*, delay 0 (semua start bersamaan), step bolak-balik.
Arus fasa (iq) ≠ arus PSU; set limit PSU dan pegang saklarnya.
Headless: `python rs05_multi.py -p COM9 --check-ids --ids 1 2 3 4`, `python rs05_multi.py -p COM9 --duration 60`.

## Motor tidak menjawab? Cek protokolnya (private / MIT / CANopen)
Motor RS05 hanya bicara **satu** protokol (disimpan di flash, berlaku setelah power-cycle). Tool di folder ini
hanya bicara *private*. Script lama di `_archive/` pernah mengirim type 25 → **MIT protocol**, jadi motor bisa
"tersangkut" di MIT dan terlihat mati.
```powershell
python rs05_protocol.py -p COM9                     # deteksi (tambah --all-bitrates kalau perlu)
python rs05_protocol.py -p COM9 --boot 15           # + dengarkan 15 s sambil motor di-power-cycle
python rs05_protocol.py -p COM9 --to-private mit --id 0x7F    # kembalikan ke private, lalu power-cycle
python rs05_protocol.py -p COM9 --to-private canopen          # dari CANopen (frame VERIFY)
python rs05_protocol.py --sim mit                   # coba dulu di simulator
```
GUI: tab 1 → *D* → **Detect protocol (private / MIT / CANopen)…** → menawarkan switch ke private.

| Probe | Yang dikirim | Balasan berarti |
|---|---|---|
| private | type 0 *get ID* ke ID 0..255 (29-bit) | PRIVATE, ID = bit 15..8 |
| MIT | *stop* `FF FF FF FF FF FF FF FD` ke ID 11-bit 1..254 (hanya disable) | MIT, ID = byte 0 balasan (balasan di ID = host ID) |
| CANopen | SDO read 0x1000 ke 0x600+node | CANopen, node = ID balasan − 0x580 (node = CAN ID private, atau 1 di firmware lama) |
| boot | (opsional) dengar saat power-cycle | CANopen boot-up 0x700+node |

Switch ke private (manual RS05): MIT command 8 = ID 11-bit motor, `FF FF FF FF FF FF 00 FD`;
CANopen = frame extended 0xFFF `01 02 03 04 05 06 00 00` (**VERIFY**). Motor membalas dengan UID 64-bit,
lalu **harus power-cycle**. Kalau gagal: software resmi RobStride (parameter `protocol_1`, 0x2022).

## Kecepatan "tidak pernah 0", grafik bergetar, motor menghangat (dan RS02)

- **Bias decode (sudah diperbaiki).** Firmware motor membulatkan ke bawah (truncate) saat mengubah float ke
  uint16. Dulu kode men-decode tanpa koreksi, jadi motor diam terbaca **-0.0008 rad/s** dan **-0.011°**
  setelah zero. Itu persis setengah langkah encoding, terlihat di `logs/gui_20261125_134142.csv`.
  Sekarang `u2f_fb()` mengambil nilai tengah langkah, jadi motor diam terbaca 0.
- **Spike ±0.085 rad/s adalah encoder 1 count.** Encoder motor berpindah 1 count (0.022°), lalu estimasi
  kecepatan internal motor melonjak satu frame. Ini data asli dari motor, bukan getaran mekanis.
  - GUI menampilkan garis abu-abu "raw" dan garis biru hasil median 5 frame. Angka kecepatan juga memakai
    median.
  - CSV tetap menyimpan data mentah.
  - Plot punya rentang-y minimum (±0.25 rad/s, ±1°, ±0.1 Nm), jadi noise 1 LSB tidak lagi di-zoom sampai
    memenuhi grafik.
- **MIT dekat batas.** Kode lama memakai tanda kecepatan yang ber-noise untuk menentukan damping dinding.
  Akibatnya Kd berganti-ganti 0.5↔1.5 dan menimbulkan chatter. Sekarang ada deadband 0.2 rad/s.
- **Panas.** Motor yang *enabled* (pos / vel / MIT hold) selalu menghangat sedikit karena loop arus dan
  PWM tetap jalan, walaupun torsinya ≈0. Ini fisika, bukan bug. Untuk mengurangi panas:
  - pilih `End = stop` di plan multi-motor, atau tekan STOP saat tidak dipakai;
  - di velocity mode, hold di 0 rad/s membuat loop PI terus "mencari" posisi; untuk hold, pakai mode posisi;
  - di MIT, Kd besar akan mengalikan spike encoder (0.5 × 0.085 = 0.04 Nm per spike). Pakai Kd seperlunya.
- **Model motor.** RS02 punya skala berbeda (±17 Nm, ±44 rad/s); RS05 ±5.5 Nm, ±50 rad/s. Pilih
  **motor model = RS02** untuk ID motor RS02:
  - di GUI tunggal: tab Setup → C;
  - di GUI multi: kolom *Model*.
  
  Kalau tidak, torsi RS02 terbaca ~3× terlalu kecil dan v_des MIT meleset 12 %. Pilihan model disimpan
  per ID di `rs05_gui_config.json`. CLI `rs05_canable_test.py` masih memakai skala RS05.

## Pakai dari script sendiri
```python
from rs05_can import RS05, open_bus, MODE_SPEED
bus = open_bus("slcan", "COM9")
m = RS05(bus, motor_id=0x7F)
m.set_mode(MODE_SPEED); m.write("limit_cur", 2.0); m.write("spd_ref", 3.0)
print(m.s.fb, m.read("iqf"))
m.stop(); m.close(); bus.shutdown()
```

Script lama ada di `python/_archive/` (tidak dipakai lagi).
