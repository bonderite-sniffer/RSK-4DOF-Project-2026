# Setup STM32CubeMX + integrasi kode HAL (rs05_hal)

Target: **WeAct STM32H723VGT6** (HSE 25 MHz, USB-C di PA11/PA12), 2× MCP2515 (kristal **16 MHz**) di SPI1/SPI2, 4× RobStride EduLite 05 (2 per bus), CAN 1 Mbit/s.

Alurnya: bikin project CubeMX → generate → copy folder `App/` → tempel 4 baris di `main.c` dan 1 baris di `usbd_cdc_if.c` → build → flash.

---

## 1. Project baru
1. CubeMX (atau STM32CubeIDE → *New STM32 Project*) → MCU Selector → **STM32H723VGTx**.
2. Project Manager:
   - Toolchain/IDE: **STM32CubeIDE**
   - Linker Settings: Minimum Heap `0x200`, Minimum Stack **`0x1000`**
   - Advanced Settings: semua driver **HAL** (jangan LL)

## 2. Pinout & konfigurasi

### System Core
| Peripheral | Setting |
|---|---|
| **RCC** | HSE = **Crystal/Ceramic Resonator**. LSE boleh Disable |
| RCC → Parameter Settings → Power Parameters | **Supply Source = PWR_LDO_SUPPLY** ⚠️ (H723 tidak punya SMPS; nilai salah bisa mengunci chip) · Voltage Scale = **Scale 1** |
| **SYS** | Debug = **Serial Wire** · Timebase Source = **SysTick** |
| CORTEX_M7 | CPU ICache = Enabled, CPU DCache = Enabled (aman: tidak ada DMA) |

### GPIO (klik pin → pilih fungsi → klik kanan → *Enter User Label*)
| Pin | Mode | Setting | **User Label** |
|---|---|---|---|
| PA4 | GPIO_Output | Output level **High**, push-pull, speed High | `CAN1_CS` |
| PC4 | GPIO_Input | **Pull-up** | `CAN1_INT` |
| PD8 | GPIO_Output | Output level **High**, push-pull, speed High | `CAN2_CS` |
| PD9 | GPIO_Input | **Pull-up** | `CAN2_INT` |

Label wajib persis seperti itu. `app_config.h` membaca `CAN1_CS_Pin` dkk. dari `main.h` dan akan `#error` kalau label tidak ada.

### Connectivity
**SPI1** dan **SPI2**: Mode = *Full-Duplex Master*, Hardware NSS = *Disable*.

| | SPI1 | SPI2 |
|---|---|---|
| SCK | **PB3** | **PA9** |
| MISO | **PA6** | **PB14** |
| MOSI | **PA7** | **PC1** |

CubeMX sering menaruh SPI1_SCK default di PA5. Klik **PB3** → pilih `SPI1_SCK` supaya pindah. Parameter (sama untuk keduanya):

| Parameter | Nilai |
|---|---|
| Frame Format / Data Size / First Bit | Motorola / **8 Bits** / MSB First |
| Prescaler | **16** → 100 MHz / 16 = **6.25 Mbit/s** (MCP2515 maks 10) |
| CPOL / CPHA | **Low / 1 Edge** (SPI mode 0) |
| NSS Signal Type | Software |
| **Master Keep IO State** | **Enabled** (SCK tidak floating di antara transfer) |

**USB_OTG_HS**: Mode = **Internal FS Phy** → **Device_Only**. Pin PA11/PA12 otomatis.
- Parameter: Speed = Full Speed 12 Mbit/s, **VBUS sensing = Disabled** (WeAct tidak menyambung VBUS ke MCU), DMA = Disabled, SOF = Disabled.

**USART2** (opsional, untuk printf debug): Asynchronous 115200, TX **PA2**, RX **PD6**. Firmware tidak membutuhkannya.

### Timers
**TIM6**: Activated.
- Prescaler **199**, Counter Period **999**. Timer clock 200 MHz / 200 / 1000 = **1 kHz**
- NVIC Settings: centang **TIM6 global interrupt**

### Middleware
**USB_DEVICE** → Class For HS IP = **Communication Device Class (Virtual Port Com)**. Parameter biarkan default (VID 0x0483 / PID 0x5740 → driver Windows otomatis).

## 3. Clock Configuration

```
HSE 25 MHz ─► PLL Source Mux = HSE
             DIVM1 /5  = 5 MHz
             DIVN1 x160 = 800 MHz (VCO)
               ├─ DIVP1 /2 = 400 MHz ─► System Clock Mux = PLLCLK ─► CPU 400 MHz
               ├─ DIVQ1 /8 = 100 MHz ─► SPI123 Clock Mux = PLL1Q  (SPI1, SPI2)
               └─ DIVR1 /2 = 400 MHz
HSI48 (enable) ─────────────────────► USB Clock Mux = HSI48 (48 MHz)
```

| Prescaler | Nilai | Hasil |
|---|---|---|
| D1CPRE | /1 | CPU 400 MHz |
| HPRE | /2 | AXI/AHB 200 MHz |
| D1PPRE, D2PPRE1, D2PPRE2, D3PPRE | /2 | APB1-4 100 MHz (timer clock 200 MHz) |

Kenapa 400 MHz, bukan 550: tetap di VOS1, tanpa urusan overdrive. Untuk loop CAN ini sudah lebih dari cukup.

Pastikan **tidak ada kotak merah** di Clock Configuration. Kalau USB merah, berarti HSI48 belum di-enable (RCC → HSI48) atau USB mux belum ke HSI48. Kalau ada opsi **CRS** (RCC → CRS SYNC Source = USB), aktifkan supaya HSI48 di-trim oleh SOF USB.

## 4. Generate code, lalu integrasi

### 4a. Copy file
Copy **isi** `App/Inc/*.h` ke `Core/Inc/` dan `App/Src/*.c` ke `Core/Src/` di project CubeIDE. Ini cara paling simpel: tanpa setting include path, dan aman saat CubeMX re-generate (file buatan kita tidak dihapus).

### 4b. `Core/Src/main.c` (hanya di dalam blok USER CODE)
```c
/* USER CODE BEGIN Includes */
#include "app.h"
/* USER CODE END Includes */
...
  /* USER CODE BEGIN 2 */
  app_init();
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    app_loop();
  }
  /* USER CODE END 3 */
```

### 4c. `USB_DEVICE/App/usbd_cdc_if.c`
```c
/* USER CODE BEGIN INCLUDE */
#include "usb_link.h"
/* USER CODE END INCLUDE */
...
static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 11 */          // nomor blok bisa beda, yang penting di dalam fungsi ini
  usb_link_rx_isr(Buf, *Len);       // <-- tambahkan baris ini
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);
  return (USBD_OK);
  /* USER CODE END 11 */
}
```
Kalau CubeMX kamu menamai fungsinya `CDC_Receive_FS` / `CDC_Transmit_FS`, ubah juga `APP_CDC_TRANSMIT` di `app_config.h`.

### 4d. Build & flash
STM32CubeIDE: Build (palu) → Run/Debug (ST-Link). Header SWD WeAct: 3V3, DIO, CLK, GND. Kalau gagal connect, coba Debug Configuration → Debugger → Reset behaviour = *Software system reset*, atau trik BOOT0.

## 5. Live Expressions (STM32CubeIDE → Debug → Window → Show View → Live Expressions)

| Expression | Isi |
|---|---|
| `g_live` | `pos_deg[4]`, `speed_rpm[4]`, `torque_nm[4]`, `temp_c[4]`, `online[4]` (satuan enak dibaca) |
| `g_motor` | struct per motor: `pos` (rad), `vel` (rad/s), `torque`, `temp`, `iq`, `vbus`, `fault`, `can_id` |
| `g_app` | `can_init_err[2]`, `loop_count`, `ctrl_max_us`, `ctrl_overruns` |
| `g_can[0].mcp.st` | health CAN1: `tx_sent`, `tx_fail`, `rx_count`, `rx_ovf`, `tec`, `rec`, `bus_off` |

`g_app.can_init_err`: **0** = OK · **-1** = SPI error · **-2** = chip tidak menjawab (tidak masuk config mode: cek CS/MISO/power) · **-3** = readback register salah (wiring/level/kecepatan SPI) · **-4** = gagal masuk normal mode.

## 6. Bring-up: 2 motor per bus = CAN ID harus beda

Motor baru biasanya punya ID yang sama (sering 127). Dua motor dengan ID sama di satu bus tidak bisa dibedakan, jadi **ganti ID satu per satu**:

```powershell
cd python
# 1. Colok HANYA motor A ke CAN1, lalu lihat ID-nya
python rs05_tests.py -p COM8 info
# misal ketemu 127 -> beri tahu firmware bahwa M1 sekarang 127, lalu ganti ke 1
python rs05_tests.py -p COM8 --ids 127 2 1 2 setid --motor 1 --new 1
# 2. Colok motor B (masih 127) di CAN1 juga -> jadikan 2
python rs05_tests.py -p COM8 --ids 1 127 1 2 setid --motor 2 --new 2
# 3. Ulangi untuk CAN2 (M3 -> 1, M4 -> 2)
# 4. Power-cycle motor, cek dengan info. Kalau ID kembali ke 127, jalankan:
python rs05_tests.py -p COM8 save -m 1 2 3 4
```
⚠️ Frame "set ID" (type 7) dan "save" (type 22) diambil dari manual RobStride seri lain dan ditandai **VERIFY** di `robstride.h`. Kalau tidak berhasil, ganti ID pakai software resmi RobStride (sekali saja), lalu isi `APP_MOTOR_IDS` di `app_config.h` atau pakai `--ids`.

Setelah ID beres: `zero` → `pos` → `vel-sweep` → `torque`.

## 7. Struktur kode (HAL, bare metal)

```
TIM6 IRQ 1 kHz ── app_tick_isr()            (cuma naikin counter)
USB IRQ ───────── CDC_Receive_HS → usb_link_rx_isr()   (cuma copy byte)
main loop: app_loop()
  ├─ can_bus_poll(0/1)      CAN RX → robstride.c decode → g_motor[]
  ├─ command_poll()         byte USB → link parser → command.c → motor_ctrl / calib
  ├─ mc_tick() @ 500 Hz     mode tiap motor, safety, g_live, telemetry (100 Hz) + health (5 Hz)
  ├─ can_bus_health_update  TEC/REC/EFLG @ 10 Hz
  └─ usb_link_poll()        kirim antrean ke CDC_Transmit_HS
```

| File | Isi |
|---|---|
| `app.c/.h` | init + superloop + statistik waktu (`g_app`) |
| `app_config.h` | **semua yang spesifik board**: jumlah motor, ID default, rate, kristal MCP2515, handle SPI/TIM |
| `spi_bus.c/.h` | SPI + CS software |
| `mcp2515.c/.h` | driver chip: init/identifikasi, bit timing, TX one-shot, RX, error flags |
| `can_bus.c/.h` | 2 bus: send/ordered send, RX poll → decode, health, **scan ID** |
| `robstride.c/.h` | protokol RobStride + tabel `g_motor[4]` |
| `motor_ctrl.c/.h` | mode IDLE/MIT/VEL/POS/CALIB, soft limit, watchdog, suhu, fault, `g_live` |
| `calib.c/.h` | **zeroing**, homing hard-stop, ganti CAN ID, save |
| `command.c/.h` | decode command biner dari Python/MATLAB |
| `telemetry.c/.h` | telemetry, health, event, ACK |
| `usb_link.c/.h` | FIFO USB RX/TX |
| `link.c/.h`, `protocol.h` | framing + CRC16 dan kontrak protokol v2 |

**Menuju FreeRTOS nanti:** tiap baris di `app_loop()` jadi task (`can_rx` prio tinggi dipicu EXTI pin INT, `control` prio tinggi dipicu TIM6, `comm` prio sedang, `usb_tx` prio rendah). HAL timebase harus pindah ke TIM7 (FreeRTOS memakai SysTick). Saat itu CubeMX membuat `HAL_TIM_PeriodElapsedCallback` di `main.c`: hapus versi di `app.c` dan panggil `app_tick_isr()` dari sana.
