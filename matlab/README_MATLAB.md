# MATLAB / Simulink untuk firmware HAL (protocol v2, 4 motor)

Kebutuhan: MATLAB **R2022a+**. Simulink hanya untuk model. Tidak butuh STM32 blockset.

| File | Isi |
|---|---|
| `RS05Link.m` | Library serial: `ping`, `scan`, `getHealth`, `pos/vel/mit/disable`, kalibrasi `zeroHere/home/setCanId/saveParams`, `setMotorId`, history, record CSV |
| `RS05App.m` | GUI: pilih M1..M4 (checkbox), CAN ID per motor, tab MIT/Velocity/Position, zeroing, ganti CAN ID, soft limit, E-STOP, 4 plot live (torsi, speed, posisi, suhu) × 4 motor, tabel + CAN health |
| `RS05Block.m` + `build_rs05_model.m` | Simulink (interpreted). Input `mask` sekarang bit mask (1 M1, 2 M2, 4 M3, 8 M4, 15 semua), output 4×1 |
| `example_script.m` | Tes posisi versi script |

```matlab
app = RS05App;                    % GUI
build_rs05_model("COM8")          % Simulink
L = RS05Link("COM8"); L.zeroHere(RS05Link.mask([1 2 3 4]))   % zero semua via calib.c
```

Catatan:
- Motor diberi nomor 1..4: M1, M2 di CAN1, M3, M4 di CAN2.
- Kolom CAN ID di GUI harus sama dengan ID asli motor (lihat `L.scan(3)`).
- Satu COM port hanya bisa dipakai satu program (GUI / Simulink / Python).
