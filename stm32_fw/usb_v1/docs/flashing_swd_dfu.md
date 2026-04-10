# Flashing and Debug (ST-LINK SWD + USB DFU)

Use SWD as the default path. Use DFU as fallback.

## A. ST-LINK SWD (Primary)

Prerequisites:

- ST-LINK connected to SWDIO, SWCLK, GND, and target power/reference
- BOOT0 in normal boot mode (from flash)

Steps:

1. Open CubeIDE and select `Release` build.
2. `Run` -> `Debug Configurations` -> `Ac6 STM32 Debugging`.
3. Select ST-LINK probe and target.
4. Flash and run.
5. Verify firmware starts and drives neutral PWM at boot.

Optional CLI (STM32CubeProgrammer):

```powershell
STM32_Programmer_CLI.exe -c port=SWD -w .\build\usb_v1.bin 0x08000000 -v -rst
```

## B. USB DFU (Fallback)

Use only if SWD is unavailable.

Prerequisites:

- board supports USB DFU on your STM32 part
- BOOT0/RESET procedure documented in `pinmap.md`

Steps:

1. Put MCU into DFU boot mode (BOOT0 high then reset, board-specific).
2. Connect USB and confirm DFU device is detected.
3. Flash with STM32CubeProgrammer GUI or CLI.
4. Restore normal boot mode (BOOT0 low), reset board.

CLI example:

```powershell
STM32_Programmer_CLI.exe -c port=USB1 -w .\build\usb_v1.bin 0x08000000 -v -rst
```

## Post-Flash Verification

- Device boots without debugger attached.
- PWM outputs are neutral at startup.
- USB command parser accepts valid `CMD,...` lines.
- Watchdog timeout forces neutral if command stream stops.
