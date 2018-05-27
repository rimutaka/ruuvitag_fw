"C:\Program Files (x86)\Nordic Semiconductor\nrf5x\bin\nrfutil.exe" settings generate --family NRF52 --application .\ruuvi_examples\ruuvi_firmware\ruuvitag_b\s132\armgcc\_build\ruuvi_firmware.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 settings.hex
mergehex -m .\nRF5_SDK_12.3.0_d7731ad\components\softdevice\s132\hex\s132_nrf52_3.0.0_softdevice.hex .\bootloader\ruuvitag_b_debug\armgcc\_build\ruuvitag_b_bootloader.hex settings.hex -o sbc.hex
mergehex -m sbc.hex .\ruuvi_examples\ruuvi_firmware\ruuvitag_b\s132\armgcc\_build\ruuvi_firmware.hex -o packet.hex
nrfjprog --family nrf52 --eraseall
nrfjprog --family nrf52 --program packet.hex -r
pause
