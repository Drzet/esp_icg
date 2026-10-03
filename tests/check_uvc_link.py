#!/usr/bin/env python3
"""Fail CI if an integration hook was silently omitted from the linked firmware."""
import subprocess
symbols = subprocess.check_output(['riscv32-esp-elf-nm', 'build/esp_icg.elf'], text=True)
names = {line.split()[-1] for line in symbols.splitlines() if line.split()}
required = {'__wrap_videod_control_xfer_cb', '__wrap_tud_descriptor_configuration_cb',
            '__wrap_tud_descriptor_device_cb', 'icg_ipa_controls_begin', 'icg_ipa_controls_end'}
missing = required - names
assert not missing, f'UVC integration hooks absent from ELF: {sorted(missing)}'
print('PASS: UVC request/descriptor wrappers and manual-ISP ownership hook linked')
