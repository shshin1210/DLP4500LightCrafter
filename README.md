# DLP4500LightCrafter

This repository provides python code Python USB controller for the TI DLP LightCrafter 4500. The main purpose is to use high speed pattern sequence mode, projecting patterns from pre-loaded patterns in firmware. Please relate to documents for pre-loading patterns in firmware.

The core program is borrowed from SivyerLab/pyCrafter4500 developed by [SivyerLab](https://github.com/SivyerLab/pyCrafter4500). I have modified the original python code, adding functions needed for pattern sequence mode. I acknowledge and respect his pioneering work.

Check the documents and code to modify more...

- Code adapted from [TI Lightcrafter 4500](https://github.com/csi-dcsc/Pycrafter6500)

- The DLPC 350 is the controller chip on the LCR 4500.

- [TI DLPC 350 documentation](http://www.ti.com/product/DLPC350/technicaldocuments) (DLPC350 Programmer’s Guide (Rev. G))

# Install and Usage

Follow the Install and Usage part introduced in [SivyerLab](https://github.com/SivyerLab/pyCrafter4500).

# Pattern sequence mode

Run ```main.py``` to test pattern sequence mode.

I have added following functions to implement pattern sequence mode. Check the description for each functions in ```pycrafter4500.py```

- set_pattern_trigger_mode
  
- set_led_current

- set_led_pwm_polarity

- set_led_control_mode
 
- send_pattern_image_index

- pattern_sequence_final
- 
