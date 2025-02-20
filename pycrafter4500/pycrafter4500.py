from __future__ import print_function

import time
from contextlib import contextmanager
from math import floor

import usb.core
import usb.util
from usb.core import USBError
from usb.backend import libusb1
import libusb_package
    
"""
Adapted for lcr4500 from https://github.com/csi-dcsc/Pycrafter6500

DLPC350 is the controller chip on the LCR4500.

Docs: http://www.ti.com/lit/ug/dlpu010f/dlpu010f.pdf
Doc strings adapted from dlpc350_api.cpp source code.

To connect to LCR4500, install libusb-win32 driver. Recommended way to do is this is
with Zadig utility (http://zadig.akeo.ie/)
"""

__author__  = 'Alexander Tomlinson'
__email__   = 'mexander@gmail.com'
__version__ = '0.7'


def conv_len(a, l):
    """
    Function that converts a number into a bit string of given length.

    :param int a: Number to convert.
    :param int l: Length of bit string.

    :return: Padded bit string.
    """
    b = bin(a)[2:]
    padding = l - len(b)
    b = '0' * padding + b
    return b


def bits_to_bytes(a, reverse=True):
    """
    Function that converts bit string into a given number of bytes.

    :param str a: Bytes to convert.
    :param bool reverse: Whether or not to reverse the byte list.

    :return: List of bytes.
    """
    bytelist = []

    # check if needs padding
    if len(a) % 8 != 0:
        padding = 8 - len(a) % 8
        a = '0' * padding + a

    # convert to bytes
    for i in range(len(a) // 8):
        bytelist.append(int(a[8 * i:8 * (i + 1)], 2))

    if reverse:
        bytelist.reverse()
    return bytelist


def fps_to_period(fps):
    """
    Calculates desired period (us) from given fps.

    :param int fps: Frames per second.

    :return: Period (us).
    """
    period = int(floor(1.0 / fps * 10**6))
    return period


@contextmanager
def connect_usb():
    """
    Context manager for connecting to and releasing usb device.

    :yields: USB device.
    """
    ## debug ##
    backend = libusb1.get_backend(find_library=libusb_package.find_library)

    vendor_id = 0x0451 # 0x2708
    product_id = 0x6401 # 0x0003

    device = usb.core.find(
                idVendor=vendor_id,
                idProduct=product_id,
                backend=backend,
            )
    if device is None:
        raise RuntimeError("DLP4500 device not found. Check USB connection")
    else:
        for cfg in device:
            for intf in cfg:
                for ep in intf:
                    print(f"Endpoint: {ep.bEndpointAddress}, Attributes: {ep.bmAttributes}")
       
    # device = usb.core.find(idVendor=0x0451, idProduct=0x6401)
    device.set_configuration()

    lcr = dlpc350(device)

    yield lcr

    device.reset()
    del lcr
    del device


class dlpc350(object):
    """
    Class representing dmd controller. Can connect to different DLPCs by changing product ID. Check IDs in
    device manager.
    """
    def __init__(self, device):
        """
        Connects the device.

        :param device: lcr4500 USB device.
        """
        self.dlpc = device

    def command(self,
                mode,
                sequence_byte,
                com1,
                com2,
                data=None):
        """
        Sends a command to the dlpc.

        :param str mode: Whether reading or writing.
        :param int sequence_byte:
        :param int com1: Command 1
        :param int com2: Command 3
        :param list data: Data to pass with command.
        """

        buffer = []

        if mode == 'r':
            flagstring = 0xc0  # 0b11000000
        else:
            flagstring = 0x40  # 0b01000000

        data_len = conv_len(len(data) + 2, 16)
        data_len = bits_to_bytes(data_len)

        buffer.append(flagstring)
        buffer.append(sequence_byte)
        buffer.extend(data_len)
        buffer.append(com2)
        buffer.append(com1)

        # if data fits into single buffer, write all and fill
        if len(buffer) + len(data) < 65:
            for i in range(len(data)):
                buffer.append(data[i])

            # append empty data to fill buffer
            for i in range(64 - len(buffer)):
                buffer.append(0x00)

            self.dlpc.write(1, buffer)

        # else, keep filling buffer and pushing until data all sent
        else:
            for i in range(64 - len(buffer)):
                buffer.append(data[i])

            self.dlpc.write(1, buffer)
            buffer = []

            j = 0
            while j < len(data) - 58:
                buffer.append(data[j + 58])
                j += 1

                if j % 64 == 0:
                    self.dlpc.write(1, buffer)
                    buffer = []

            if j % 64 != 0:
                while j % 64 != 0:
                    buffer.append(0x00)
                    j += 1

                self.dlpc.write(1, buffer)

        # wait a bit between commands
        time.sleep(0.02)

        # done writing, read feedback from dlpc
        try:
            self.ans = self.dlpc.read(0x81, 64)
        except USBError as e:
            print('USB Error:', e)

        time.sleep(0.02)

    def read_reply(self):
        """
        Reads in reply.
        """
        for i in self.ans:
            print(hex(i))

    def get_main_status(self, pretty_print=False):
        """The Main Status command shows the status of DMD park and DLPC350 sequencer, frame buffer, and gamma
         correction.

         (USB: CMD2: 0x02, CMD3: 0x0C)
         """
        self.command('r', 0x00, 0x1a, 0x0c, [])
        if pretty_print:
            # ans = str(bin(self.ans[4]))[2:]
            ans = format(self.ans[4], '08b')
            print(f'\nDMD micromirrors are {"parked" if int(ans[-1]) else "not parked"}')
            print(f'Sequencer is {"running normally" if int(ans[-2]) else "stopped"}')
            print(f'Frame buffer is {"frozen" if int(ans[-3]) else "not frozen"}')
            print(f'Gamma correction is {"enabled" if int(ans[-4]) else "disabled"}')

    def set_power_mode(self, do_standby=False):
        """
        The Power Control places the DLPC350 in a low-power state and powers down the DMD interface. Standby mode should
        only be enabled after all data for the last frame to be displayed has been transferred to the DLPC350. Standby
        mode must be disabled prior to sending any new data.

        (USB: CMD2: 0x02, CMD3: 0x00)

        :param bool do_standby:
            :True: Standby mode. Places DLPC350 in low power state and powers down the DMD interface.
            :False: Normal operation. The selected external source will be displayed.
        """
        do_standby = int(do_standby)
        self.command('w', 0x00, 0x02, 0x00, [do_standby])

    def start_pattern_lut_validate(self):
        """
        This API checks the programmed pattern display modes and indicates any invalid settings. This command needs to
        be executed after all pattern display configurations have been completed.

        (USB: CMD2: 0x1A, CMD3: 0x1A)
        """
        self.command('w', 0x00, 0x1a, 0x1a, bits_to_bytes(conv_len(0x00, 8)))

    def set_dmd_park(self, park=False):
        """
        This command is used to park or unpark the DMD, whenever system is idle user can send this command to park the
        DMD.

        (USB: CMD2: 0x06, CMD3: 0x09)

        :param bool park: Whether to park the dmd mirrors
        """
        park = int(park)
        self.command('w', 0x00, 0x06, 0x09, [park])

    def set_buffer_freeze(self, freeze=False):
        """
        The Display Buffer Freeze command disables swapping the memory buffers.

        (USB: CMD2: 0x10, CMD3: 0x0A)

        :param bool park: Whether to park the dmd mirrors
        """
        park = int(freeze)
        self.command('w', 0x00, 0x10, 0x0a, [freeze])

    def set_display_mode(self, mode='pattern'):
        """
        Selects the input mode for the projector.

        (USB: CMD2: 0x1A, CMD3: 0x1B)

        :param int mode:
            :0: "video" mode
            :1: "pattern" mode
        """
        modes = ['video', 'pattern']
        if mode in modes:
            mode = modes.index(mode)

        self.command('w', 0x00, 0x1a, 0x1b, [mode])

    def set_pattern_input_source(self, mode='flash'):
        """
        Selects the input source.

        (USB: CMD2: 0x1A, CMD3: 0x00)

        :param int mode:
            :0: "parallel interface with 8-bit"
            :1: "internal test pattern"
            :2: "Flash, images are 24-bit singel-frame"
        """
        modes = ['parallel', 'internal', 'flash']
        if mode in modes:
            mode = modes.index(mode)

        # Hard coding
        self.command('w', 0x00, 0x1a, 0x00, [mode])

    def set_pattern_display_data_input_source(self, mode='flash'):
        """
        The Pattern Display Data Input Source command selects the source of the data for pattern display

        (USB: CMD2: 0x1A, CMD3: 0x22)

        :param int mode:
            :0: "video"
            :3: "flash"
        """
        modes = ['video', '', '', 'flash']
        if mode in modes:
            mode = modes.index(mode)

        self.command('w', 0x00, 0x1a, 0x22, [mode])
        
    def set_pattern_trigger_mode(self, mode='pattern_trigger_mode1'):
        """
        Selects the trigger type for pattern sequence.

        (USB: CMD2: 0x1A, CMD3: 0x23)

        :param int mode:
            :0: "vsync"
            "1": "pattern trigger mode1" : internally or externally generated trigger
        """
        modes = ['vsync', 'pattern_trigger_mode1']
        if mode in modes:
            mode = modes.index(mode)

        self.command('w', 0x00, 0x1a, 0x23, [mode])
   
    def set_led_current(self, red: int, green: int, blue: int):
        """
        Sets the LED current for the Red, Green, and Blue LEDs using an 8-bit PWM value (0-255).

        (USB: CMD2: 0x0B, CMD3: 0x01)

        :param int red: Red LED current (0-255).
        :param int green: Green LED current (0-255).
        :param int blue: Blue LED current (0-255).
        """

        # Ensure values are within 0-255 range
        red = max(0, min(255, red))
        green = max(0, min(255, green))
        blue = max(0, min(255, blue))

        # Send command with 3-byte data payload
        self.command('w', 0x00, 0x0B, 0x01, [red, green, blue])
    
    def set_led_pwm_polarity(self, polarity: int):
        """
        Sets the PWM polarity for the LEDs.
        
        :param int polarity: 0 for normal polarity, 1 for inverted polarity.
        """
        polarity = max(0, min(1, polarity))  # Ensure the value is 0 or 1
        self.command('w', 0x00, 0x1A, 0x05, [polarity])
        
    def set_led_control_mode(self, automatic: bool, red: bool = True, green: bool = True, blue: bool = True):
        """
        Sets the LED selection mode to Automatic (controlled by sequencer) or Manual, 
        and enables/disables individual LEDs.

        (USB: CMD2: 0x1A, CMD3: 0x07)

        :param bool automatic: 
            - True: Automatic mode (sequencer controls LEDs).
            - False: Manual mode (controlled by software via bits 2:0).
        :param bool red: Enable Red LED (default: True).
        :param bool green: Enable Green LED (default: True).
        :param bool blue: Enable Blue LED (default: True).
        """
        red_bit = 1 if red else 0
        green_bit = 1 if green else 0
        blue_bit = 1 if blue else 0
        mode_bit = 1 if automatic else 0  # Automatic or manual control mode

        # Construct the byte with bits (R, G, B, Mode)
        led_control_byte = (mode_bit << 3) | (blue_bit << 2) | (green_bit << 1) | red_bit

        # Send the command
        self.command('w', 0x00, 0x1A, 0x07, [led_control_byte])

    def set_gamma_correction(self, apply_gamma=True):
        """
        This command only works in video mode.

        Because the DMD is inherently linear in response, the Gamma Correction command specifies the removal of the
        gamma curve that is applied to the video data at the source. Two degamma tables are provided: TI Video
        (Enhanced) and TI Video (Max Brightness).

        (USB: CMD2: 0x1A, CMD3: 0x0E)

        :param bool apply_gamma: Whether to apply gamma correction while in video mode.
        """
        if apply_gamma:
            apply_gamma = '10000000'
        else:
            apply_gamma = '00000000'
        self.command('w', 0x00, 0x1a, 0x0e, bits_to_bytes(apply_gamma))

    def pattern_display(self, action='start'):
        """
        This API starts or stops the programmed patterns sequence.

        (USB: CMD2: 0x1A, CMD3: 0x24)

        :param int action: Pattern Display Start/Stop Pattern Sequence

            :0: Stop Pattern Display Sequence. The next "Start" command will restart the pattern sequence from the
               beginning.
            :1: Pause Pattern Display Sequence. The next "Start" command will start the pattern sequence by
               re-displaying the current pattern in the sequence.
            :2: Start Pattern Display Sequence.
        """
        actions = ['stop', 'pause', 'start']
        if action in actions:
            action = actions.index(action)

        self.command('w', 0x00, 0x1a, 0x24, [action])

    def set_exposure_frame_period(self,
                                  exposure_period,
                                  frame_period):
        """
        The Pattern Display Exposure and Frame Period dictates the time a pattern is exposed and the frame period.
        Either the exposure time must be equivalent to the frame period, or the exposure time must be less than the
        frame period by 230 microseconds. Before executing this command, stop the current pattern sequence. After
        executing this command, call ``DLPC350_ValidatePatLutData()`` API before starting the pattern sequence.

        (USB: CMD2: 0x1A, CMD3: 0x29)

        :param int exposure_period: Exposure time in microseconds (4 bytes).
        :param int frame_period: Frame period in microseconds (4 bytes).
        """
        exposure_period = conv_len(exposure_period, 32)
        frame_period = conv_len(frame_period, 32)

        payload = frame_period + exposure_period
        payload = bits_to_bytes(payload)

        self.command('w', 0x00, 0x1a, 0x29, payload)

    def set_pattern_config(self,
                           num_lut_entries=3,
                           do_repeat=True,
                           num_pats_for_trig_out2=3,
                           num_images=0):
        """
        This API controls the execution of patterns stored in the lookup table. Before using this API, stop the current
        pattern sequence using ``DLPC350_PatternDisplay()`` API. After calling this API, send the Validation command
        using the API DLPC350_ValidatePatLutData() before starting the pattern sequence.

        (USB: CMD2: 0x1A, CMD3: 0x31)

        :param int num_lut_entries: Number of LUT entries.
        :param bool do_repeat:

            :True: Execute the pattern sequence once.
            :False: Repeat the pattern sequence.

        :param int num_pats_for_trig_out2: Number of patterns to display(range 1 through 256). If in repeat mode, then
           this value dictates how often TRIG_OUT_2 is generated.

        :param int num_images: Number of Image Index LUT Entries(range 1 through 64). This Field is irrelevant for Pattern
            Display Data Input Source set to a value other than internal.
        """
        num_lut_entries = '0' + conv_len(num_lut_entries - 1, 7)
        do_repeat = '0000000' + str(int(do_repeat))
        num_pats_for_trig_out2 = conv_len(num_pats_for_trig_out2 - 1, 8)
        num_images = '00' + conv_len(num_images, 6)

        payload = num_images + num_pats_for_trig_out2 + do_repeat + num_lut_entries
        payload = bits_to_bytes(payload)

        self.command('w', 0x00, 0x1a, 0x31, payload)

    def mailbox_set_address(self, address=0):
        """
        This API defines the offset location within the DLPC350 mailboxes to write data into or to read data from.

        (USB: CMD2: 0x1A, CMD3: 0x32)

        :param int address: Defines the offset within the selected (opened) LUT to write/read data to/from (0-127).
        """
        address = bits_to_bytes(conv_len(address, 8))
        self.command('w', 0x00, 0x1a, 0x32, address)

    def open_mailbox(self, mbox_num):
        """
        This API opens the specified Mailbox within the DLPC350 controller. This API must be called before sending data
        to the mailbox/LUT using DLPC350_SendPatLut() or DLPC350_SendImageLut() APIs.

        (USB: CMD2: 0x1A, CMD3: 0x33)

        :param mbox_num:
            :0: Disable (close) the mailboxes.
            :1: Open the mailbox for image index configuration.
            :2: Open the mailbox for pattern definition.
            :3: Open the mailbox for the Variable Exposure.
        """
        mbox_num = bits_to_bytes(conv_len(mbox_num, 8))
        self.command('w', 0x00, 0x1a, 0x33, mbox_num)

    def send_pattern_image_index(self, image_indices):
        """
        Writes image indices to the Pattern Display LUT Data (USB: CMD2: 0x1A, CMD3: 0x34).

        :param list image_indices: List of image indices (0-based) to be used in pattern sequence.
        """

        # Convert image indices to a bit string
        bit_string = ''.join(conv_len(index, 8) for index in image_indices)

        # Convert the bit string to a byte list
        payload = bits_to_bytes(bit_string)

        # Send the converted payload
        self.command('w', 0x00, 0x1A, 0x34, payload)
        
        # payload = bytes(image_indices)
        # self.command('w', 0x00, 0x1A, 0x34, payload)
    
    def send_pattern_lut(self,
                         trig_type,
                         pat_num,
                         bit_depth,
                         led_select,
                         do_invert_pat=False,
                         do_insert_black=False,
                         do_buf_swap=True,
                         do_trig_out_prev=False):
        """
        Mailbox content to setup pattern definition. See table 2-65 in programmer's guide for detailed description of
        pattern LUT entries.

        (USB: CMD2: 0x1A, CMD3: 0x34)

        :param int trig_type: Select the trigger type for the pattern

            :0: Internal trigger.
            :1: External positive.
            :2: External negative.
            :3: No Input Trigger (Continue from previous; Pattern still has full exposure time).
            :0x3FF: Full Red Foreground color intensity

        :param int pat_num: Pattern number (0 based index). For pattern number ``0x3F``, there is no pattern display. The
            maximum number supported is 24 for 1 bit-depth patterns. Setting the pattern number to be 25, with a
            bit-depth of 1 will insert a white-fill pattern. Inverting this pattern will insert a black-fill pattern.
            These patterns will have the same exposure time as defined in the Pattern Display Exposure and Frame Period
            command. Table 2-66 in the programmer's guide illustrates which bit planes are illuminated by each pattern
            number.

        :param bit_depth: Select desired bit-depth

            :0: Reserved
            :1: 1-bit
            :2: 2-bit
            :3: 3-bit
            :4: 4-bit
            :5: 5-bit
            :6: 6-bit
            :7: 7-bit
            :8: 8-bit

        :param int led_select: Choose the LEDs that are on (bit flags b0 = Red, b1 = Green, b2 = Blue)

            :0: 0b000 No LED (Pass through)
            :1: 0b001 Red
            :2: 0b010 Green
            :3: 0b011 Yellow (Green + Red)
            :4: 0b100 Blue
            :5: 0b101 Magenta (Blue + Red)
            :6: 0b110 Cyan (Blue + Green)
            :7: 0b111 White (Blue + Green + Red)

        :param bool do_invert_pat:
            :True: Invert pattern.
            :False: Do not invert pattern.

        :param bool do_insert_black:
            :True: Insert black-fill pattern after current pattern. This setting requires 230 us of time before the
               start of the next pattern.
            :False: Do not insert any post pattern.

        :param bool do_buf_swap:
            :True: perform a buffer swap.
            :False: Do not perform a buffer swap.

        :param do_trig_out_prev:
            :True: Trigger Out 1 will continue to be high. There will be no falling edge between the end of the
               previous pattern and the start of the current pattern. Exposure time is shared between all patterns
               defined under a common trigger out). This setting cannot be combined with the black-fill pattern.
            :False: Trigger Out 1 has a rising edge at the start of a pattern, and a falling edge at the end of the
               pattern.

        """
        # byte 0
        trig_type = conv_len(trig_type, 2)
        pat_num = conv_len(pat_num, 6)

        byte_0 = pat_num + trig_type

        # byte 1
        bit_depth = conv_len(bit_depth, 4)
        led_select = conv_len(led_select, 4)

        byte_1 = led_select + bit_depth

        # byte 2
        do_invert_pat = str(int(do_invert_pat))
        do_insert_black = str(int(do_insert_black))
        do_buf_swap = str(int(do_buf_swap))
        do_trig_out_prev = str(int(do_trig_out_prev))

        byte_2 = '0000' + do_trig_out_prev + do_buf_swap + do_insert_black + do_invert_pat

        payload = byte_2 + byte_1 + byte_0
        payload = bits_to_bytes(payload)

        self.command('w', 0x00, 0x1a, 0x34, payload)

def pattern_sequence_final(input_mode='pattern',
                            input_source='flash',
                            num_pats = 8,
                            trigger_type='pattern_trigger_mode1',
                            exposure_period = 100000,
                            frame_period = 100000,
                            bit_depth= [1,1,1,1,1,1,1,1],
                            led_color= 0b111,  # BGR
                            trig_type = 0,
                            **kwargs
                            ):

    
    with connect_usb() as lcr:    
    
        # 1. Stop Pattern Display Before Configuring
        lcr.pattern_display('stop')
        time.sleep(0.2)

        # 2. Set Display Mode to Pattern
        lcr.set_display_mode(input_mode)
        
        # 3. Set LED Configurations
        lcr.set_led_pwm_polarity(1)  # Ensures that PWM=0 means LED OFF
        # Set LED currents (from image settings)
        lcr.set_led_current(red=100, green=100, blue=100)
        # Set LED selection mode (manual selection)
        lcr.set_led_control_mode(automatic=False, red=True, green=True, blue=True)
        
        # 4. Set Pattern Input Source (Before LUT Configuration)
        lcr.set_pattern_input_source(input_source)
        # Set pattern input source to flash
        lcr.set_pattern_display_data_input_source(input_source)
        
        # 5. Set Pattern Configuration
        lcr.set_pattern_config(num_lut_entries=num_pats,
                               num_pats_for_trig_out2=num_pats,
                               num_images=num_pats,
                               do_repeat = True)

        # 6. Set Pattern Trigger Mode
        lcr.set_pattern_trigger_mode(trigger_type)

        # 7. Set Exposure and Frame Period
        lcr.set_exposure_frame_period(exposure_period=exposure_period, frame_period=frame_period)

        # 8. Configure Image Index LUT (Open Mailbox)
        lcr.open_mailbox(1)  # Open mailbox for image index configuration
        lcr.mailbox_set_address(0)
        lcr.send_pattern_image_index([7, 6, 5, 4, 3, 2, 1, 0])  # Send indices uploaded in Firmware

        lcr.open_mailbox(0)  # Close mailbox

        # 9. Apply Image LUT Validation
        lcr.command('w', 0x00, 0x1A, 0x1A, [0x00])  # Image LUT validation

        # 10. Configure Pattern LUT
        lcr.open_mailbox(2)
        for i in range(num_pats):
            lcr.mailbox_set_address(i)
            lcr.send_pattern_lut(trig_type=trig_type,
                                 pat_num=i,
                                 bit_depth=bit_depth[i],
                                 led_select=led_color,
                                 do_invert_pat=False,
                                 do_insert_black=False,
                                 do_buf_swap=True,
                                 do_trig_out_prev=False)
        lcr.open_mailbox(0)
        
        # 11. Validate Pattern LUT
        lcr.start_pattern_lut_validate()
        time.sleep(2)
        
        # 10: start sequence
        lcr.pattern_display('start')
        time.sleep(3)
        
def video_mode():
    """
    Puts LCR4500 into video mode.
    """
    with connect_usb() as lcr:
        lcr.pattern_display('stop')
        lcr.set_display_mode('video')

def power_down():
    """
    Puts LCR4500 into standby mode.
    """
    with connect_usb() as lcr:
        lcr.pattern_display('stop')
        # lcr.set_power_mode(do_standby=True)

def power_up():
    """
    Wakes LCR4500 up from standby mode.
    """
    with connect_usb() as lcr:
        lcr.set_power_mode(do_standby=False)

def set_gamma(value):
    """
    Sets gamma.
    """
    with connect_usb() as lcr:
        lcr.set_gamma_correction(apply_gamma=value)
        lcr.get_main_status(True)
