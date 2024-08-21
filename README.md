# nfc_ndef_read
Read NDEF-info from NFC type A/B/F/V tags.

Using PlatformIO/Arduino w. ESP32-S3 Mini-M1 kit and "ST25R3911B-DISCO" kit.

NOTE: configured to read NFC-A/B/F/V tags (ISO14443(A) etc.).



# Setup

"config.h" contains the GPIO definitions for the SPI.
Set to work w. ESP32-S3 Mini-M1 kit connected to SPI-header on ST25R3911B-DISCO kit.
Can also work with the X-NUCLEO-NFC05A1 add-on kit.

The RFAL-lib (from STmicro) can be configured in the "src/rfal_platform/rfal_platform.h" file, 
for specific NFC functionality.

# ST RFAL implementation

- src/rfal_core
- src/rfal_core/st25r3911
- src/rfal_platform

# ST NDEF implementation

- src/ndef
- src/ndef/polling
- src/ndef/message

# Debug Output:

Each step returns the NFC lib error code.
The UID represent the unique ID of the NFC device.
NDEF type and content is printed on console.

```text
...

```

