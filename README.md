# TlsrComProg
TLSR825x programmator via COM port

(!) Limitations: Only TLSR8251, TLSR8253, TLSR8258 Chip Used!

### Telink SWIRE simulation on a COM port + COM Flasher.

Flash programming for TLSR825x chips using only a COM port.
(modules TB-02, TB-04, ...)

![SCH](https://github.com/pvvx/TlsrComProg825x/blob/main/Doc/img/schematic.gif)

    Usage: TlsrComProg [-h] [--port PORT] [--tact TACT] [--fldr FLDR]
                       [--baud BAUD]
                       {rf,we,wf,es,ea} ...
    
    TLSR825x Floader version 10.11.20
    
    positional arguments:
      {rf,we,wf,es,ea}      Run TlsrComProg {command} -h for additional help
        rf                  Read Flash to binary file
        we                  Write file to Flash with sectors erases
        wf                  Write file to Flash without sectors erases
        es                  Erase Region (sectors) of Flash
        ea                  Erase All Flash
    
    optional arguments:
      -h, --help            show this help message and exit
      --port PORT, -p PORT  Serial port device (default: COM1)
      --tact TACT, -t TACT  Time Activation ms (0-off, default: 600 ms)
      --fldr FLDR, -f FLDR  Filename floader (default: floader.bin)
      --baud BAUD, -b BAUD  UART Baud Rate (default: 230400)


------------

#### Samples:
> **Write Flash:** python.exe TlsrComProg.py -p COM10 -t 600 we 0 AiThinkerAT.bin

```
================================================
TLSR825x Floader version 10.11.20
------------------------------------------------
Open COM10, 230400 baud...
Reset module (RTS low)...
Activate (600 ms)...
Connection...
Load <floader.bin> to 0x40000...
Bin bytes writen: 1912
CPU go Start...
------------------------------------------------
ChipID: 0x5562 (TLSR8253), Floader ver: 1.0
Flash JEDEC ID: c86013, Size: 512 kbytes
------------------------------------------------
Inputfile: AiThinkerAT.bin
Write Flash data 0x00000000 to 0x00012328...
------------------------------------------------
Done!
```
> **Read Flash:** python.exe TlsrComProg.py -p COM10 rf 0 0x80000 ff.bin
```
================================================
TLSR825x Floader version 10.11.20
------------------------------------------------
Open COM10, 230400 baud...
Reset module (RTS low)...
Activate (600 ms)...
Connection...
Load <floader.bin> to 0x40000...
Bin bytes writen: 1912
CPU go Start...
------------------------------------------------
ChipID: 0x5562 (TLSR8253), Floader ver: 1.0
Flash JEDEC ID: c86013, Size: 512 kbytes
------------------------------------------------
Read Flash from 0x000000 to 0x080000...
Outfile: ff.bin
------------------------------------------------
Done!
```
> **Erase All Flash:** python.exe TlsrComProg.py -p COM10 ea
```
================================================
TLSR825x Floader version 10.11.20
------------------------------------------------
Open COM10, 230400 baud...
Reset module (RTS low)...
Activate (600 ms)...
Connection...
Load <floader.bin> to 0x40000...
Bin bytes writen: 1912
CPU go Start...
------------------------------------------------
ChipID: 0x5562 (TLSR8253), Floader ver: 1.0
Flash JEDEC ID: c86013, Size: 512 kbytes
------------------------------------------------
Erase All Flash ...
------------------------------------------------
Done!
```

[Examples of using](https://github.com/pvvx/TlsrComProg825x/tree/master/Doc)

[UartFloader Source Code](https://github.com/pvvx/TlsrComProg825x/tree/master/Uartfloader)


