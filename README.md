| Supported Targets | ESP32 |
| ----------------- | ----- |

## ESP-IDF Bluetooth PPG PCG machine

This project is firmware for machine that measure PPG PCG signal from sensors and send data to phone and computer through bluetooth protocol using the APIs of **Serial Port Protocol** (**SPP**) 

## Structure of project
```
project
    |--component
    |    |---esp_idf_lib_helpers
    |    |---FileManager: manipulate sd card (Nguyen Nhu Hai Long, SEEE, HUST)
    |    |---i2cdev: working with i2c devices (Uncle Rus)
    |    |---MAX30102: driver of MAX30102 for esp-idf framework (Nguyen Doan Khanh, SEEE, HUST)
    |--main
    |    |---CMakeLists.txt
    |    |---main.c
    |--CMakeLists.txt
    |--ESP32_SSP,md
    |--README.md
    |--sdkconfig
```

### Hardware Required

This project is designed to run on commonly available ESP32 development board, e.g. ESP32-DevKitC. To operate the example, it should be connected to an SPP Initiator running on a smartphone or a computer

### Configure the project

1. Open the project configuration menu:

```
idf.py menuconfig
```

2. Enable the SPP functionality by choosing the path as following:

`Component config --> Bluetooth --> Bluedroid Options --> SPP`

3. If you want to limit the number of connected devices, please make sure set the `BT/BLE MAX ACL CONNECTIONS` and `BR/EDR ACL Max Connections` with same value you want.

`Component config --> Bluetooth --> Bluedroid Options --> BT/BLE MAX ACL CONNECTIONS(1~7)`
and
`Component config --> Bluetooth --> Bluetooth --> Bluetooth controller --> BR/EDR ACL Max Connections`


4. SSP is enabled as default in this example. If you prefer the legacy pairing, you can disable it in the following path.

`Component config --> Bluetooth--> Bluedroid Options --> Secure Simple Pair`.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Project Description

After the program starts, the machine will start to measure PPG signal by MAX30102 sensor, PCG signal by INMP441 digital microphone. While measuring, there is a task that receive data from sensor and then store them into SD card. After 20s, a timer will be triggered, measuring and saving data into SD tasks will be delete. At this time, the program will start an SPP acceptor. It will read data that saved into SD card before and send them to Android phone or computer which poerforms as the SPP initiator.


## FAQ
Q: Is this program measures data and send them through Bluetooth realtime ?
A: Unfortunately, no :). It's really hard to send PPG and PCG data through Bluetooth realtime because measuring PPG and PCG signal requires a high frequent task. Sending data through Bluetooth make these tasks much slower, and then we can lost samples (which is really not good). Maybe there are solutions that I've not find out. I'll try my best. But at the moment, it cannot be realtime unfortunately.

Below questions are about SPP (which I copied README.md from example of Espressif)
Q: How to change the process of SSP?
A: Users can set the IO Capability and Security Mask for their device (fixed Security Mode, Security Mode 4). In short, the Security Mask sets the security level for authentication stage and the IO Capability determines the way of user interaction during pairing. The default Security Mask of this demo is `ESP_SPP_SEC_AUTHENTICATE` which support MITM (Man In The Middle) protection. For more information about Security Simple Pair on ESP32, please refer to [ESP32_SSP](./ESP32_SSP.md).


Q: How many SPP servers does ESP32 support?
A: For now, the maximum number of SPP servers is 6, which is limited by the maximum number of SDP records. When the SPP server is successfully started, the unique SCN (Server Channel Number) will be mapped to the SPP server.

Q: Is SPP absolutely reliable?
A: For now, most Bluetooth stacks implement the SPP based on the L2CAP Basic Mode, and the reliability only depends on the controller. If errors(e.g. CRC) are missed by controller, the L2CAP packet will not be retransmitted, so it is recommended that you should implement the retransmission at the application layer. For more information about L2CAP operation modes, please refer to Bluetooth Core Specification, Version 4.2 or later, Volume 3, Part A: Logical Link Control and Adaptation Protocol Specification.# Bluetooth_PPG_PCG_Machine
