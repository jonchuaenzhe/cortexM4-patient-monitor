September 2020
# ARM-Based IoT-Enabled Patient Monitor

As part of the NUS module EE2028 Microcontroller Programming and Interfacing, this project uses the ARM Cortex M4 chip to design an IoT-Enabled Patient Monitor for hospitals, where real-time data is streamed to a mobile dashboard.

![Dashboard](Mobile_Dashboard.png)

The STM32L4 kit was used for this implementation. All code is detailed in main.c, additional functions are called from standard STM32 drivers.

## Data Measured

The sensors used are:
1. LSM6DSL: Accelerometer (for fall detection) and Gyroscope (detection of patient's sudden twisting)
2. LIS3MDL: Magnetometer (observe patient orientation)
3. HTS221: Temperature Sensor and Humidity Sensor (measure humidity of breath)
4. LPS22HB: Barometer (measure lung air pressure)

## Cloud Connection

The data collected from the sensors are sent to a cloud server via WiFi, and then displayed on a phone application. This was doing using the ISM43362-M3G-L44 module from Inventek Systems.

WiFi Setup:
```
WiFi_Stat = WIFI_Init();
WiFi_Stat &= WIFI_Connect(WiFi_SSID, WiFi_password, WiFi_security);
if(WiFi_Stat!=WIFI_STATUS_OK) while(1);
WiFi_Stat = WIFI_GetHostAddress(SERVER_NAME, server_ipaddr);
```

WiFi Transmission:
```
WiFi_Stat = WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, DEST_PORT, SOURCE_PORT);
sprintf((char*)req, "GET /rvXK2x3GtPfvJnaNbHA4ln50v2unophS/update/V2?value=%d HTTP/1.1\r\nHost: blynk-cloud.com\r\n\r\n", operation_mode);
WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
WiFi_Stat = WIFI_CloseClientConnection(1);
```