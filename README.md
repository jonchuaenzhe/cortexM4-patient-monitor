September 2020
# ARM-Based IoT-Enabled Patient Monitor

As part of the NUS module EE2028 Microcontroller Programming and Interfacing, this project uses the ARM Cortex M4 chip to design an IoT-Enabled Patient Monitor meant for use in hospitals to remotely monitor critical patients. Real-time data is streamed to a mobile dashboard called "COPEMON".

![Dashboard](Mobile_Dashboard.png)

The STM32L4 kit was used for this implementation. All code is detailed in main.c, additional libraries are standard STM32 libraries.

## Data Measured

The sensors used are:
1. Accelerometer (LSM6DSL): To observe the patient's posture, especially useful for fall detection.
2. Magnetometer (LIS3MDL) - To observe the orientation of the patient lying on the bed to ensure that life-saving equipment remains connected properly.
3. Barometer (LPS22HB) - To measure the pressure of air in the patient's lungs to monitor lung function.
4. Temperature Sensor (HTS221) - To measure body temperature.
5. Humidity Sensor (HTS221) - To measure the relative humidity of the air passed into the patient's lungs to ensure that it is not too dry.
6. Gyroscope (LSM6DSL) - To sense the patient's sudden twisting which could be an indication that the patient is in pain.

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