September 2020
# ARM-Based IoT-Enabled Patient Monitor

As part of the NUS module EE2028 Microcontroller Programming and Interfacing, this project uses the ARM Cortex M4 chip to design an IoT-Enabled Patient Monitor meant for use in hospitals to remotely monitor critical patients. Real-time data is streamed to a mobile dashboard called "COPEMON".

![Dashboard](Mobile_Dashboard.png)

## Data Measured

The sensors used are:

### what does 3 hash look like?

1. Accelerometer - To simulate the posture of the patient, specifically for fall detection.
2. Magnetometer - To simulate the orientation of the patient lying on the bed. Proper orientation is important in ensuring that other monitoring / life-saving equipment remains connected properly.
Pressure Sensor - To simulate the pressure of air in the patient's lungs.
Temperature Sensor - To simulate body temperature.
Humidity Sensor - To simulate the relative humidity of the air passed into the patient's lungs.
Gyroscope - To simulate patient's movement; specifically to sense the patient's sudden twisting/twitching which could be an indication that the patient is in pain. A sudden movement can also cause issues with the other monitoring / life-saving equipment such as ventilators

The data collected from the sensors are sent to a Blynk.io server, and then displayed on a phone application:

Mobile Dashboard:

![Dashboard](Mobile_Dashboard.png)
