# Dual axis solar controller
[Raspberry Pi 3b+](https://www.raspberrypi.org/products/raspberry-pi-3-model-b-plus/) 

[HAT-MDD10](http://www.cytron.com.my/p-HAT-MDD10) is a dual DC motor driver board, HAT stackable design to be used with a Raspberry Pi.

[ADS1115](https://wiki.seeedstudio.com/4-Channel_16-Bit_ADC_for_Raspberry_Pi-ADS1115/) is a 4-channel analog-to-digital converter(ADC) based on Texas Instrument's ADS1115

[DS3231](https://www.amazon.com/dp/B07RYT1KLH) optional Real-Time clock needed when Raspberry Pi is not network connected

this project uses time of day to determine what orientation the sun is at for a configured lat / long, 
by using the [pvlib library](https://github.com/pvlib/pvlib-python),
this application then independantly controls each motor, elevation and / or azimuth to move them to stay in alignment throughout the day.
Each minute the sun position is compared to a configured error threshold to determine if repositioning is needed.
The application uses the [tkinter](https://docs.python.org/3/library/tkinter.html) GUI library.

DC linear actuators were chosen for the motors and also ones that contain a potentiometer (POT) to allow the system to understand the position of the actuators
by feeding the POT wiper into the ADC.
[example actuator](https://www.amazon.com/gp/product/B00NVI7NA8/)

the application also allows for monitoring wind speed to use as a safety feature to tip the elevation into a horizontal position when winds exceed a configured threshold.
[Anemometer](https://www.adafruit.com/product/1733)
