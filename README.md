# Cansat_AlphaSpot

## Official 
program, run on the arduino Feather m0 Express, that 	takes the measurements. 
- controls the sensors (BMP280 and MPU6050
- stores the data of the sensors on a SD card
- retrieves gps data (GPS breakout v3)
- sends GPS coordinates through the radio module
## receiver
program, run on a arduino uno, that retrieves the gps
coordinates send by the arduino Feather.
- Antenna: a 7 element yagi antenna with the dipole connected to a radio module
## live gps
program, run on a pc, that takes the gps coordinates that are retrieved from the antenna and places them in google earth in wich you can then see the live location of the cansat
## raket
makes a simulation with the gps coordinates in blender to precisely see the path of the cansat in 3D