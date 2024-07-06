#  Motus Test Tag Simulater
This code will alternately transmit a CTT tag and a series of LOTEK tag pulses to test and verify a Motus receiver Station. 

#  base hardware: 
        Adafruit Feather M0 RFM96 LoRa Radio - 433MHz - RadioFruit Product ID: 3179
#  antenna used: 
        TWAYRDIO 15.4-Inch Whip Antenna, SMA Male Antenna, Dual Band 2m/70cm although smaller dual band antenna would be more appropriate for a deployed model
    
  this code will alternatively transmit a CTT test tag followed by 6 pulses of a lotek tag to facilitate 
  verification of the working status of a sensor gnome station at a more cost friendly price point.

## Note
The Lotek test tag needs to be added to the tag database in the Sensorgnome station in order for it to be recognized as a tag
