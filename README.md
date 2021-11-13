# Nissan-LEAF-Inverter-Upgrade
Software for extracting more power from LEAF inverter upgrades. Enables 110kW and 160kW inverters to function with the older LEAF

## Disclaimer !
Tuning electric vehicles can be very dangerous. I take no responsibility (see license) for any damages that this firmware might cause. You are entirely on your own when using this software. Respect your local legislation.

## How to use ?
Now that we got that out of the way, here's what you need to get started.
- A 2013-2017 Nissan LEAF (AZE0 model)
- A newer inverter from a 2018+ ZE1 LEAF (see this video for installation: https://www.youtube.com/watch?v=VGBA8VPWwIg )
- A CAN-bridge with the firmware in this repository flashed onto it. The CAN-bridge attaches anywhere between the VCM and INV on EV-CAN. 

## FAQ
- Q: Why doesn't this work on my 2011-2012 LEAF? A: The early leaf has EM61 motors. These aren't compatible with the EM57 inverter style.
- Q: Resoler coding not available in Leafspy yet! What do? A: Manual resolver coding instructions here: https://openinverter.org/forum/viewtopic.php?p=31056

## Part numbers
Here are the part numbers so you know what to get when ordering inverters
ZE1 inverters (2018-2022)
- 291A0-5SA1A - 110kW inverter
- 291A0-5SA1B - 110kW inverter
- 291A0-5SA0A - 110kW inverter
- 291A0-5SN0A - ???
- 291A0-5SN1A - 160kW inverter

Don't get these, just listed so you know what they are.
AZE0 inverters (2013-2017)
291A0-3NF1B - 80kW 
291A0-3NF1A - 80kW
291A0-3NF0A - 80kW
ZE0 inverters (2011-2012)
291A0-3NA0A - 80kW

## Example installation pics and wiring info
Here's where the CAN-bridge attaches, behind the glovebox on the left connector going into the VCM.

![name-of-you-image](https://github.com/dalathegreat/Nissan-LEAF-Inverter-Upgrade/blob/main/CAN-bridge1.jpeg)
![name-of-you-image](https://github.com/dalathegreat/Nissan-LEAF-Inverter-Upgrade/blob/main/CAN-bridge2.jpeg)
![name-of-you-image](https://github.com/dalathegreat/Nissan-LEAF-Inverter-Upgrade/blob/main/CAN-bridge3.jpeg)

Here are the differences in wiring. Please note that one wire is different color, all the rest matches!
![name-of-you-image](https://github.com/dalathegreat/Nissan-LEAF-Inverter-Upgrade/blob/main/Wiring.png)
![name-of-you-image](https://github.com/dalathegreat/Nissan-LEAF-Inverter-Upgrade/blob/main/Wiring2.png)
