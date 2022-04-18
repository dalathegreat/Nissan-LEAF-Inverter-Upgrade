# Nissan-LEAF-Inverter-Upgrade
Software for extracting more power from LEAF inverter upgrades. Enables 110kW and 160kW inverters to function with the older LEAF

## Disclaimer !
Tuning electric vehicles can be very dangerous. I take no responsibility (see license) for any damages that this firmware might cause. You are entirely on your own when using this software. Respect your local legislation.

## How to use ?
Now that we got that out of the way, here's what you need to get started.
- A 2013-2017 Nissan LEAF (AZE0 model)
- A newer inverter from a 2018+ ZE1 LEAF (see this video for installation: https://www.youtube.com/watch?v=VGBA8VPWwIg )
- Leafspy Pro to code in the new resolver offset (see this video for howto: https://www.youtube.com/watch?v=K0v269B0xqo )
- A CAN-bridge with the firmware in this repository flashed onto it. The CAN-bridge attaches anywhere between the VCM and INV on EV-CAN. 
- Pre-compiled .hex files are available in the "debug" folder. One for 110kW and one for 160kW. Flashing instructions: https://youtu.be/eLcNSo2Vn6U?t=167

## FAQ
- Q: Why doesn't this work on my 2011-2012 LEAF? A: The early leaf has EM61 motors. These aren't compatible with the EM57 inverter style.
- Q: What is resolver offset coding? A: See this video for more info: https://www.youtube.com/watch?v=Of2vCYgblY4

## Part numbers
Here are the part numbers so you know what to get when ordering inverters
ZE1 inverters (2018-2022)
- 291A0-5SA1A - 110kW inverter
- 291A0-5SA1B - 110kW inverter
- 291A0-5SA0A - 110kW inverter
- 291A0-5SN0A - 160kW inverter
- 291A0-5SN1A - 160kW inverter

22-pin Yazaki wiring part number: 7283-8750-30
Buy link: https://nl.aliexpress.com/item/1005003344398420.html?spm=a2g0s.9042311.0.0.25994c4doh13qv

Temperature sensor (no link yet, just use butt-crimp connectors)

Don't get these, just listed so you know what they are.
AZE0 inverters (2013-2017)
291A0-3NF1B - 80kW 
291A0-3NF1A - 80kW
291A0-3NF0A - 80kW
ZE0 inverters (2011-2012)
291A0-3NA0A - 80kW

## Example installation pics and wiring info
Here's where the CAN-bridge attaches, behind the glovebox on the left connector going into the VCM. The EV-CAN is a twisted pair green/blue cable. Cut it and install the CAN-bridge wires there.
The other two wires that the CAN-bridge needs is +12V constant(red wire) and GND(black wire). +12V constant can be taken from many places, e.g. OBD2port.

![name-of-you-image](https://github.com/dalathegreat/Nissan-LEAF-Inverter-Upgrade/blob/main/CAN-bridge1.jpeg)
![name-of-you-image](https://github.com/dalathegreat/Nissan-LEAF-Inverter-Upgrade/blob/main/CAN-bridge2.jpeg)
![name-of-you-image](https://github.com/dalathegreat/Nissan-LEAF-Inverter-Upgrade/blob/main/CAN-bridge3.jpeg)

Here are the differences in wiring. Please note that one wire is different color, all the rest matches!
![name-of-you-image](https://github.com/dalathegreat/Nissan-LEAF-Inverter-Upgrade/blob/main/Wiring.png)
![name-of-you-image](https://github.com/dalathegreat/Nissan-LEAF-Inverter-Upgrade/blob/main/Wiring2.png)
