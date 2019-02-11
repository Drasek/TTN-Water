TTN-Water
=========
The Things Network / LoRaWAN Water Level Detection

Hardware list:
--------------
- PCB from this store https://www.tinytronics.nl/shop/nl/communicatie/rf(id)-wifi-bt/things-network-lorawan-node-workshop-2017
- Arduino Pro Mini 3.3V (remove Power-LED to save power)
- HopeRF RFM95W 868MHz
- some headers and wires
- MAXBOTIX MB7363 HRXL-MAXSONAR-WRLS
- MAXBOTIX HR-MAXTEMP PCB
- MAXBOTIX MB7950 MOUNTING HARDWARE
- 6600mAh accu with JST connector
- RP1065 IP65 enclosure with mounting kit MF0041

![Hardware opened](https://raw.githubusercontent.com/Freie-Netzwerker/TTN-Water/master/images/ttn_water001.jpg "Hardware opened")

![Hardware programming](https://raw.githubusercontent.com/Freie-Netzwerker/TTN-Water/master/images/ttn_water002.jpg "Hardware programming")

![Hardware with accu](https://raw.githubusercontent.com/Freie-Netzwerker/TTN-Water/master/images/ttn_water003.jpg "Hardware with accu")

![Hardware assembled](https://raw.githubusercontent.com/Freie-Netzwerker/TTN-Water/master/images/ttn_water004.jpg "Hardware assembled")

![Hardware assembled](https://raw.githubusercontent.com/Freie-Netzwerker/TTN-Water/master/images/ttn_water005.jpg "Hardware assembled")

![Dataconnection to TTN](https://raw.githubusercontent.com/Freie-Netzwerker/TTN-Water/master/images/ttn_water006.jpg "Dataconnection to TTN")

Software
--------

The device joins OTAA to the LoRaWAN / The Things Network and transmitts every 15min the actual data (distance read from serial). If the data is "0" or "500" it will repeat the reading because that will be a error (the sensor range from 500 to 9999mm). If the distance is changing to fast (a hound swimming under die sensor, or a canoe at the time of reading) the node will switch into a fast reading sequence reading and transmitting every 5min till the distance change is inside the normal behaviour again.