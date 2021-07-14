# T Wilfred MK - River Drifer [UQ THESIS] <br />

## Project Abstract <br />
The aim of this project is to create a real-time flood monitoring drifter in association
with the UQ Aquatic Research Group. These drifters would be used during flood events
to closely monitor channel erosion (in South East Queensalnd) and the movement of
soil from catchment areas down to Morton Bay.

The monitoring data provided by the drifter such as turbidity and location, is crucial
for deploying good mitigation strategies.

## Device Interaction <br />
The device is equipped with a shell and logging through USB. It can be accessed over micro usb with any PC, on linux the use the following to open a session
>screen /dev/ttyACMX 115200

Shell allows for specific commands to be issued to the device for debugging/testing purposes. Logging will show any crucial errors/issues. 

## Modes / Status LED <br />

The drifter once it has established a network LTE-M1 connection will slowly blink purple (status led), if the device is not connected
or is attemting to connect, it will blink red at a faster rate. 


## Data viewing <br />

The drifter will stream data to a public channel on ThingSpeak using MQTT. Due to the bandwidth limit restrictions imposed by thingspeak IO (Packet/15 Seconds), the drifter sends a '#' delimited string of data of the following format [turbidity#lattitude#longitude].  This can be viewed at the following link:

>https://thingspeak.com/channels/1416495

Clicking on 'export recent data', then 'JSON' is the easiest way of watching the data received onto the server. 

<br />