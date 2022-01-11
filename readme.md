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

The drifter once it has established a network LTE-M1 connection will slowly blink green (status led), if the device is not connected
or is attemting to connect, it will blink the red status led. 


## Data viewing <br />
### Data viewing <br />
The drifter will stream data to a public channel on ThingSpeak using MQTT. Due to the bandwidth limit restrictions imposed by thingspeak IO (Packet/15 Seconds), the drifter sends a '#' delimited string of data of the following format [turbidity#lattitude#longitude].  This can be viewed at the following link:

>https://thingspeak.com/channels/1416495

Clicking on 'export recent data', then 'JSON' is the easiest way of watching the data received onto the server. 

### GUI <br />
The output stream from the drifter can also be viewed using the GUI, it shows a recent history (last 10 updates) from the drifter and the ability to locate the device based on its last known location using google maps (click the locate button).

### Deployment Results <br />

The drifter was deployed on the 1/12/2021 and was successfully recovered on the 2/12/2021. The following images show deployment adventures of the drifter!

![image](https://user-images.githubusercontent.com/36925352/148931658-9dc4be07-5994-477d-bb8f-62e7b48bced9.png)

![image](https://user-images.githubusercontent.com/36925352/148931574-7093fadb-25af-4d9a-a38d-ab5d96dfe263.png)

![image](https://user-images.githubusercontent.com/36925352/148931792-c4fbe9ed-3eac-4f0a-aa4a-36516d186fbc.png)

### Drifter Poster <br />
![2021 ITEE Innovation Showcase Poster Templates](https://user-images.githubusercontent.com/36925352/148932114-dd61cc2b-0f69-4a2b-9b30-ab8555b7bb4d.jpg)

<br />
