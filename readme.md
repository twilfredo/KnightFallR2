# T Wilfred MK - River Drifer [UQ THESIS] <br />

## Project Abstract <br />
The aim of this project is to create a real-time flood monitoring drifter in association
with the UQ Aquatic Research Group. These drifters would be used during flood events
to closely monitor channel erosion (in South East Queensalnd) and the movement of
soil from catchment areas down to Morton Bay.

The monitoring data provided by the drifter such as turbidity and location, is crucial
for deploying good mitigation strategies.

## TCP Listening Guide<br />

The home router has port forwarding active to the desktops local IP address on port 6969.

Drifter connects to the public address of the modem, and sends data to port 6969. 

To listen to the data sent by the modem:
> ~$ nc -l 4011 <br />

<br />