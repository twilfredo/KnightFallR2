## Dev Notes - Wilfred MK
## 01-08-2021 : 
Testing the D Cell battery pack, USB stack required to be disabled for the modem to function as per usual
                further testing required. However, testing battery pack with an RTL shell/log seems to work fine. 
## 04-09-2021 : 
    Tested with 6 x AA Energizer Max Plus for 2 hours.                                                                                              OK
    Reduced LED Duty Cycles to lower concurrent power draw.                                                                                         OK
    Added PMIC current limit modifier and disabled charging.                                                                                        OK
    Added GPS turning on at initial boot, to speed up cold start lock (Increases boot time current draw ~140mA Peak, ~60mA avg)                     OK
    Added GUI interface for on the go debugging                                                                                                     OK
    

