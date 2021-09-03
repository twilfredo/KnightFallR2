import urllib.request
import requests
import threading
import json
import webbrowser
import threading
import time

from tkinter.ttk import Progressbar

from tkinter import *

#ThingSpeak IO Access 
URL='https://api.thingspeak.com/channels/1416495/fields/1.json?timezone=Australia/Brisbane&results=10'
TIME='timezone=Australia/Brisabne'
#Drifter Online Indicator
drifterOnlineStat = None
#Google Track URL
GOOGLE_LOC_URL = 'https://www.google.com/maps/search/?api=1&query='


#Drifter Parsed String Buffer
dataPoints = 10
createdAt = ["" for x in range(dataPoints)]
ntus = ["" for x in range(dataPoints)]
long = ["" for x in range(dataPoints)]
latt = ["" for x in range(dataPoints)]
entry_id = ["" for x in range(dataPoints)]

#Thread to poll data from ThingSpeak IO
def data_loop():
    
    oldPacketNum = 0
    newPacketNum = 0
        
    while True:
        recv_data=requests.get(URL).json()
        #print(recv_data)
        channel_id=recv_data['channel']['id']

        field_1=recv_data['feeds']  #Data Stream
        listLen = (len(field_1)) - 1 #-1 for array indexing
        
        loopCount = 0

        for x in field_1:
            
            if loopCount == listLen:
                newPacketNum = x['entry_id'] 
                #Last iteration of loop
                if newPacketNum <= oldPacketNum:
                    #No new data points received
                    #print("Drifter: No new data points...")
                    drifterOnlineStat = None; 
                else:
                    drifterOnlineStat = True;
                    
                oldPacketNum = x['entry_id']               
                   
            #Parse time into simpler format
            timeX = (x['created_at']) 
            timeY = timeX.split('T')         
            createdAt[loopCount] = timeY[0] + '  ' + timeY[1].split('+')[0]

            dbuf = (x['field1'].split('##'))

            ntus[loopCount] = (dbuf[0].split(':')[1])
            latt[loopCount] = (dbuf[1].split(':')[1])
            long[loopCount] = (dbuf[2].split(':')[1])
            entry_id[loopCount] = str(x['entry_id'])
     
            loopCount += 1
        
        # print(createdAt)
        # print(ntus)
        # print(latt)
        # print(long)
        #Sleep for a little bit and loop again. 
        time.sleep(5)



def google_maps_cb():
    
    last_lat = (latt[dataPoints-1])
    last_lon = (long[dataPoints-1])
    webbrowser.open_new(GOOGLE_LOC_URL + str(last_lat) + ',' + str(last_lon))

#Thread to draw the GUI
def gui_loop():
    window = Tk()
    window.title("UQ Drifter - Receiver")
    window.config(bg='#2c3e50')
    window.geometry("820x535")
    
    header_lab = Label(
        window, text="Previously received data", bg='#2c3e50', fg='#16a085', font=('Arial', 20)
    )
    
    
    last_lab = Label(
        window, text="Last update", bg='#2c3e50', fg='#16a085', font=('Arial', 20)
    )
    
    find_me_button = Button(
        window, text="Locate Device", command=google_maps_cb, bg ='#1c2833', fg='white'
    )    
    
    connection_status = Label(
            window, text="Drifter Status:", bg='#2c3e50', fg='#581845', font=('Arial', 15)
    )
     
    
    
    
    disp_ent1 = Entry(
        window, 
        width=85,
        font=('Arial', 12),
        bg='#5d6d7e',
        fg="white"
    )

    disp_ent2 = Entry(
        window, 
        width=85,
        font=('Arial', 12),
        bg='#5d6d7e',
        fg="white"
    )

    disp_ent3 = Entry(
        window, 
        width=85,
        font=('Arial', 12),
        bg='#5d6d7e',
        fg="white"
    )
    
    disp_ent4 = Entry(
        window, 
        width=85,
        font=('Arial', 12),
        bg='#5d6d7e',
        fg="white"
    )
    
    disp_ent5 = Entry(
        window, 
        width=85,
        font=('Arial', 12),
        bg='#5d6d7e',
        fg="white"
    )
    
    disp_ent6 = Entry(
        window, 
        width=85,
        font=('Arial', 12),
        bg='#5d6d7e',
        fg="white"
    )
    
    disp_ent7 = Entry(
        window, 
        width=85,
        font=('Arial', 12),
        bg='#5d6d7e',
        fg="white"
    )

    disp_ent8 = Entry(
        window, 
        width=85,
        font=('Arial', 12),
        bg='#5d6d7e',
        fg="white"
    )
    
    disp_ent9 = Entry(
        window, 
        width=85,
        font=('Arial', 12),
        bg='#5d6d7e',
        fg="white"
    )
    
    disp_ent10 = Entry(
        window, 
        width=85,
        font=('Arial', 12),
        bg='#5d6d7e',
        fg="white"
    )
    
    last_lab.pack(pady=3)
    disp_ent10.pack(pady=5)
    
    find_me_button.pack(pady=5)
    
    
    header_lab.pack(pady=3)   
    disp_ent9.pack(pady=5)
    disp_ent8.pack(pady=5)
    disp_ent7.pack(pady=5)
    disp_ent6.pack(pady=5) 
    disp_ent5.pack(pady=5)
    disp_ent4.pack(pady=5)
    disp_ent3.pack(pady=5)
    disp_ent2.pack(pady=5)
    disp_ent1.pack(pady=5)
    
    connection_status.pack(pady=3)
    
    p = Progressbar(window,orient=HORIZONTAL,length=60,mode="determinate",takefocus=True,maximum=50)
    p.pack()  

    space = '       '
    while(True):
        
                       
        p.step()            
        window.update()
        #Clear Old Data
        disp_ent1.delete(0, 'end')
        disp_ent2.delete(0, 'end')
        disp_ent3.delete(0, 'end')
        disp_ent4.delete(0, 'end')
        disp_ent5.delete(0, 'end')
        disp_ent6.delete(0, 'end')
        disp_ent7.delete(0, 'end')
        disp_ent8.delete(0, 'end')
        disp_ent9.delete(0, 'end')
        disp_ent10.delete(0, 'end')
        
        #Update New Data, last index of the array is the most recent data
        disp_ent1.insert(0, 'eID:' + entry_id[0] + space + createdAt[0] + space + 'NTU: ' + ntus[0] + space + 'Lattitude: ' + latt[0] + space + 'Longitude: ' + long[0])
        disp_ent2.insert(0, 'eID:' + entry_id[1] + space + createdAt[1] + space + 'NTU: ' + ntus[1] + space + 'Lattitude: ' + latt[1] + space + 'Longitude: ' + long[1])
        disp_ent3.insert(0, 'eID:' + entry_id[2] + space + createdAt[2] + space + 'NTU: ' + ntus[2] + space + 'Lattitude: ' + latt[2] + space + 'Longitude: ' + long[2])
        disp_ent4.insert(0, 'eID:' + entry_id[3] + space + createdAt[3] + space + 'NTU: ' + ntus[3] + space + 'Lattitude: ' + latt[3] + space + 'Longitude: ' + long[3])
        disp_ent5.insert(0, 'eID:' + entry_id[4] + space + createdAt[4] + space + 'NTU: ' + ntus[4] + space + 'Lattitude: ' + latt[4] + space + 'Longitude: ' + long[4])
        disp_ent6.insert(0, 'eID:' + entry_id[5] + space + createdAt[5] + space + 'NTU: ' + ntus[5] + space + 'Lattitude: ' + latt[5] + space + 'Longitude: ' + long[5])
        disp_ent7.insert(0, 'eID:' + entry_id[6] + space + createdAt[6] + space + 'NTU: ' + ntus[6] + space + 'Lattitude: ' + latt[6] + space + 'Longitude: ' + long[6])
        disp_ent8.insert(0, 'eID:' + entry_id[7] + space + createdAt[7] + space + 'NTU: ' + ntus[7] + space + 'Lattitude: ' + latt[7] + space + 'Longitude: ' + long[7])
        disp_ent9.insert(0, 'eID:' + entry_id[8] + space + createdAt[8] + space + 'NTU: ' + ntus[8] + space +'Lattitude: ' + latt[8] + space + 'Longitude: ' + long[8])
        disp_ent10.insert(0, 'eID:' + entry_id[9] + space + createdAt[9] + space + 'NTU: ' + ntus[9] + space +'Lattitude: ' + latt[9] + space + 'Longitude: ' + long[9])
        
        #Set Connection Status
        if (drifterOnlineStat):
           connection_status.config(text="Drifter Status: Online", fg='#DFFF00')
        else:
            connection_status.config(text="Drifter Status: Offline", fg="red")          
        

            
        
        window.update()
        time.sleep(0.1)
    
#Set and start threads
thread_dataLoop = threading.Thread(target=data_loop, args=())
thread_guiLoop = threading.Thread(target=gui_loop, args=())

thread_guiLoop.start()
thread_dataLoop.start()


