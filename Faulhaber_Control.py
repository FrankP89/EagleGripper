#!/usr/bin/env python

#################################
############ ARTC ###############
#################################
### Walter Frank Pintor Ortiz ###
#################################

import serial
import time
import socket

##########################################

def openFHPort():
    ser = serial.Serial('/dev/ttyUSB1',9600, timeout=1)
    return ser;

def closeFHPort(ser):
    ser.close()
    return;


##########################################
ser = openFHPort()
v = 500 #Test velocity
cpt = 0

### TCP Client service ###

#def commAsTCPClient():
    ## Under construction ##
    #TCP_IP = '192.168.127.1'
    #TCP_PORT = 5006
    #BUFFER_SIZE = 1024
    #MESSAGEreply = "Connected from Pi!"

    #s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #s.connect((TCP_IP, TCP_PORT))
    #s.send(MESSAGEreply)
    #data = s.recv(BUFFER_SIZE)
    #s.close()

    #print ("received data:", data)    
    
    #return;

#########################

### Activation/deactivation functions ###

def disableFHDrive(ser):
    #disableDriver = "DI\r"
    ser.write(bytes([68,73,13]))
    time.sleep(1)
    return;
    
def enableFHDrive(ser):
    #enableDriver = "EN\r"
    
    ser.write(bytes([69,78,13]))
    time.sleep(1)
    return;

#########################################


######## Type of motor #######

def isFHMotorConnected(ser):
    ser.flushInput()
    ser.flushOutput()
    
    ser.write(bytes([71,84,89,80]))
    print(ser.write(bytes([71,84,89,80])))
    gtyp = ser.read(12)
    print(gtyp)

    if (gtyp == ""):
        print("Motor not connected")
        ser.cancel_write()
        ser.flushInput()
        ser.flushOutput()
        time.sleep(1)
        return 0;
    else:
        print("Motor connected: ", gtyp)
        ser.cancel_write()
        ser.flushInput()
        ser.flushOutput()
        time.sleep(1)
        return 1;  

#############################


### Read/Write Parameters ###
    

def readFHCurrent(ser):
    ser.flushInput()
    ser.flushOutput()
    
    ser.write(bytes([71,82,67,13]))
    current = ser.readline()

    if (checkingOK(current) == True):
        ser.write(bytes([71,82,67,13]))
        current = ser.readline()
    
    ser.cancel_write()
    ser.flushInput()
    ser.flushOutput()
    time.sleep(1)     
    
    print("mA: ", current.decode("utf-8"))
    return current.decode("utf-8"); 

def readFHVelocity(ser):
    ser.flushInput()
    ser.flushOutput()
    
    ser.write(bytes([71,78,13]))
    velocity = ser.readline()

    if (checkingOK(velocity) == True):
        ser.write(bytes([71,78,13]))
        velocity = ser.readline()

    ser.cancel_write()
    ser.flushInput()
    ser.flushOutput()
    time.sleep(1) 
  
    print("RPMs: ",velocity.decode("utf-8"))
    return velocity.decode("utf-8"); 

def readFHPosition(ser):
    
    ser.flushInput()
    ser.flushOutput()
    
    ser.write(bytes([80,79,83,13]))
    position = ser.readline()
    
    if(checkingOK(position) == True):
        ser.write(bytes([80,79,83,13]))
        position = ser.readline()
    
    ser.cancel_write()
    ser.flushInput()
    ser.flushOutput()
    time.sleep(1)  
    print("Position: ", position.decode("utf-8"))    
    
    return position.decode("utf-8");

def readFHAll(ser):
    readFHPosition(ser)
    readFHVelocity(ser)
    readFHCurrent(ser)
    return;

def setFHVelocity(ser,num):
    
    vStr = str(num)
    #ser.write(b"1000")  
    ser.write(bytes([86]))      
    #After math operation
    ser.write(vStr.encode())
    
    ser.write(bytes([13,10,13]))
    #time.sleep(1)
    print("Velocity set: ", vStr) 
    return;

def setFHPosition(ser,num):
    pStr = str(num)
    ser.write(bytes([77]))      
    #After math operation
    ser.write(pStr.encode())
    
    ser.write(bytes([13,10,13]))
    #time.sleep(1)
    print("Position set: ", pStr) 
    
    return;

def checkingFHOK(okmsg):
    if (okmsg == bytes([79,75,13,10])):        
        result = True
    else:
        result = False
    return result;

def setFHMaxSpeed(speedlimit):
    if (speedlimit <= 0):
        speedlimit = 5000 #Default
    vStr = str(speedlimit)
    ser.write(bytes([83,80]))
    ser.write(vStr.encode())
    ser.write(bytes([13,10]))        
    return;

def getFHMaxSpeed(ser):
    ser.flushInput()
    ser.flushOutput()
    
    ser.write(bytes([71,83,80]))
    speedlimit = ser.readline()

    if (checkingOK(speedlimit) == True):
        ser.write(bytes([71,83,80]))
        speedlimit = ser.readline()
           
    print("OK")
    ser.cancel_write()
    ser.flushInput()
    ser.flushOutput()
    time.sleep(1)  
    print(speedlimit)
    
    return;

def setFHMaxAcceleration(ser,acc):
    if (acc <= 0):
        acc = 30000 #Default
    vStr = str(acc)
    ser.write(bytes([65,67]))
    ser.write(vStr.encode())
    ser.write(bytes([13,10]))        
    return;

def getFHMaxAcceleration(ser):
    ser.flushInput()
    ser.flushOutput()
    
    ser.write(bytes([71,65,67]))
    acc = ser.readline()
 
    if (checkingFHOK(acc) == True):
        ser.write(bytes([71,65,67]))
        acc = ser.readline()
           
    print("OK")
    ser.cancel_write()
    ser.flushInput()
    ser.flushOutput()
    time.sleep(1)  
    print(acc)
    
    return;


def setFHMaxDeceleration(ser,decel):
    if (decel <= 0):
        decel = 30000 #Default
    vStr = str(decel)
    ser.write(bytes([68,69,67]))
    ser.write(vStr.encode())
    ser.write(bytes([13,10]))        
    return;

def getFHMaxDeceleration(ser):
    ser.flushInput()
    ser.flushOutput()
    
    ser.write(bytes([71,68,69,67]))
    decel = ser.readline()
 
    if (checkingFHOK(decel) == True):
        ser.write(bytes([71,68,69,67]))
        decel = ser.readline()
           
    print("OK")
    ser.cancel_write()
    ser.flushInput()
    ser.flushOutput()
    time.sleep(1)  
    print(decel)
    
    return;

def setFHContinuousCurrent(ser,lcc):    
    if (lcc <= 0):
        lcc = 4800 #Default
    vStr = str(lcc)
    ser.write(bytes([76,67,67]))
    ser.write(vStr.encode())
    ser.write(bytes([13,10]))  
    return;

def getFHContinuousCurrent():
    ser.flushInput()
    ser.flushOutput()
    
    ser.write(bytes([71,67,67]))
    gcc = ser.readline()

    if (checkingFHOK(gcc) == True):
        ser.write(bytes([71,67,67]))
        gcc = ser.readline()
           
    print("OK")
    ser.cancel_write()
    ser.flushInput()
    ser.flushOutput()
    time.sleep(1)  
    print(gcc)
    return;



def setFHPeakCurrent(ser,lpc):    
    if (lpc <= 0):
        lpc = 10000 #Default
    vStr = str(lpc)
    ser.write(bytes([76,80,67]))
    ser.write(vStr.encode())
    ser.write(bytes([13,10]))  
    return;

def getFHPeakCurrent(ser):
    ser.flushInput()
    ser.flushOutput()
    
    ser.write(bytes([71,80,67,13]))
    gpc = ser.readline()

    if (checkingFHOK(gpc) == True):
        ser.write(bytes([71,80,67]))
        gpc = ser.readline()
           
    print("OK")
    ser.cancel_write()
    ser.flushInput()
    ser.flushOutput()
    time.sleep(1)  
    print(gpc.decode("utf-8"))
    return gpc.decode("utf-8");

##########################################

### Control ###

def getFHGainController(ser):
    ser.flushInput()
    ser.flushOutput()
    
    ser.write(bytes([71,80,80,13,10]))
    gpp = ser.readline()
    
    ser.cancel_write()
    ser.flushInput()
    ser.flushOutput()
    time.sleep(1)  
    print(gpp.decode("utf-8"))
    return (gpp.decode("utf-8"));

def setFHGainController(ser,pp):
    if (pp <= 0):
        pp = 64 #Default
    vStr = str(pp)
    ser.write(bytes([80,80]))
    ser.write(vStr.encode("utf-8"))
    ser.write(bytes([13,10]))
                
    if (checkingFHOK(pp) == True):
        print("OK")
    
    return;

def getFHPTerm(ser):
    ser.flushInput()
    ser.flushOutput()
    
    ser.write(bytes([71,80,79,82,13,10]))
    gpor = ser.readline()         


    ser.cancel_write()
    ser.flushInput()
    ser.flushOutput()
    time.sleep(1)  
    print("P Term: ", gpor.decode("utf-8"))
    return (gpor.decode("utf-8"));

def setFHPTerm(ser,por):
    ser.flushInput()
    ser.flushOutput()
    if (por <= 0):
        por = 2 #Default
    vStr = str(por)
    ser.write(bytes([80,79,82]))
    ser.write(vStr.encode("utf-8"))
    ser.write(bytes([13,10]))
    
    por = ser.readline()
        
    if (checkingFHOK(por) == True):
        print("OK")

def getFHITerm(ser):
    ser.flushInput()
    ser.flushOutput()
    
    ser.write(bytes([71,73,13,10]))
    gi = ser.readline()
           
    ser.cancel_write()
    ser.flushInput()
    ser.flushOutput()
    time.sleep(1)  
    print("I Term: ", gi.decode("utf-8"))
    return (gi.decode("utf-8"));

def setFHITerm(ser,i):
    if (i <= 0):
        i = 61 #Default
    vStr = str(i)
    ser.write(bytes([73]))
    ser.write(vStr.encode("utf-8"))
    ser.write(bytes([13,10]))
    
    i = ser.readline()
    
    if (checkingFHOK(i) == True):
        print("OK")
        
    
    return;

def getFHDTerm(ser):
    ser.flushInput()
    ser.flushOutput()
    
    ser.write(bytes([71,80,68,13,10]))
    gpd = ser.readline()      
    
    ser.cancel_write()
    ser.flushInput()
    ser.flushOutput()
    time.sleep(1)  
    print("D Term: ", gpd.decode("utf-8"))

    return (gpd.decode("utf-8"));

def setFHDTerm(ser,pd):
    if (pd <= 0):
        pd = 2 #Default
    vStr = str(pd)
    ser.write(bytes([80,68]))
    ser.write(vStr.encode("utf-8"))
    ser.write(bytes([13,10]))
    
    pd = ser.readline()
    
    if (checkingFHOK(pd) == True):
        print("OK")
    
    return;
   

 
##########################################

############### Sequence #################
 
#commAsTCPClient()






    
ser.close()



    
