#!/usr/bin/env python

#################################
############ ARTC ###############
#################################
### Walter Frank Pintor Ortiz ###
#################################


import socket
import sys
import os
sys.path.append(os.path.abspath("/home/pi/Desktop/rpi_ws281x"))
import time
import Faulhaber_Control
import Shinano_Control
from LEDstrip import *
import select
import threading

##########################################


class myThread(threading.Thread):
    def __init__(self,threadID,name,counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):
        ## Things to do        
        correctTQ = conditionCheckSN(Shinano_Control.feedbackAll(ser1),currentSN)
        

def conditionCheckSN(sensorCurrent,setpointCurrent):
    print ("Current: ", sensorCurrent)
    if (sensorCurrent >= setpointCurrent):     
      
      Shinano_Control.setSpeedTarget(ser1, 0) ## Stopping gripper!      
      print ('Gripper halted!')
      datumstr = "0"
      sock.sendto('startCommClient'.encode('utf-8'),(server_address))
      return True
    else:
      return False
    
##########################################
    
# Main program logic follows:
if __name__ == '__main__':

    # Initializes Thread
    thread1 = myThread(1,"Thread-SN",1)
    thread1.daemon = True
    
    # Initializes LED configuration
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--clear', action='store_true', help='clear the display on exit')
    args = parser.parse_args()

    # Create NeoPixel object with appropriate configuration.
    strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
    # Intialize the library (must be called once before other functions).
    strip.begin()

    print ('Press Ctrl-C to quit.')
    if not args.clear:
        print('Use "-c" argument to clear LEDs on exit')

    # Connected to server - LED
    theaterChase(strip, Color(127, 127, 127))  # White theater chase

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    

    # Connect the socket to the port where the server is listening
    #server_address = ('192.168.1.196', 5004)
    server_address = ('172.31.1.147', 5004)
    print ('connecting to %s port %s',  server_address)
    sock.connect(server_address)

    sock.setblocking(0)

    # Control FH Parameters
    FHGain = "64"
    FHKp = "2"
    FHKi = "61"
    FHKd = "1"

    # Control SN Parameters
    SNGain = "64"
    SNKp = "2"
    SNKi = "61"
    SNKd = "1"

    # Variables to control
    positionFH = 0
    velocityFH = 0
    currentFH = 400

    positionSN = 0
    velocitySN = 0
    currentSN = 800      

    disconnectCall = 0

    exitFlag = 0
    
    # Variable for data timeout
    timeout_in_seconds = 0.1

    # Initialization
    datumstr = 0

    try:
        
        # Start Faulhaber
        ser2 = Faulhaber_Control.openFHPort()
        
        #Start Shinano
        ser1 = Shinano_Control.openSNPort()
        
        # Send data
        message = 'startCommClient'
        print ('sending: ',  message)
        sock.sendto(message.encode('utf-8'),(server_address))

        # Look for the response
        amount_received = 0
        amount_expected = len(message)
        
        thread1.start()
        
        
        #Infinite Loop to check for information requests
        while 1:
            ready = select.select([sock],[],[], timeout_in_seconds)
            if ready[0]:
                data = sock.recv(11)
                datastr =  data.decode('utf-8')
                datastr = ''.join(datastr.split())
                amount_received += len(data)
                print ('Received start command: ',  datastr)
                print (amount_received)
                print (amount_expected)        
                time.sleep( 1 )            

                print (datastr)            
            
            ##Initialization Server side - Receiving command to start from Server
            while (datastr == "startServer"):

                # Check Current
                thread1.join()

                # Stop to receive
                sock.setblocking(True)
                
                # Connected to server - LED
                theaterChase(strip, Color(0, 0, 255))  # Blue theater chase            
                                                                
                ready = select.select([sock],[],[], timeout_in_seconds)
                #print (ready)
                #if ready[0]:
                datum = sock.recv(13)
                datumstr =  datum.decode('utf-8')
                datumstr = ''.join(datumstr.split())
                amount_received += len(data)
                print ('Received action command: ',  datumstr)
                    
                time.sleep(0.1)                           

                #thread1.exit()

                ##TODO insert here command to send constant information regarding Torque and Speed
                ######            
                ######
                ###################################################################
                ## Getting PID values for FH motor
                if (datumstr == "getPIDrotval"):
                    
                    sock.sendto('Roger'.encode('utf-8'),(server_address))
                    FHgain = Faulhaber_Control.getFHGainController(ser2).encode('utf-8')
                    ##time.sleep( 1 )
                    FHp = Faulhaber_Control.getFHPTerm(ser2).encode('utf-8')
                    ##time.sleep( 1 )
                    FHi = Faulhaber_Control.getFHITerm(ser2).encode('utf-8')
                    ##time.sleep( 1 )
                    FHd = Faulhaber_Control.getFHDTerm(ser2).encode('utf-8')
                    ##time.sleep( 1 )
                    
                    if (FHgain == "$"):
                        FHgain = Faulhaber_Control.getFHGainController(ser2).encode('utf-8')
                    else:
                        sock.sendto(FHgain,(server_address))
                        time.sleep( 1 )
                    
                    if (FHp == "$"):
                        FHp = Faulhaber_Control.getFHPTerm(ser2).encode('utf-8')
                    else:
                        sock.sendto(FHp,(server_address))
                        time.sleep( 1 )
                        
                    if (FHi == "$"):
                        FHi = Faulhaber_Control.getFHITerm(ser2).encode('utf-8')
                    else:
                        sock.sendto(FHi,(server_address))
                        time.sleep( 1 )
                        
                    if (FHd == "$"):
                        print(FHd)
                        FHd = Faulhaber_Control.getFHDTerm(ser2).encode('utf-8')
                    else:
                        sock.sendto(FHd,(server_address))
                        time.sleep( 1 )                             
                    
                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                   
                    break
                
                ## Getting PID values for SN motor
                elif (datumstr == "getPIDgrpval"):
                    
                    sock.sendto('Roger'.encode('utf-8'),(server_address))
                    SNgain = Shinano_Control.getSNGainController(ser1).encode('utf-8')
                    
                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    break
                 
                #Defining PID values for FH motor  
                elif (datumstr == "setPIDrotval"):
                    print("test")
                    value1 = sock.recv(24)
                    value1str = value1.decode("utf-8")
                    #print (value1str)
                    index = value1str.find('$')                
                    value1str.split('$')
                    
                    information = value1str.split('$')
                    print(information)
                    indexes = value1str.index('$')
                    FHGain = float(information[0])
                    print(FHGain)
                    Faulhaber_Control.setFHGainController(ser2,FHGain)
                    print ('Received Gain: ',  FHGain)

                    FHKp = float(information[1])
                    print(FHKp)
                    Faulhaber_Control.setFHPTerm(ser2,FHKp)
                    print ('Received Kp: ',  FHKp)
          
                    FHKi = float(information[2])
                    print(FHKi)
                    Faulhaber_Control.setFHITerm(ser2,FHKi)
                    print ('Received Ki: ',  FHKi)

                    FHKd = float(information[3])
                    print(FHKd)
                    Faulhaber_Control.setFHDTerm(ser2,FHKd)
                    print ('Received Kd: ',  FHKd)
                    
                    datumstr = "0"
                    time.sleep( 1 )
                    
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    time.sleep( 1 )
                    
                    datumstr = "0"
                    break

                ## Setting PID values for SN motor
                #if (datumstr == "getPIDgrpval"):           
                
                    
                #break        

                ##Setting limit of FH current##
                elif (datumstr == "currotaLimit"):           
                    datum = sock.recv(6)
                    currentstr =  datum.decode('utf-8')
                    currentstr = ''.join(currentstr.split())
                    currentFH = float(currentstr)
                    Faulhaber_Control.setFHPeakCurrent(ser2,currentFH)
                    print ('Current limit (mA): ', currentFH)
                    
                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))                
                    #datastr = "0"
                    break
                
                ##Setting limit of SN current##
                elif (datumstr == "curgripLimit"):
                    datum = sock.recv(6)
                    currentstr =  datum.decode('utf-8')
                    currentstr = ''.join(currentstr.split())
                    currentSN = float(currentstr)
                    print ('Current limit set: ', currentSN)
                    
                    torqueTargetBytes = Shinano_Control.setTorqueTarget(ser1,currentSN,velocitySN) #Torque in Dec // Requires 3 params
                    Shinano_Control.moveMotorWithTorqueLimit(ser1,torqueTargetBytes)                    
                                        
                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    break                        
                 
                ###Setting limit of SN Speed##
                elif (datumstr == "SPDgrpCMD"):
                    
                    #ready = select.select([sock],[],[], timeout_in_seconds)
                    #if ready[0]:
                    datum = sock.recv(6)
                    velocitystr =  datum.decode('utf-8')
                    velocitystr = ''.join(velocitystr.split())
                    velocitySN = float(velocitystr)
                    
                    Shinano_Control.setSpeedTarget(ser1, velocitySN)                        
                    
                    print ('Speed limit set: ', velocitySN)
                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    break                        
                            
                                
                 
                ##Defining FH rotation speed##
                elif (datumstr == "SPDrotCMD"):	   
                    datum = sock.recv(6)
                    velocitystr =  datum.decode('utf-8')
                    velocitystr = ''.join(velocitystr.split())
                    velocityFH = int(float(velocitystr))
                    Faulhaber_Control.setFHVelocity(ser2,velocityFH)
                    print ('Velocity commanded (RPMs): ', velocityFH)

                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    #datastr = "0"
                    break
                

                ##Defining SN position ##
                elif (datumstr == "POSgrpCMD"):
                    time.sleep(2)
                    datum = sock.recv(6)
                    positionstr =  datum.decode('utf-8')
                    positionstr = ''.join(positionstr.split())
                    positionSN = float(positionstr)
                    print('testing pos command')
                    if (positionSN < 0):                        
                        Shinano_Control.setPosTarget(ser1, positionSN)
                    elif (positionSN == 0):
                        Shinano_Control.setPosTarget(ser1, 0)
                    else:                        
                        Shinano_Control.setPosTarget(ser1, positionSN)                
                    print ('Position set: ', positionSN)
                    
                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    break
            
                ##Grab with Torque of gripper
                elif (datumstr == "GrpTrqCMD"):                    
                                    
                    torqueTargetBytes = Shinano_Control.setTorqueTarget(ser1,currentSN,0)           
                    #Shinano_Control.moveMotorWithTorqueLimit(ser1,torqueTargetBytes)

                    # Connected to server - LED
                    theaterChase(strip, Color(255, 165, 0))  # Orange theater chase
                    
                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    break
                
                ##Grab component Speed 
                elif (datumstr == "GrpVelCMD"):
                    
                    
                    speedTargetBytes = Shinano_Control.setSpeedTarget(ser1, 0) #Speed in Dec // Requires 2 params

                    # Connected to server - LED
                    theaterChase(strip, Color(255, 140, 0))  # Orange theater chase
                    
                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    break
                
                
                    
                ###################################################################              
                ##Enabling functions
                ###################################################################
                
                ##Enabling FH Drive
                elif (datumstr == "enabFHDrive"):                 
                    Faulhaber_Control.enableFHDrive(ser2)
                    print ('FH Drive enabled')   
                                    
                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    break
                
                ##Disabling FH Drive
                elif (datumstr == "disaFHDrive"):
                    Faulhaber_Control.disableFHDrive(ser2)
                    print ('FH Drive disabled')

                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    break            
                
                
                ##Enabling SN Drives for operating modes
                elif (datumstr == "enabSNDrivT"):                 
                    # Connected to server - LED
                    theaterChase(strip, Color(0, 0, 255))  # Blue theater chase
                    
                    Shinano_Control.turnOFFServoOp(ser1)                
                    Shinano_Control.setTorqueControl(ser1)                
                    Shinano_Control.inputModeTorqueControl(ser1)                
                    Shinano_Control.turnONServoOp(ser1)
                    torqueTargetBytes = Shinano_Control.setTorqueTarget(ser1,0,0)
                    print ('SN Drive enabled for Torque')   
                                    
                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    break
                
                elif (datumstr == "enabSNDrivS"):                 
                    # Connected to server - LED
                    theaterChase(strip, Color(0, 0, 255))  # Blue theater chase
                    
                    Shinano_Control.turnOFFServoOp(ser1)                
                    Shinano_Control.setSpeedControl(ser1)                
                    Shinano_Control.inputModeSpeedControl(ser1)                
                    Shinano_Control.turnONServoOp(ser1)
                    speedTargetBytes = Shinano_Control.setSpeedTarget(ser1, 0) #Speed in Dec // Requires 2 params
                    print ('SN Drive enabled for Speed')   
                                    
                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    break
                
                elif (datumstr == "enabSNDrivP"):                 
                    # Connected to server - LED
                    theaterChase(strip, Color(0, 0, 255))  # Blue theater chase
                    
                    Shinano_Control.turnOFFServoOp(ser1)                
                    Shinano_Control.setPosControl(ser1)
                    Shinano_Control.inputModePosControl(ser1)                
                    Shinano_Control.turnONServoOp(ser1)
                    print ('SN Drive enabled for Position')   
                                    
                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    break
                
                ##Disabling SN Drive
                elif (datumstr == "disaSNDrive"):
                    # Connected to server - LED
                    theaterChase(strip, Color(221, 160, 221))  # Plum theater chase
                    
                    Shinano_Control.turnOFFServoOp(ser1)
                    print ('SN Drive disabled')

                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    break
                
                
                ##Disconnect IoT Device
                elif (datumstr == "disconnect"):
                    # Connected to server - LED
                    theaterChase(strip, Color(127, 127, 127))  # White theater chase
                    
                    #sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    datumstr = "0"
                    datastr = ""
                    break
                
                #If no case taken
                else:
                    datumstr = "0"
                    sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                    break
                             
    finally:
        print ('Closing Socket')
        
        # Disconnecting LED
        theaterChase(strip, Color(0, 0, 0),10)  # White theater chase
        
        Shinano_Control.turnOFFServoOp(ser1)
        
        sock.close()
        print ('Closing Faulhaber Connection')
        Faulhaber_Control.closeFHPort(ser2)
        print ('Closing Shinano Connection')
        Shinano_Control.closeSNPort(ser1)
        



