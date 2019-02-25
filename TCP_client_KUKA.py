#!/usr/bin/env python

#################################
############ ARTC ################
#################################
###### Walter Frank Pintor Ortiz #########
#################################
########In-House Gripper#############
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
      sock.sendto('startCommClient\n'.encode('utf-8'),(server_address))
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

    #######################################################################
    #######################################################################
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    counter = 0
    server_address = ('172.31.1.146', 30004)

    # Connect the socket to the port where the server is listening
    # server_address = ('192.168.1.196', 5004)
    
    while (counter < 100):
        try:            
            
            print ('connecting to %s port %s',  server_address)
            sock.connect(server_address)
            connected = True
            #######################################################################
            #######################################################################
            
            sock.setblocking(True)

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
                print ("Faulhaber motor: ",ser2)
                #Start Shinano
                ser1 = Shinano_Control.openSNPort()
                print ("Shinano motor: ",ser1)
                # Send data
                time.sleep(2)
                message = 'startCommClient\n'
                print ('sending: ',  message)
                sock.sendto(message.encode('utf-8'),(server_address))

                # Look for the response
                amount_received = 0
                amount_expected = len(message)
                
                #thread1.start()
                RunAlways = True
                
                #Infinite Loop to check for information requests
                while RunAlways:
                    #ready = select.select([sock],[],[], timeout_in_seconds)
                    #if ready[0]:
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
                        #thread1.join()

                        # Stop to receive
                        sock.setblocking(True)
                        
                        # Connected to server - LED
                        theaterChase(strip, Color(0, 0, 255))  # Blue theater chase            
                                                                        
                        #ready = select.select([sock],[],[], timeout_in_seconds)
                        #print (ready)
                        #if ready[0]:
                        datum = sock.recv(13)
                        datumstr =  datum.decode('utf-8')
                        datumstr = ''.join(datumstr.split())
                        amount_received += len(data)
                        print ('Received action command: ',  datumstr)
                            
                        time.sleep(1)                           

                        #thread1.exit()

                        ##TODO insert here command to send constant information regarding Torque and Speed
                        ######            
                        ######
                        ###################################################################
                        ## Getting PID values for FH motor
                        if (datumstr == "getPIDrotval"):
                            
                            sock.sendto('Roger\n'.encode('utf-8'),(server_address))
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
                            
                        
                        ## Getting PID values for SN motor
                        elif (datumstr == "getPIDgrpval"):
                            
                            sock.sendto('Roger'.encode('utf-8'),(server_address))
                            SNgain = Shinano_Control.getSNGainController(ser1).encode('utf-8')
                            
                            datumstr = "0"
                            
                         
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
                            
                        
                        ##Setting limit of SN current##
                        elif (datumstr == "curgripLimit"):
                            time.sleep(0.8)
                            datum = sock.recv(18)
                            
                            while not datum:
                                datum = sock.recv(18)


                            print ("This is datum: ", datum)
                            value1str = datum.decode("utf-8")
                            #print (value1str)
                            index = value1str.find('$\r\n')
                            
                            while (index == -1):
                                datum = sock.recv(18)
                                value1str = datum.decode("utf-8")                        
                                index = value1str.find('$\r\n')

                                               
                            information = value1str.split('$\r\n')
                            print(information)
                            print ("This is information[0] =", information[0])
                            print ("This is information[1] =", information[1])
                            
                            indexes = value1str.index('$')
                            
                            currentSN = float(information[0])                    
                            print ('Current limit set: ', currentSN)
                            
                            if information[1].lstrip("-").isdigit():
                                velocitySN = float(information[1])                        
                                 
                                print ('Speed limit set: ', velocitySN)
                            else:
                                datum = sock.recv(18)
                                value1str = datum.decode("utf-8")                        
                                index = value1str.find('$')
                                print ("Value of index: ", index)
                                if (index != -1):
                                    information = value1str.split('$')
                                    indexes = value1str.index('$')
                                    velocitySN = float(information[0])
                                    print ('Speed limit set: ', velocitySN)
                                else:
                                    velocitySN = 0
                            
                            torqueTargetBytes = Shinano_Control.setTorqueTarget(ser1,currentSN,velocitySN) #Torque in Dec // Requires 3 params
                            Shinano_Control.moveMotorWithTorqueLimit(ser1,torqueTargetBytes)          
                                                    
                            ##TODO add an extra loop that queries the current value until it reaches the defined currentSN
                            ## After condition is met, send and boolean to tell robot to proceed
                            
                            currentSNreal = []
                            currentSNreal.append(Shinano_Control.getCurrentTorque(ser1))

                            ## While loop to check force is met
                            temp = 1         
                            while (temp == 1):       
                            
                                for x in range(4): ##Providing and conservative measure
                                    currentSNreal.append(Shinano_Control.getCurrentTorque(ser1))

                                currentSNrealAvg = sum(currentSNreal) / float(len(currentSNreal))

                                print ("Commanded current : ", currentSN)
                                print ("Real average current: ", currentSNrealAvg)
                                if (currentSN < currentSNrealAvg*1.2): ##20% Higher torque before accepting value
                                    temp = -1
                                currentSNreal.clear()

                            time.sleep(0.5)

                            
                            time.sleep(0.5)
                            Shinano_Control.setSpeedTarget(ser1,0)
                            Message = "Roger\n"
                            ##Don't rotate more
                            Shinano_Control.setTorqueTarget(ser1,currentSN,0)
                            ##
                            sock.send(Message.encode('utf-8'))                        
                            #sock.sendall(Message.encode('utf-8'))
                            
                            print ("Roger sent")             
                            datumstr = "0"
                                                  
                         
                        ###Setting limit of SN Speed##
                        elif (datumstr == "SPDgriperCMD"):
                            
                            #ready = select.select([sock],[],[], timeout_in_seconds)
                            #if ready[0]:
                            datum = sock.recv(6)
                            velocitystr =  datum.decode('utf-8')
                            velocitystr = ''.join(velocitystr.split())
                            velocitySN = float(velocitystr)
                            
                            Shinano_Control.setSpeedTarget(ser1, velocitySN)                        
                            
                            print ('Speed limit set: ', velocitySN)
                            datumstr = "0"
                                                   
                                    
                        
                        ##Stopping FH rotation##
                        elif (datumstr == "SPDstopitCMD"):	 
                            Faulhaber_Control.setFHVelocity(ser2,0)
                            print ('Velocity commanded (RPMs): ', 0)
                            datumstr = "0"
                        
                        ##Stopping FH rotation##
                        elif (datumstr == "POSstopitCMD"):	 
                            Faulhaber_Control.setFHPosition(ser2,0)
                            print ('Position commanded (Encoder): ', 0)
                            datumstr = "0"
                        
                        
                        ##Defining FH rotation speed##
                        elif (datumstr == "SPDrotateCMD"):	   
                            
                            time.sleep(0.5)
                            ##Receive size of possible digits
                            datum = sock.recv(2)
                            
                            print ('Thing sent (RPMs): ', (datum))
                            print ('Size of commanded (RPMs): ', len(datum))
                            
                            sizestr =  datum.decode('utf-8')
                            sizestr = sizestr.rstrip('\n')
                            
                            sizestr = sizestr.replace("$","")
                            sizestr = sizestr.replace("\n","")
                            sizestr = sizestr.replace(' ','')
                            sizestr = ''.join(sizestr.split())
                            while (sizestr == ""):
                                datum = sock.recv(2)
                                sizestr =  datum.decode('utf-8')
                                sizestr = sizestr.rstrip('\n')
                            
                                sizestr = sizestr.replace("$","")
                                sizestr = sizestr.replace("\n","")
                                sizestr = sizestr.replace(' ','')
                                sizestr = ''.join(sizestr.split())
                                print ("Corrected value found: ",sizestr)
                            
                            ##receive final value
                            time.sleep(2)
                            datum = sock.recv(int(sizestr)+2)
                            velocitystr =  datum.decode('utf-8')
                            velocitystr = velocitystr.rstrip('\n')
                            print ('Thing sent (RPMs): ', (velocitystr))
                            
                            velocitystr = velocitystr.replace("$","")
                            velocitystr = velocitystr.replace("\n","")
                            velocitystr = velocitystr.replace(' ','')
                            velocitystr = ''.join(velocitystr.split())
                            
                            print ("Rotation data: ",velocitystr)
                            
                            if velocitystr == "":                        
                                velocitystr = 0                                          
                            
                            velocityFH = int(velocitystr)                    
                            
                            Faulhaber_Control.setFHVelocity(ser2,velocityFH)
                            print ('Velocity commanded (RPMs): ', velocityFH)
                            
                            

                            time.sleep(0.5)
                            datumstr = "0"
                            
                        ##Defining FH rotation position##
                        elif (datumstr == "POSrotateCMD"):	   
                            
                            time.sleep(0.5)
                            ##Receive size of possible digits
                            datum = sock.recv(2)
                            
                            print ('Thing sent (Econder): ', (datum))
                            print ('Size of commanded (Encoder): ', len(datum))
                            
                            sizestr =  datum.decode('utf-8')
                            sizestr = sizestr.rstrip('\n')
                            
                            sizestr = sizestr.replace("$","")
                            sizestr = sizestr.replace("\n","")
                            sizestr = sizestr.replace(' ','')
                            sizestr = ''.join(sizestr.split())
                            while (sizestr == ""):
                                datum = sock.recv(2)
                                sizestr =  datum.decode('utf-8')
                                sizestr = sizestr.rstrip('\n')
                            
                                sizestr = sizestr.replace("$","")
                                sizestr = sizestr.replace("\n","")
                                sizestr = sizestr.replace(' ','')
                                sizestr = ''.join(sizestr.split())
                                print ("Corrected value found: ",sizestr)
                            
                            ##receive final value
                            time.sleep(2)
                            datum = sock.recv(int(sizestr)+2)
                            positionstr =  datum.decode('utf-8')
                            positionstr = positionstr.rstrip('\n')
                            print ('Thing sent (Encoder Values): ', (positionstr))
                            
                            positionstr = positionstr.replace("$","")
                            positionstr = positionstr.replace("\n","")
                            positionstr = positionstr.replace(' ','')
                            positionstr = ''.join(positionstr.split())
                            
                            print ("Position data: ",positionstr)
                            
                            if positionstr == "":                        
                                positionstr = 0                                          
                            
                            positionFH = int(positionstr)                    
                            
                            Faulhaber_Control.setFHposition(ser2,positionFH)
                            print ('Position commanded (Encoder): ', positionFH)
                            
                            

                            time.sleep(0.5)
                            datumstr = "0"

                        ##Defining SN position ##
                        elif (datumstr == "POSgriperCMD"):
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
                            
                    
                        ##Grab with Torque of gripper
                        elif (datumstr == "GRPTorqueCMD"):                    
                                            
                            torqueTargetBytes = Shinano_Control.setTorqueTarget(ser1,currentSN,0)           
                            #Shinano_Control.moveMotorWithTorqueLimit(ser1,torqueTargetBytes)

                            # Connected to server - LED
                            theaterChase(strip, Color(255, 165, 0))  # Orange theater chase
                            
                            datumstr = "0"
                            
                        
                        ##Grab component Speed 
                        elif (datumstr == "GRPVelociCMD"):
                            
                            
                            speedTargetBytes = Shinano_Control.setSpeedTarget(ser1, 0) #Speed in Dec // Requires 2 params

                            # Connected to server - LED
                            theaterChase(strip, Color(255, 140, 0))  # Orange theater chase
                            
                            datumstr = "0"
                            
                        
                        
                            
                        ###################################################################              
                        ##Enabling functions
                        ###################################################################
                        
                        ##Enabling FH Drive
                        elif (datumstr == "enabFHDrive"):                 
                            Faulhaber_Control.enableFHDrive(ser2)
                            print ('FH Drive enabled')   
                                            
                            datumstr = "0"
                            
                        
                        ##Disabling FH Drive
                        elif (datumstr == "disaFHDrive"):
                            Faulhaber_Control.disableFHDrive(ser2)
                            print ('FH Drive disabled')

                            datumstr = "0"
                                       
                        
                        
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
                            
                        
                        elif (datumstr == "enabSNDrivP"):                 
                            # Connected to server - LED
                            theaterChase(strip, Color(0, 0, 255))  # Blue theater chase
                            
                            Shinano_Control.turnOFFServoOp(ser1)                
                            Shinano_Control.setPosControl(ser1)
                            Shinano_Control.inputModePosControl(ser1)                
                            Shinano_Control.turnONServoOp(ser1)
                            print ('SN Drive enabled for Position')   
                                            
                            datumstr = "0"
                            
                        
                        ##Disabling SN Drive
                        elif (datumstr == "disaSNDrive"):
                            # Connected to server - LED
                            theaterChase(strip, Color(221, 160, 221))  # Plum theater chase
                            
                            Shinano_Control.turnOFFServoOp(ser1)
                            print ('SN Drive disabled')

                            datumstr = "0"
                            
                        
                        
                        ##Disconnect IoT Device
                        elif (datumstr == "disconnect"):
                            # Connected to server - LED
                            theaterChase(strip, Color(127, 127, 127))  # White theater chase
                            
                            #sock.sendto('startCommClient'.encode('utf-8'),(server_address))
                            RunAlways = False
                            datastr = ""
                            datumstr = "0"
                            break
                        
                        
                        #If no case taken
                        else:
                            datumstr = "0"
                            
                                     
            finally:
                print ('Closing Socket')
                
                # Disconnecting LED
                theaterChase(strip, Color(0, 0, 0),10)  # White theater chase

                #Turning off motors
                Faulhaber_Control.disableFHDrive(ser2)
                Shinano_Control.turnOFFServoOp(ser1)
                
                #Closing Serial comm
                print ('Closing Faulhaber Connection')
                Faulhaber_Control.closeFHPort(ser2)
                print ('Closing Shinano Connection')
                Shinano_Control.closeSNPort(ser1)

                #Closing socket
                sock.close()
            
            #######################################################################
            #######################################################################    
        except socket.error as error:
            #######################################################################
            #######################################################################
            # Create a TCP/IP socket
            connected = False
            
            print( "connection lost... reconnecting" )
            time.sleep(1)
            
            while not connected:
                #Trying to reconnect
                try:
                    #sock.close()
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    #sock.connect(server_address)
                    connected = True
                    print( "re-connection successful" )
                    
                except socket.error:
                    time.sleep(3)
                    print("Connection Failed **BECAUSE:** {}".format(error))
                    print("Attempt {} of 100".format(counter))
                    counter += 1
