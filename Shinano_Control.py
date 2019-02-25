#!/usr/bin/env python

#################################
############ ARTC ###############
#################################
### Walter Frank Pintor Ortiz ###
#################################

import serial
import time
import socket
import array
import binascii
import codecs

##########################################

def openSNPort():
    ser = serial.Serial('/dev/ttyUSB0',115200, timeout=1)
    return ser;

def closeSNPort(ser):
    ser.close()
    return;


##########################################
ser = openSNPort()
v = 500 #Test velocity
cpt = 0          

### Activation/deactivation functions ###
                                                 
####################################################################################################################

def turnONServoOp(ser):
    #Turn on servo operation
    TurnonValues = bytearray([0x81, 0x06, 0x02, 0x1E, 0x01, 0x57])
    #print (str(binascii.hexlify(TurnonValues)))
    ser.write(TurnonValues)

    response = ser.readline()
    #print ('Response from controller:', response)    
    return;

def turnOFFServoOp(ser):
    #Turn off servo operation
    TurnoffValues = bytearray([0x81, 0x06, 0x02, 0x1E, 0x00, 0x58])
    #print (str(binascii.hexlify(TurnoffValues)))
    ser.write(TurnoffValues)

    response = ser.readline()
    #print ('Response from controller:', response)        
    return;

####################################################################################################################

def getFirmware(ser):
    getFirm = bytearray([0x81, 0x07, 0x01, 0x01])
    checksumFinal = checkSumInHex(getFirm)
    getFirm.append(checksumFinal)
    ser.write(getFirm)
    
    ##Implement the reading firmaware method
    
    return;

def getServoOffMode(ser):
    getServoMode = bytearray([0x81, 0x07, 0x01, 0x1F])
    checksumFinal = checkSumInHex(getServoMode)
    getServoMode.append(checksumFinal)
    ser.write(getServoMode)
    
    response = ser.readline()
    #print ('Response from controller:', response)
    
    servoMode = response.hex()
    servostr = servoMode[8:10]
    servoMode = int(servostr, 16)
    
    if (servoMode == 0):
        print ('Servo OFF Mode: ', servoMode)   
    else:
        print ("Check servo quality")
    
    return;

def getFirstInitialization(ser):
    getFirstInit = bytearray([0x81, 0x07, 0x01, 0x20])
    checksumFinal = checkSumInHex(getFirstInit)
    getFirstInit.append(checksumFinal)
    ser.write(getFirstInit)
    
    response = ser.readline()
    #print ('Response from controller:', response)
    
    initDrive = response.hex()
    initDrivestr = initDrive[8:10]
    initDrive = int(initDrivestr, 16)
    
    if (servoMode == 0):
        print ('Initial Drive rotate: ', initDrive)   
    else:
        print ("Check servo quality")
    
    return;

def getInitialPosition(ser): #get current position
    getInitPosCommand = bytearray([0x81, 0x07, 0x01, 0x30])
    checksumFinal = checkSumInHex(getInitPosCommand)
    getInitPosCommand.append(checksumFinal)
    ser.write(getInitPosCommand)
    
    response = ser.readline()
    #print ('Response from controller:', response)
    
    initPos = response.hex()
    initPosstr = initPos[8:16]
    initPos = int(initPosstr, 16)
    
    #print ('Initial Drive Position: ', initPos)   
                
        
    return;



####################################################################################################################

def feedbackAbsPos(ser):
    AbsPosFeedbackCommand = bytearray([0x81, 0x07, 0x01, 0x30])
   
    checksumFinal = checkSumInHex(AbsPosFeedbackCommand)
    AbsPosFeedbackCommand.append(checksumFinal)
    ser.write(AbsPosFeedbackCommand)

    response = ser.readline()
        
    absposResponse = response.hex()
    absposstr = absposResponse[18:26]
    absposResponse = int(absposstr, 16)    
    
    if (absposResponse > 1000000):
        absposResponse = 4294967295 - absposResponse
        absposResponse = absposResponse * (-1)
        print ('Absolute position: ', absposResponse)
        return absposResponse;
    else:
        print ('Absolute position: ', absposResponse)
        return absposResponse;     
    

def feedbackRelPos(ser):
    
    #TODO: Implement Relative position feedback
    return;



def feedbackSpeed(ser):
    #Get Speed present value
      
    SpeedFeedbackCommand = bytearray([0x81, 0x07, 0x01, 0x2F])
   
    checksumFinal = checkSumInHex(SpeedFeedbackCommand)
    SpeedFeedbackCommand.append(checksumFinal)
    ser.write(SpeedFeedbackCommand)

    response = ser.readline()    
    
    speedResponse = response.hex()
    speedstr = speedResponse[18:26]
    speedFeedback = int(speedstr, 16)
               
    if (speedFeedback > 1000000):
        speedFeedback = 4294967295 - speedFeedback
        speedFeedback = speedFeedback * (-1)
        print ('Total speed: ', speedFeedback)
        return speedFeedback;
    else:
        print ('Total speed: ', speedFeedback)
        return speedFeedback;    
    

def feedbackCurrent(ser):
        #Get Speed present value
      
    CurrentFeedbackCommand = bytearray([0x81, 0x07, 0x01, 0x2E, 0x48])   

    ser.write(CurrentFeedbackCommand)

    response = ser.readline()
    #print ('Response from controller:', response)
    
    currentResponse = response.hex()
    currentstr = currentResponse[18:26]
    currentFeedback = int(currentstr, 16)   
    

    
    if (currentFeedback > 15000):
        currentFeedback = 4294967295 - abs(currentFeedback)
        currentFeedback = currentFeedback * (-1)
        print ('Torque: ', currentFeedback)
        return currentFeedback;
    else:
        print ('Torque: ', currentFeedback)
        return currentFeedback;      
    


def feedbackAll(ser):
    
    feedbackAbsPos(ser)    
    feedbackSpeed(ser)
    currentFeedback = feedbackCurrent(ser)
    
    return currentFeedback;


####################################################################################################################




####################################################################################################################

def setPosControl(ser):
    #Set pos control operation
    PosControlValues = bytearray([0x81, 0x06, 0x03, 0x28, 0x03, 0x00, 0x4A])
    #print (str(binascii.hexlify(PosControlValues)))
    ser.write(PosControlValues)

    response = ser.readline()
    #print ('Response from controller:', response)        
    return;

def inputModePosControl(ser):
    #Set input mode pos control operation
    inputModePosControlValues = bytearray([0x81, 0x06, 0x04, 0x29, 0x03, 0x00, 0x00, 0x48])
    print (str(binascii.hexlify(inputModePosControlValues)))
    ser.write(inputModePosControlValues)

    response = ser.readline()
    #print ('Response from controller:', response)     
    
    return;

def setPosTarget(ser, targetPos):
    #Set pos target operation

    ##Assuming a single byte used!!
    ####TODO needs to make this function scalable for larger position values

    #Set speed target operation
    isCCWFlag = False
    if (targetPos < 0): 
        targetPos = targetPos*-1
        isCCWFlag = True

    PosTrans = (targetPos / 60) * (49 * 8192)
    PosHex = hex(int(PosTrans))
    PosHex = PosHex.replace('x','0')    
    PosHex = PosHex[::-1]
    sizePosHex = len(PosHex)
    
    if sizePosHex > 10:
        print ('error')
        return;
    
    for num in range(len(PosHex),11):
        PosHex += str(0)
        #print (PosHex)    
    
    lsbPos1_Hex = '0x'+ PosHex[1] + PosHex[0]
    lsbPos1 = int(lsbPos1_Hex,16)
    
    lsbPos2_Hex = '0x'+ PosHex[3] + PosHex[2]
    lsbPos2 = int(lsbPos2_Hex,16)
    
        
    msbPos1_Hex = '0x'+ PosHex[5] + PosHex[4]
    msbPos1 = int(msbPos1_Hex,16)
    
    msbPos2_Hex = '0x'+ PosHex[7] + PosHex[6]
    msbPos2 = int(msbPos2_Hex,16) 

    if (isCCWFlag):        
        posBytes = invertingBytes(posBytes)
        msbPos2 = posBytes[0]
        msbPos1 = posBytes[1]
        lsbPos2 = posBytes[2]
        lsbPos1 = posBytes[3]        
    
    ####TODO needs to make this function scalable for larger position values
    targetPosValues = bytearray([0x81, 0x06, 0x06, 0x2D, msbPos2, msbPos1, lsbPos2, lsbPos1])
    
    checksumFinal = checkSumInHex(targetPosValues)
    #print ('test: ', checksumFinal)
    targetPosValues.append(checksumFinal)
    
    #print (str(binascii.hexlify(targetPosValues)))
    ser.write(targetPosValues)

    response = ser.readline()
    #print ('Response from controller:', response) 
    
    return targetSpeedValues;



def moveMotorWithPosControl(ser2, targetPosValues):
    
    ser.write(targetPosValues)

    response = ser.readline()
    #print ('Response from controller:', response) 
    
    return;



def getPosTarget(ser):
    #Get position
      
    checkPosValues = bytearray([0x81, 0x07, 0x01, 0x30, 0x46])
   
    #print (str(binascii.hexlify(checkPosValues)))
    ser.write(checkPosValues)

    response = ser.readline()
    #print ('Response from controller:', response)
    
    positionResponse = response.hex()
    
    
    if (positionResponse[8:16] == '0xFFFFFFFF'):
        positionstr = 0
    
    if (positionResponse[8:10] == '0xFF'):
        positionstr = '0xFFFFFFFF' - positionResponse[8:16]
    else:
        positionstr = positionResponse[8:16]        
        
    position = int(positionstr, 16)
    #print ('Actual distance: ', position)
    
    return;



####################################################################################################################
def setSpeedControl(ser):
    #Set speed control operation
    SpeedControlValues = bytearray([0x81, 0x06, 0x03, 0x28, 0x02, 0x00, 0x4B])
    #print (str(binascii.hexlify(SpeedControlValues)))
    ser.write(SpeedControlValues)

    response = ser.readline()
    #print ('Response from controller:', response)        
    return;

def inputModeSpeedControl(ser):
    #Set input mode speed control operation
    inputModeSpeedControlValues = bytearray([0x81, 0x06, 0x04, 0x29, 0x02, 0x00, 0x00, 0x49])
    #print (str(binascii.hexlify(inputModeSpeedControlValues)))
    ser.write(inputModeSpeedControlValues)

    response = ser.readline()
    #print ('Response from controller:', response)     
    
    return;


def setSpeedTarget(ser, targetSpeed):
    #Set speed target operation
    isCCWFlag = False
    if (targetSpeed < 0): 
        targetSpeed = targetSpeed*-1
        isCCWFlag = True
    elif (targetSpeed == 0): 
        targetSpeedValues = bytearray([0x81, 0x06, 0x05, 0x2C, 0x00, 0x00, 0x00, 0x00])
        checksumFinal = checkSumInHex(targetSpeedValues)
        #print ('test: ', checksumFinal)
        targetSpeedValues.append(checksumFinal)
        ser.write(targetSpeedValues)
        return targetSpeedValues;
        
    SpeedTrans = (targetSpeed / 60) * (49 * 8192)
    SpeedHex = hex(int(SpeedTrans))
    SpeedHex = SpeedHex.replace('x','0')    
    SpeedHex = SpeedHex[::-1]
    sizeSpeedHex = len(SpeedHex)
    
    if sizeSpeedHex > 10:
        print ('error')
        return;
    
    for num in range(len(SpeedHex),11):
        SpeedHex += str(0)
        #print (SpeedHex)    
    
    lsbSpeed1_Hex = '0x'+ SpeedHex[1] + SpeedHex[0]
    lsbSpeed1 = int(lsbSpeed1_Hex,16)
    
    lsbSpeed2_Hex = '0x'+ SpeedHex[3] + SpeedHex[2]
    lsbSpeed2 = int(lsbSpeed2_Hex,16)    
        
    msbSpeed1_Hex = '0x'+ SpeedHex[5] + SpeedHex[4]
    msbSpeed1 = int(msbSpeed1_Hex,16)
    
    msbSpeed2_Hex = '0x'+ SpeedHex[7] + SpeedHex[6]
    msbSpeed2 = int(msbSpeed2_Hex,16)

    speedBytes = bytearray ([msbSpeed2, msbSpeed1, lsbSpeed2, lsbSpeed1])

    if (isCCWFlag):        
        speedBytes = invertingBytes(speedBytes)
        msbSpeed2 = speedBytes[0]
        msbSpeed1 = speedBytes[1]
        lsbSpeed2 = speedBytes[2]
        lsbSpeed1 = speedBytes[3]    
    
    ####TODO needs to make this function scalable for larger position values
    targetSpeedValues = bytearray([0x81, 0x06, 0x05, 0x2C, msbSpeed2, msbSpeed1, lsbSpeed2, lsbSpeed1])
        
    checksumFinal = checkSumInHex(targetSpeedValues)
    #print ('test: ', checksumFinal)
    targetSpeedValues.append(checksumFinal)
    
    #print (str(binascii.hexlify(targetSpeedValues)))
    ser.write(targetSpeedValues)

    response = ser.readline()
    #print ('Response from controller:', response) 
    
    return targetSpeedValues;

def moveMotorWithSpeedLimit(ser,targetSpeedValues):
    ser.write(targetSpeedValues)

    response = ser.readline()
    #print ('Response from controller:', response) 
    
    return;


def getSpeedTarget(ser):
    #Get Speed
      
    checkSpeedValues = bytearray([0x81, 0x07, 0x01, 0x2F, 0x47])
   
    #print (str(binascii.hexlify(checkSpeedValues)))
    ser.write(checkSpeedValues)

    response = ser.readline()
    #print ('Response from controller:', response)
    
    speedResponse = response.hex()
    
    if(speedResponse[8:10] == '0xFF'):
        speedstr = '0xFFFFFFFF' - speedResponse[8:16]
    else:
        speedstr = speedResponse[8:16]
    speedFeedback = int(speedstr, 16) - 1
    #print ('Actual speed: ', speedFeedback)
    
           
    #Get Torque
     
    checkTorqueValues = bytearray([0x81, 0x07, 0x01, 0x2B, 0x4B])
   
    #print (str(binascii.hexlify(checkTorqueValues)))
    ser.write(checkTorqueValues)

    response = ser.readline()
    #print ('Response from controller:', response)
    
    torqueResponse = response.hex()
    
    if(torqueResponse[8:10] == '0xFF'):
        torquestr = '0xFFFFFFFF' - torqueResponse[8:16]
    else:
        torquestr = torqueResponse[8:16]
    
    torquestr = torqueResponse[8:16]
    torqueFeedback = int(torquestr, 16) - 1
    #print ('Actual torque: ', torqueFeedback)   
    
    return;


####################################################################################################################

def setTorqueControl(ser):
    #Set Torque control operation
    TorqueControlValues = bytearray([0x81, 0x06, 0x03, 0x28, 0x01, 0x00, 0x4C])
    #print (str(binascii.hexlify(TorqueControlValues)))
    ser.write(TorqueControlValues)

    response = ser.readline()
    #print ('Response from controller:', response)        
    return;

def inputModeTorqueControl(ser):
    #Set input mode pos control operation
    inputModeTorqueControlValues = bytearray([0x81, 0x06, 0x04, 0x29, 0x01, 0x00, 0x00, 0x4A])
    #print (str(binascii.hexlify(inputModeTorqueControlValues)))
    ser.write(inputModeTorqueControlValues)

    response = ser.readline()
    #print ('Response from controller:', response)     
    
    return;

def setTorqueTarget(ser, targetTorque, targetSpeed):
    #Set torque target operation
    isCCWFlag = False
    
    if (targetSpeed == 0 or targetTorque == 0): 
        targetTorqueValues = bytearray([0x81,0x06,0x09,0x2B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
        checksumFinal = checkSumInHex(targetTorqueValues)
        #print ('test: ', checksumFinal)
        targetTorqueValues.append(checksumFinal)
        ser.write(targetTorqueValues)
        return targetTorqueValues;

    if (targetSpeed < 0 and targetTorque > 0):
        targetSpeed = targetSpeed*-1        
        isCCWFlag = True
        
    if (targetTorque < 0 and targetSpeed > 0):
        targetTorque = targetTorque*-1
        isCCWFlag = True

    if (targetTorque < 0 and targetSpeed < 0):
        targetSpeed = targetSpeed*-1  
        targetTorque = targetTorque*-1
        isCCWFlag = True
            

    TorqueTrans = 0
    TorqueTrans = (targetTorque)
    TorqueHex = hex(int(TorqueTrans))
    TorqueHex = TorqueHex[::-1]
    TorqueHex = TorqueHex.replace('x','0')
    
    sizeTorqueHex = len(TorqueHex)
    
    if sizeTorqueHex > 10:
        #print ('error')
        return;
    
    for num in range(len(TorqueHex),11):
        TorqueHex += str(0)
             
    
    lsbTorque1_Hex = '0x'+ TorqueHex[1] + TorqueHex[0]
    lsbTorque1 = int(lsbTorque1_Hex,16)
    
    lsbTorque2_Hex = '0x'+ TorqueHex[3] + TorqueHex[2]
    lsbTorque2 = int(lsbTorque2_Hex,16)
    
        
    msbTorque1_Hex = '0x'+ TorqueHex[5] + TorqueHex[4]
    msbTorque1 = int(msbTorque1_Hex,16)
    
    msbTorque2_Hex = '0x'+ TorqueHex[7] + TorqueHex[6]
    msbTorque2 = int(msbTorque2_Hex,16)    
    

    torqueBytes = bytearray ([msbTorque2, msbTorque1, lsbTorque2, lsbTorque1])
    print (torqueBytes)  

    if (isCCWFlag == True):
        # Torque Variables
        torqueBytes = invertingBytes(torqueBytes)
        msbTorque2 = torqueBytes[0]
        msbTorque1 = torqueBytes[1]
        lsbTorque2 = torqueBytes[2]
        lsbTorque1 = torqueBytes[3]

    # Speed Variables
    targetSpeedValues = setSpeedTarget(ser,targetSpeed)
    #targetSpeedValues = invertingBytes(targetSpeedValues)        
    
    
        
    ####TODO needs to make this function scalable for larger position values - Completed
    targetTorqueValues = bytearray([0x81, 0x06, 0x09, 0x2B, msbTorque2, msbTorque1, lsbTorque2, lsbTorque1, targetSpeedValues[4],targetSpeedValues[5],targetSpeedValues[6],targetSpeedValues[7]])
    print (targetTorqueValues)  

    
    checksumFinal = checkSumInHex(targetTorqueValues)
    
    targetTorqueValues.append(checksumFinal)
    
    #print (str(binascii.hexlify(targetTorqueValues)))
    
    
    return targetTorqueValues;


def moveMotorWithTorqueLimit(ser,targetTorqueValues):
    ser.write(targetTorqueValues)

    response = ser.readline()
    #print ('Response from controller:', response) 
    
    return;


def getTorqueTarget(ser):
    #Get speed
    checkSpeedValues = bytearray([0x81, 0x07, 0x01, 0x2F, 0x47])
   
    #print (str(binascii.hexlify(checkSpeedValues)))
    ser.write(checkSpeedValues)

    response = ser.readline()
    #print ('Response from controller:', response)
    
    speedResponse = response.hex()
    
    if(speedResponse[8:10] == '0xFF'):
        speedstr = '0xFFFFFFFF' - speedResponse[8:16]
    else:
        speedstr = speedResponse[8:16]
    speedFeedback = int(speedstr, 16) - 1
    #print ('Actual speed: ', speedFeedback)
    
       
    #Get Torque
     
    checkTorqueValues = bytearray([0x81, 0x07, 0x01, 0x2B, 0x4B])
   
    #print (str(binascii.hexlify(checkTorqueValues)))
    ser.write(checkTorqueValues)

    response = ser.readline()
    #print ('Response from controller:', response)
    
    torqueResponse = response.hex()
    
    if(torqueResponse[18:20] == '0xFF'):
        torquestr = '0xFFFFFFFF' - torqueResponse[18:26]
    else:
        torquestr = torqueResponse[18:26]
    
    #torquestr = torqueResponse[18:6]
    torqueFeedback = int(torquestr, 16) - 1
    #print ('Actual torque: ', torqueFeedback)       
    
    return;

def getCurrentTorque(ser):
    #Get Torque
     
    checkTorqueValues = bytearray([0x81, 0x07, 0x01, 0x2E, 0x48])
   
    #print (str(binascii.hexlify(checkTorqueValues)))
    ser.write(checkTorqueValues)

    response = ser.readline()
    #print ('Response from controller:', response)
    print (response)
    torqueResponse = response.hex()
    sizetorqueResponse = int(len((torqueResponse)))
    print ("Size: ", sizetorqueResponse)
    
    
    if(torqueResponse[sizetorqueResponse-8:sizetorqueResponse-6] == "ff"):
        torquestr = str(0xffffffff - int(torqueResponse[sizetorqueResponse-10:sizetorqueResponse-2],16) )
    else:
        torquestr = torqueResponse[sizetorqueResponse-10:sizetorqueResponse-2]

    print ("Torque response: ", torqueResponse[sizetorqueResponse-8:sizetorqueResponse-6])
    print ("Total current hex: ", torqueResponse)        
      
    #torquestr = torqueResponse[8:16]
    torqueFeedback = int(torquestr, 16)    

    print ("Current value (int): ", torqueFeedback)    
    return torqueFeedback;


####################################################################################################################    
####################################################################################################################

### Calculate Sum of Bytes

def checkSumInHex(arrayDataDec):

    checksum = 0
    if sum(arrayDataDec) >= 255 :
        checksum = ((sum(arrayDataDec)//255)*256) - (sum(arrayDataDec)-255)
    else:
        checksum = 255 - (sum(arrayDataDec))
    
    #print (checksum)

    return checksum;


####################################################################################################################    
####################################################################################################################


### Inverse of bytes

def invertingBytes(arrayDataHex):

    sizeByte = len(arrayDataHex)
    invertedBytes = bytearray(sizeByte)
    
    for x in range(sizeByte):
        invertedBytes[x] = 255
        invertedBytes[x] = invertedBytes[x] - arrayDataHex[x]          

    invertedBytes[sizeByte-1] = invertedBytes[sizeByte-1] + 1

   

    return invertedBytes;

 
##########################################


 



    
ser.close()



    
