import socket
import sys
import time
#import Faulhaber_Control
import Shinano_Control

try:
    
    # Start Faulhaber
    #ser1 = Faulhaber_Control.openFHPort()
    
    #Start Shinano
    
    
    select = input('Select Option 1,2 or 3')
    print (select)
    
    print ('input is: ',select)
    if select == '1':
        print ('sss')
        ser2 = Shinano_Control.openSNPort()    
        Shinano_Control.turnOFFServoOp(ser2)
        time.sleep(1)
        Shinano_Control.setPosControl(ser2)
        time.sleep(2)
        Shinano_Control.inputModePosControl(ser2)
        time.sleep(3)
        Shinano_Control.turnONServoOp(ser2)
        time.sleep(4)   
        posTargetBytes = Shinano_Control.setPosTarget(ser2, 150) #Position in Dec
        time.sleep(5)
        Shinano_Control.moveMotorWithPosControl(ser2,posTargetBytes)
    elif select == '2':
        print ('sss')
        ser2 = Shinano_Control.openSNPort()    
        Shinano_Control.turnOFFServoOp(ser2)
        time.sleep(1)
        Shinano_Control.setSpeedControl(ser2)
        time.sleep(2)
        Shinano_Control.inputModeSpeedControl(ser2)
        time.sleep(3)
        Shinano_Control.turnONServoOp(ser2)
        time.sleep(4)   
        speedTargetBytes = Shinano_Control.setSpeedTarget(ser2, 40) #Speed in Dec // Requires 2 params
        time.sleep(5)
        Shinano_Control.moveMotorWithSpeedLimit(ser2,speedTargetBytes)
        time.sleep(5)
        Shinano_Control.turnOFFServoOp(ser2)
    elif select == '3':
        ser2 = Shinano_Control.openSNPort()    
        Shinano_Control.turnOFFServoOp(ser2)
        time.sleep(1)
        Shinano_Control.setTorqueControl(ser2)
        time.sleep(1)
        Shinano_Control.inputModeTorqueControl(ser2)
        time.sleep(1)
        Shinano_Control.turnONServoOp(ser2)
        time.sleep(1)        
        torqueTargetBytes = Shinano_Control.setTorqueTarget(ser2,600,-10) #Torque in Dec // Requires 3 params
        time.sleep(1)
        Shinano_Control.moveMotorWithTorqueLimit(ser2,torqueTargetBytes)

finally:
    print ('Closing Socket')
        
    print ('Closing Shinano Connection')
    Shinano_Control.closeSNPort(ser2)