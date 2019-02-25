import socket
import sys
import time
import Faulhaber_Control


try:
    
    #Start Faulhaber
    ser1 = Faulhaber_Control.openFHPort()
    
    
    
    
    select = input('Select Option 1,2 or 3')
    print (select)
    
    print ('input is: ',select)
    if select == '1':
        
        print ('sss')
        
        Faulhaber_Control.enableFHDrive(ser1)
        Faulhaber_Control.setFHVelocity(ser1,50)
        time.sleep(10)   
        Faulhaber_Control.setFHVelocity(ser1,0)
            
        Faulhaber_Control.disableFHDrive(ser1)
        
    elif select == '2':
        print ('sss')

finally:
    print ('Closing Socket')
        
    print ('Closing Shinano Connection')
    Faulhaber_Control.closeFHPort(ser1)
