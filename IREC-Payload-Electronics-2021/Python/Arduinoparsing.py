import serial
import time
arduinoData = serial.Serial('COM4',115200);
time.sleep(1)
while(1==1):
     while (arduinoData.inWaiting()==0):
         pass
     dataPacket = arduinoData.readline()
     
     dataPacket = str(dataPacket,'utf-8')
     splitPacket = dataPacket.split(',')
     acccal=float(splitPacket[0])
     gyrcal=float(splitPacket[1])
     magcal=float(splitPacket[2])
     syscal=float(splitPacket[3])
     theta=float(splitPacket[4])
     phi=float(splitPacket[5])
     psi=float(splitPacket[6])

     print("acccal=",acccal,"gyrcal=",gyrcal,"magcal=",magcal,"syscal=",syscal,"theta=",theta,"phi=",phi,"psi=",psi);
