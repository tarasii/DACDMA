#!/usr/bin/python
#
#import requests
#import mysql.connector
import datetime
import serial
from time import sleep

def getserial():
  res = ("0","0","0");
  try:
    #ser = serial.Serial(0, 9600)
    ser = serial.Serial("/dev/ttyUSB0", 9600)
    if (ser.isOpen()):
      ser.write("u")
      sleep(0.1)
      tmp = ser.readline()
      ser.read(1)
      #print tmp
      ser.write("z")
      sleep(0.1)
      tmp = ser.readline().split(';')
      ser.read(1)
      lst = [(0,1,tmp[1]),(0,2,tmp[4]),(0,3,tmp[6])]
      #print tmp

      ser.write("t")
      sleep(0.1)
      num = ser.inWaiting()
      #print num
      while (num>0):
        tmp = ser.readline().split()
        ser.read(1)
        #print tmp
        lst.append((1,tmp[0],tmp[1]))
        num = ser.inWaiting()

      res = tuple(lst)
      ser.close()


  except:
    print ("Error comm")

  return res


#def writetable(val):
#  try:
#    conn = mysql.connector.connect(host="localhost", 
#                     user="root", 
#                      passwd="")  
  #except:
#    print ("Error connection")
#    return

 # c = conn.cursor()

  #curdate = str(datetime.datetime.now())

  #r = requests.post("http://emoncms.org/input/post?json={power:"+str(val[0])+"}&apikey=7ac88045c3f843febe32d0d75d7b8a72")
  #r = requests.post("http://emoncms.org/input/post?json={power:"+str(val[1])+"}&apikey=7ac88045c3f843febe32d0d75d7b8a72")
  #r = requests.post("http://emoncms.org/input/post?json={power:"+str(val[2])+"}&apikey=7ac88045c3f843febe32d0d75d7b8a72")

  #try:
    #c.execute('USE bolejka;') 
    #c.execute('INSERT INTO mesurements (mesuredatetime, sensorid, value) VALUES ("'+curdate+'",1,'+str(val[0])+');')
    #c.execute('INSERT INTO mesurements (mesuredatetime, sensorid, value) VALUES ("'+curdate+'",2,'+str(val[1])+');')
    #c.execute('INSERT INTO mesurements (mesuredatetime, sensorid, value) VALUES ("'+curdate+'",3,'+str(val[2])+');')
    #c.execute('SELECT * FROM mesurements;')
    #print c.fetchall()
  
  #except:
#    print ("Error command")
#    conn.close()
#    return
  

 # conn.commit()
  #conn.close()
 # return


if __name__ == "__main__":
  val = getserial() 
  print val
  #writetable(val)
