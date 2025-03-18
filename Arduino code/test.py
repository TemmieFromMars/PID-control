import serial
import numpy as np
from matplotlib import pyplot as plt
from IPython import display
import time


ser = serial.Serial('COM4', 9600, timeout=.1)
savefile = "pidtest.txt"
t0 = time.time()
tlist = []
temlist = [[] for i in range(6)]
outputlist = [[] for i in range(6)]
plt.figure(figsize=(20, 15), dpi=200)
lenlim = 500
writebuf = []
def plot_tem(face,t,tem):
    plt.subplot(4, 3, face+1)
    plt.title("T"+str(face+1))
    plt.plot(t[-lenlim:],tem[-lenlim:],"-o",markersize=1.4,label = "T")
def plot_output(face,t,output):
    plt.subplot(4, 3, face+7)
    plt.title("PWM amplitude "+str(face+1))
    plt.ylim(-5,260)
    plt.plot(t[-lenlim:],output[-lenlim:],"-o",markersize=1.4,label = "T")
def savedata(writebuf,filename):
    try:
        with open(filename, 'a') as file:
            file.write(str(writebuf)[1:-1]+'\n')  # Write each line followed by a newline character
    except IOError as e:
        print(f"An error occurred while writing to the file: {e}")
        
        
while True:
    try:
        
        if len(tlist)>lenlim:
            writebuf.append(tlist.pop(0))
            for i in range(6):
                writebuf.append(temlist[i].pop(0))     
            for i in range(6):
                writebuf.append(outputlist[i].pop(0)) 
            savedata(writebuf,savefile)
            writebuf=[]
        returndat = ser.readline() 
        if len(returndat) <66:
            continue
        t = time.time()-t0
        tlist.append(t)
        d = np.array(returndat.decode()[:-2].split(","),dtype=float)
        tem = d[::2]
        output = d[1::2]
        for face in range(6):
            
            temlist[face].append(tem[face])
            outputlist[face].append(output[face])
            plot_tem(face,tlist,temlist[face])
            plot_output(face,tlist,outputlist[face])
        #plt.legend()
        display.display(plt.gcf())
        display.clear_output(wait=True)
        plt.clf()
        #time.sleep(0.5)
    except KeyboardInterrupt:
        for k in range(len(tlist)):
            writebuf.append(tlist.pop(0))
            for i in range(6):
                writebuf.append(temlist[i].pop(0)) 
            for i in range(6):
                writebuf.append(outputlist[i].pop(0)) 
            savedata(writebuf,savefile)
            writebuf=[]
        #np.savetxt('readtest7.txt',voltage , delimiter=',')
        ser.close()
        plt.close()
        break