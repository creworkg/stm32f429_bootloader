'''
https://pythonhosted.org/pyserial/index.html
https://pythonhosted.org/pyserial/pyserial_api.html
https://docs.python.org/3/library/struct.html

make sure you have serial lib in your computer:

pip isntall pyserial

'''

import serial
import sys
import numpy as np
import struct as st
import os
import math
import platform as pf

FRAME_HEAD = 0xA5
FRAME_TAIL = 0xAA
FRAME_HEAD_LEN = 1
FRAME_TAIL_LEN = 1
FRAME_TAIL_LEN = 1
FRAME_TAIL_LEN = 1
FRAME_DATA_LEN = 1
FRAME_ID_0 = 0x00 #frame info
FRAME_ID_1 = 0x01 #kerenl ID
FRAME_ID_2 = 0x02 #total data len
FRAME_ID_3 = 0x03 

UPDATE_CMD = b"you need update"
UPDATE_CONFIRM = b"Oh,really?"
UPDATE_CHECKOUT = b"Oh,yes"
UPDATE_FINAL_CONFIRM = b"OK,come on"
update_protocol = (UPDATE_CMD, UPDATE_CONFIRM, UPDATE_CHECKOUT, UPDATE_FINAL_CONFIRM)

KERN_BIN_PATH = r'../build/STM32F429IG_BootLoader.bin'

frame_size = 300
frame = np.zeros(frame_size, dtype=np.ubyte)

#Determine the system type
sys_type = pf.system()
if sys_type == 'Linux':
    ser_port = '/dev/ttyUSB0'
elif sys_type == 'Windows':
    ser_port = 'COM4'
else:
    print('unknown system: ', sys_type)
    sys.exit('unknown system')
    
def make_frame(one_frame_data, data_len, p_head, frame_type):
    st.pack_into('<B', frame, 0, FRAME_HEAD)
    st.pack_into('<B', frame, 2, data_len)
    st.pack_into('<B', frame, data_len+3, FRAME_TAIL)

    if frame_type == FRAME_ID_2:
        #total data len
        st.pack_into('<c', frame, 1, FRAME_ID_2)
        st.pack_into('<L', frame, 3,one_frame_data)
        return
    elif frame_type == FRAME_ID_1:
        #kernel ID
        st.pack_into('<B', frame, 1, FRAME_ID_1)

    for i in range(data_len):
        frame[i] = one_frame_data[i + p_head]
    return
    



#open serial port
ser = serial.Serial(port=ser_port, baudrate=115200)
print(ser.name, ' connected')

'''
handshake
'''
#send cmd
ser.write(update_protocol[0])
#recv confirm
rx_buff = ser.read(len(update_protocol[1]))
print(rx_buff)
if rx_buff != update_protocol[1]:
    ser.close()
    sys.exit('error 0')


#send check out
ser.write(update_protocol[2])
#recv fianl confirm
rx_buff = ser.read(len(update_protocol[3]))
print(rx_buff)
if rx_buff != update_protocol[3]:
    ser.close()
    sys.exit('error 1')

'''
#send data
    #make frame
    #send frame
    #recv confirm
'''

#send total data len 
pure_data_sz = 128

#read bin
bin_file = np.fromfile(KERN_BIN_PATH, dtype=np.uint8, count=-1,sep='',offset=0)
dat_total_len = bin_file.shape[0]
make_frame(dat_total_len, 4, 1, FRAME_ID_2)
ser.write(frame)

crl = math.ceil(bin_file.shape[0]/pure_data_sz)
print('circle times:',crl)

dl = pure_data_sz
for i in range(0, crl):
    p_head = i * pure_data_sz
    p_tail = (i+1) * pure_data_sz - 1
    if i + 1 == crl:
        dl = dat_total_len - p_head
    make_frame(bin_file, dl, p_head, FRAME_ID_0)
    ser.write(frame)
    rx_buff = ser.read(1)
    if rx_buff == 'c' :
        print('frame %d sent' %i)
    else:
        pass
        

#print(rx_buff)

ser.close()

if __name__ == '__main__':
    print(update_protocol[0])
    
    
    
    
    