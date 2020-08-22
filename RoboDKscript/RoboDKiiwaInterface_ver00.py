
# Python script used for realtime control of
# KUKA iiwa robots from RoboDK simulation.

# Copyright: Mohammad Safeea, 22-August-2020
# Tested successfully using:    RoboDK v5.0.1
#                               Python v3.7.3
#                               iiwa 7R820

from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
from tkinter import *
from tkinter import messagebox
import _thread
import time
import socket
import struct
import math
import webbrowser
import csv

def close_program():
    global PublisherThreadFlag
    if(PublisherThreadFlag):
        title="Can't terminate program now"
        message='Robot is being controlled on-line, turn-off control by clicking the "Stop" button first'
        popupInfo(title,message)
    else:
        saveToolsData()
        root.destroy()
        
def saveToolsData():
    # Check input data
    Mass_Str=txt_Tool_Mass.get()
    COM_X_Str=txt_Tool_COM_x.get()
    COM_Y_Str=txt_Tool_COM_y.get()
    COM_Z_Str=txt_Tool_COM_z.get()
    temp=isNumeric(Mass_Str)
    if(temp[0]==False):
        return
    else:
        if((temp[1]>14) or (temp[1]<0)):
            return       
    com_dict={'X':COM_X_Str,'Y':COM_Y_Str,'Z':COM_Z_Str}
    for daKey in com_dict:
        temp=isNumeric(com_dict[daKey])
        maxVal=1000
        minVal=-1000
        if(temp[0]==False):
            return
        else:
            if((temp[1]>maxVal) or (temp[1]<minVal)):
                return
    daPath=RDK.getParam('PATH_OPENSTATION')
    try:
        path_file=os.path.join(daPath,'ToolsData.csv')
        file=open(path_file,'w')
        writer=csv.writer(file)
        temp=['Name','M (kg)','COM X','COM Y','COM Z']
        writer.writerow(temp)
        temp=['Tool_0',Mass_Str,COM_X_Str,COM_Y_Str,COM_Z_Str]
        writer.writerow(temp)
        file.close()
    except:
        print("An error happened while writing the Tool's data")

def popupInfo(title,message):
    messagebox.showinfo(title,message)

def popupError(title,message):
    messagebox.showerror(title,message)

def  popupMoveRobotToInitialPosition():
    tempTitle='Move Robot'
    tempMessage='The configurations of the real robot differes from the simulation'
    tempMessage=tempMessage+'. Do you want to move the real-robot to match the simulation'
    tempMessage=tempMessage+'\n MAKE SURE NOTHING IS IN COLLISON COURSE WITH THE ROBOT'
    tempMessage=tempMessage+'\n IF YES ROBOT WILL MOVE IN JOINTS SPACE'
    resposne=messagebox.askyesno(tempTitle,tempMessage)
    if resposne==1:
        pass
    else:
        pass

def convertFloatList2ByteArray(daList):
    ba=bytearray()
    for i in range(7):
        z=struct.pack('f',daList[i])
        for j in range(4):
            ba.append(z[j])
    return ba

def isNumeric(Val_Str):
    try:
        val=float(Val_Str)
        l=[True,val]
        return l
    except:
        l=[False]
        return l

def toString(val):
    return str(val)

def initializeRobotControl():
    global ROBOT_IP
    # Check input data
    Mass_Str=txt_Tool_Mass.get()
    COM_X_Str=txt_Tool_COM_x.get()
    COM_Y_Str=txt_Tool_COM_y.get()
    COM_Z_Str=txt_Tool_COM_z.get()
    temp=isNumeric(Mass_Str)
    if(temp[0]==True):
        if(temp[1]>14):
            daMessage='The mass of the tool shall not be more than 14 kg'
            print(daMessage)
            tempTitle='Error, the value of the tool mass is not valid'
            popupError(tempTitle,daMessage)
            return False
        elif(temp[1]<0):
            daMessage='The mass of the tool can not be a negative number'
            print(daMessage)
            tempTitle='Error, the value of the tool mass is not valid'
            popupError(tempTitle,daMessage)
            return False
    else:
        daMessage='The mass of the tool shall be a number from 0 to 14 kg'
        print(daMessage)
        tempTitle='Error, type error for the tool mass'
        popupError(tempTitle,daMessage)
        return False
    com_dict={'X':COM_X_Str,'Y':COM_Y_Str,'Z':COM_Z_Str}
    for daKey in com_dict:
        daMessage='Testing the value of COM '+daKey
        print(daMessage)
        temp=isNumeric(com_dict[daKey])
        maxVal=1000
        minVal=-1000
        if(temp[0]==True):
            if(temp[1]>maxVal):
                daMessage='The COM '+ daKey +' of the tool shall not exceed ' + toString(maxVal) + ' [mm]'
                print(daMessage)
                tempTitle="Error,  the value for the tool's COM "+ daKey +" is not valid"
                popupError(tempTitle,daMessage)
                return False
            elif(temp[1]<minVal):
                daMessage='The mass of the tool can not be be lessthan ' + toString(maxVal) + ' [mm]'
                print(daMessage)
                tempTitle="Error,  the value for the tool's COM "+ daKey +" is not valid"
                popupError(tempTitle,daMessage)
                return False
        else:
            daMessage='The COM '+daKey+' of the tool shall be a number from '
            daMessage=daMessage+toString(maxVal)
            daMessage=daMessage+' to '+toString(minVal)
            print(daMessage)
            tempTitle='Error, type error for the tool COM-' + daKey
            popupError(tempTitle,daMessage)
            return False    
    try:
        # Get tool data into a string
        str=Mass_Str+'_'
        str=str+COM_X_Str+'_'
        str=str+COM_Y_Str+'_'
        str=str+COM_Z_Str+'_'
        # Get initial joints positions for robot in simulation into a string
        q=item.Joints()
        qMemory=[]
        for i in range(7):
            qMemory.append(q[i,0])
            temp='{:.2f}'.format(q[i,0])
            if i==6:
                str=str+temp+'\n'
            else:
                str=str+temp+'_'                    
        # start a tcp/ip connection with the robot
        soc=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        soc.connect((ROBOT_IP,30003))
        time.sleep(1)
        # Send tool's inertial data (and initial angles of robot in
        # simulation) to the real KUKA iiwa
        bytes=str.encode()
        soc.send(bytes)
        time.sleep(0.2)
        # Read the confirmation message from the robot
        daBytes=soc.recv(1024)
        confirmationMessage=daBytes.decode('utf-8')
        print("Confirmation message recieved from the robot after"+
              "sending the initialization data is:")
        print(confirmationMessage)
        # Check the confirmation message
        if confirmationMessage.startswith("ack")==True:
            # Check that robot in the simulation did not move
            q=item.Joints()
            for k in range(7):
                error=qMemory[k]-q[k,0]
                if error*error>0.00001:
                    daMessage='Out of sync, the simulation robot shall not move '
                    daMessage=daMessage+'until the homing configuration is reached by the real robot'
                    print(daMessage)
                    tempTitle='Error, simluation robot pose changed while the robot is homing'
                    popupError(tempTitle,daMessage)
                    return False
            return True
        else:
            daMessage='Program was terminated from the SmartPad'
            print(daMessage)
            tempTitle='Error'
            popupError(tempTitle,daMessage)
            return False
    except:
        print('Error has happened while trying to stream tool inertial data')
        tempTitle='Comunication Error'
        tempMessage='TCPIP socket error, verify that:\n'
        tempMessage=tempMessage+'   1-You are on the same network as the robot\n'
        tempMessage=tempMessage+'   2-You have inserted the correct IP address of the robot\n'
        tempMessage=tempMessage+'If the previous is OK, the problem might be due to the firewall or an antivirus program\n'
        popupError(tempTitle,tempMessage)
        return False;
    
def updateJointAngle(threadName, var):
    global PublisherThreadFlag
    global ROBOT_IP
    tmpList=udpateJointTextGetAngleList()
    MESSAGE = convertFloatList2ByteArray(tmpList)
    try:
        sockUDP.sendto(MESSAGE, (ROBOT_IP, UDP_PORT))
        print('Successfully UDP publishing')
    except:
        print('Could not publish the UDP socket')
        PublisherThreadFlag=False
        tempTitle='Comunication Error'
        tempMessage='UDP socket error, verify that:\n'
        tempMessage=tempMessage+'   1-You are on the same network as the robot\n'
        tempMessage=tempMessage+'   2-You have inserted the correct IP address of the robot\n'
        tempMessage=tempMessage+'If the previous is OK, the problem might be due to the firewall or an antivirus program\n'
        popupError(tempTitle,tempMessage)
    while(PublisherThreadFlag):
        tmpList=udpateJointTextGetAngleList()
        MESSAGE = convertFloatList2ByteArray(tmpList)
        sockUDP.sendto(MESSAGE, (ROBOT_IP, UDP_PORT))
    print('Thread is aported')

def udpateJointTextGetAngleList():
    q=item.Joints()
    tmpList=[]
    for i in range(7):
        temp=q[i,0]*math.pi/180.0
        tmpList.append(temp)
        temp='{:.2f}'.format(q[i,0])
        txtsList[i].set(str(temp))
    return tmpList
    
def cmdStartControl():
    global PublisherThreadFlag
    global item
    global ROBOT_IP
    if(PublisherThreadFlag):
        print('Am already publishing')
        tempTitle="Can't perform operation"
        tempMessage="Already connected to the robot, can't effectuate two connection to same robot"
        popupInfo(tempTitle,tempMessage)
        return
    ROBOT_NAME=txt_ROBOT_NAME.get()
    try:
        item = RDK.Item(ROBOT_NAME)
        q=item.Joints()
        print('Robot id is retreived successfully')
    except:
        tempTitle='Robot name in RoboDK'
        tempMessage='Error could not find the item: '+ROBOT_NAME
        print(tempMessage)
        popupError(tempTitle,tempMessage)
        return
    ROBOT_IP=txt_ROBOT_IP.get()
    if initializeRobotControl():
        print('Tool data are updated successfully in the robot')
    else:
        return
    try:
       _thread.start_new_thread( updateJointAngle, ("Thread-1", 2, ) )
       PublisherThreadFlag=True
    except:
        print ("Error: unable to start thread")
        PublisherThreadFlag=False
    btnConnect.configure(state=DISABLED, background='cadetblue')
    btnDisConnect.configure(state=NORMAL, background=original_color)

def cmdExitControl():
    global PublisherThreadFlag
    PublisherThreadFlag=False
    btnConnect.configure(state=NORMAL, background=original_color)
    btnDisConnect.configure(state=DISABLED, background='cadetblue')

def cmdForceDisconnet():
    global PublisherThreadFlag
    PublisherThreadFlag=False
    
def select_a_Robot(message):
    print('Select a kuka iiwa robot')
    robotTemp=RDK.ItemUserPick('Select a robot',ITEM_TYPE_ROBOT)
    if not robotTemp.Valid():
        print(message)
        popupError('IIWA robot in RoboDK',message)
    else:
        tempStr=str(robotTemp)
        txt_ROBOT_NAME.set(tempStr)
          
def info_Window():
    window=Toplevel()
    daFrame=LabelFrame(window)
    daFrame.pack()
    txt='Copyright: Mohammad Safeea, August-2020 \n'
    txt=txt+'Plug-in used to control KUKA iiwa robot \n'
    txt=txt+'from RoboDK program.\n'
    txt=txt+'==============================.\n'
    txt=txt+'For more info check\n'
    txt=txt+'https://github.com/Modi1987\n';
    txt=txt+'==============================.\n'
    label=Label(daFrame,text=txt)
    label.pack()
    tempButton=Button(daFrame,text='ok', command=lambda: window.destroy())
    tempButton.pack()
    w_width=window.winfo_reqwidth()
    w_height=window.winfo_reqheight()
    s_width=window.winfo_screenwidth()
    s_height=window.winfo_screenheight()
    x_pos=int((s_width-w_width)/2)
    y_pos=int((s_height-w_height)/2)
    window.geometry("+{}+{}".format(x_pos,y_pos))
    window.lift()
    window.attributes('-topmost', True)    

def online_Help():
    daURL='https://github.com/Modi1987/'
    webbrowser.open(daURL, new=2)


txtsList=[]
UDP_PORT=30002
padxVal=5
padyVal=5
entryWidth=24
global PublisherThreadFlag
PublisherThreadFlag=False
sockUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
RDK = Robolink()

root=Tk()
root.title('iiwa control')
#root.geometry('400x600')
# create the menu
my_menu=Menu(root)
root.config(menu=my_menu)
# add the file menu
file_menu=Menu(my_menu)
file_menu.add_command(label='Connect',command=cmdStartControl)
file_menu.add_command(label='Disconnect',command=cmdExitControl)
file_menu.add_command(label='Force Disconnect',command=cmdForceDisconnet)
file_menu.add_separator()
my_menu.add_cascade(label='Robot',menu=file_menu)
file_menu.add_command(label='Exit',command=close_program)
# add the help menu
help_menu=Menu(my_menu)
my_menu.add_cascade(label='help',menu=help_menu)
help_menu.add_command(label='online help',command=online_Help)
help_menu.add_separator()
help_menu.add_command(label='About',command=info_Window)
robotInfo_Frame=LabelFrame(root,text='Robot info',padx=padxVal,pady=padyVal)
robotInfo_Frame.pack(padx=padxVal,pady=padyVal)
RowIndex=0
caption='Robot ID in RoboDK'
w=Label(robotInfo_Frame,text=caption)
w.grid(padx=1,row=RowIndex,column=0,sticky=W)
RowIndex=RowIndex+1
txt_ROBOT_NAME=StringVar()
w=Entry(robotInfo_Frame,textvariable=txt_ROBOT_NAME,width=entryWidth)
w.grid(padx=padxVal,row=RowIndex,column=0)
txt_ROBOT_NAME.set('No robot is available!!!')
message='No robot is available in RoboDK, add a kuka iiwa robot to the simulation then click on "select robot" button'
w=Button(robotInfo_Frame,text='select robot',command=lambda: select_a_Robot(message))
w.grid(padx=padxVal,row=RowIndex,column=1)
RowIndex=RowIndex+1
caption='Real robot IP'
w=Label(robotInfo_Frame,text=caption)
w.grid(padx=1,row=RowIndex,column=0,sticky=W)
RowIndex=RowIndex+1
txt_ROBOT_IP=StringVar()
w=Entry(robotInfo_Frame,textvariable=txt_ROBOT_IP,width=entryWidth)
w.grid(padx=padxVal,row=RowIndex,column=0,columnspan=2,sticky=W)
txt_ROBOT_IP.set('172.31.1.147')
toolInfo_Frame=LabelFrame(root,text='Tool inertial data',padx=padxVal,pady=padyVal)
toolInfo_Frame.pack(padx=padxVal,pady=padyVal)
RowIndex=0
caption='m [kg]'
w=Label(toolInfo_Frame,text=caption)
w.grid(padx=1,row=RowIndex,column=0,sticky=W)
caption='x [mm]'
w=Label(toolInfo_Frame,text=caption)
w.grid(padx=padxVal,row=RowIndex,column=1,sticky=W)
caption='y [mm]'
w=Label(toolInfo_Frame,text=caption)
w.grid(padx=padxVal,row=RowIndex,column=2,sticky=W)
caption='z [mm]'
w=Label(toolInfo_Frame,text=caption)
w.grid(padx=padxVal,row=RowIndex,column=3,sticky=W)
RowIndex=RowIndex+1
tempCharactersNum=6
txt_Tool_Mass=StringVar()
w=Entry(toolInfo_Frame,textvariable=txt_Tool_Mass,width=tempCharactersNum)
w.grid(padx=padxVal,row=RowIndex,column=0)
txt_Tool_Mass.set('0.0')
txt_Tool_COM_x=StringVar()
w=Entry(toolInfo_Frame,textvariable=txt_Tool_COM_x,width=tempCharactersNum)
w.grid(padx=padxVal,row=RowIndex,column=1)
txt_Tool_COM_x.set('0.0')
txt_Tool_COM_y=StringVar()
w=Entry(toolInfo_Frame,textvariable=txt_Tool_COM_y,width=tempCharactersNum)
w.grid(padx=padxVal,row=RowIndex,column=2)
txt_Tool_COM_y.set('0.0')
txt_Tool_COM_z=StringVar()
w=Entry(toolInfo_Frame,textvariable=txt_Tool_COM_z,width=tempCharactersNum)
w.grid(padx=padxVal,row=RowIndex,column=3)
txt_Tool_COM_z.set('0.0')
RowIndex=RowIndex+1

jAngles_Frame=LabelFrame(root,text='Joint angles',padx=padxVal,pady=padyVal)
jAngles_Frame.pack(padx=padxVal,pady=padyVal)
RowIndex=0
entryWidth=8
for i in range(7):
    jIndex=i+1
    caption='Joint ' + str(jIndex) + ' [deg]: '
    w=Label(jAngles_Frame,text=caption)
    w.grid(padx=padxVal,row=RowIndex,column=0)
    txt=StringVar()
    w=Entry(jAngles_Frame,textvariable=txt,width=entryWidth)
    w.grid(padx=padxVal,row=RowIndex,column=1)
    txtsList.append(txt)
    txt.set('')
    RowIndex=RowIndex+1

ctrlButtons_Frame=LabelFrame(root,text='',padx=padxVal,pady=padyVal)
ctrlButtons_Frame.pack(padx=padxVal,pady=padyVal)
RowIndex=0
btnConnect=Button(ctrlButtons_Frame,text='Connect',command=cmdStartControl)
btnConnect.grid(padx=padxVal,row=RowIndex,column=0)
btnDisConnect=Button(ctrlButtons_Frame,text='Disconnect',command=cmdExitControl)
btnDisConnect.grid(padx=padxVal,row=RowIndex,column=1)
original_color = btnConnect.cget("background")
btnDisConnect.configure(state=DISABLED, background='cadetblue')
RowIndex=RowIndex+1

copyRight=Button(root,text='Copyright: Mohammad Safeea', command=info_Window,bd=1,bg='blue',fg='white',anchor='w')
copyRight.pack(padx=20,pady=5,fill=BOTH)
status=Label(root,text='interface 4 iiwa control',bd=1,relief=SUNKEN,anchor='w')
status.pack(fill=BOTH)

message='No robot is available in RoboDK simulation, add a kuka iiwa robot to the simulation then click on "select robot" button'
select_a_Robot(message)
try:
    daPath=RDK.getParam('PATH_OPENSTATION')
    path_file=os.path.join(daPath,'ToolsData.csv')
    file=open(path_file,'r')
    reader=csv.reader(file)
    header=next(reader)
    print('CSV header is:')
    print(header)
    data1=[]
    for line in reader:
        print(line)
        if len(line)!=0:
            data1=line
            print(data1)
    lengthFlag=True
    if len(data1)!=5:
        print('Data curropted in file, list length error')
        lengthFlag=False
    numericFlag=True
    for val in data1[1:]:
        temp=isNumeric(val)
        if temp[0]==False:
            numericFlag=False
            print('Data curropted in file, value error')
            print(val)
    if numericFlag and lengthFlag:
        txt_Tool_Mass.set(data1[1])
        txt_Tool_COM_x.set(data1[2])
        txt_Tool_COM_y.set(data1[3])
        txt_Tool_COM_z.set(data1[4])
    file.close()
except:
    print("An error happened while reading the Tool's data")
# Notify user:
print('GUI is on!')
root.geometry("+{}+{}".format(80,15))
root.protocol("WM_DELETE_WINDOW",close_program)
root.lift()
root.attributes('-topmost', True)
root.mainloop()


