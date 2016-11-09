#!/usr/bin/env python
import rospy
from Tkinter import *
from sensor_msgs.msg import JointState
from threading import Thread

 
class jointgui():

    SCALE=10    
    pub=False
    endthread=False # variable for ending the trhead
    rospy.init_node('joint_gui')
    master = Tk()
    master.title('JOINT GUI')
    
    Frame1=Frame(master)
    #Frame2=Frame(master,bg="blue")
    #Frame3=Frame(Frame1,bg="green")
    
    Frame1.grid(row=2, column=0,columnspan=2, sticky="nsew")
    #Frame2.grid(row=0,column=0,columnspan=2,sticky="nsew")
    master.grid_rowconfigure(0, weight=1)  
    master.grid_columnconfigure(0, weight=1) 
    
    
    
    number_of_cables = rospy.get_param("/number_of_cables")
    
    current_joint_state=JointState()
    desired_joint_state=JointState()           
    q_current=number_of_cables*[0.0]
    q_desired=number_of_cables*[0.0]
    qdot_desired=number_of_cables*[0.0]
    torque_desired=number_of_cables*[0.0]
    
    PositionInputs=[]
    VelocityInputs=[]
    TorqueInputs=[]
    
    PositionOutputs=[]
    VelocityOutputs=[]
    TorqueOutputs=[]
    
    
    q_label=[] 
    
    q_val=[]
    qdot_val=[] 
    torque_val=[]
        
    def __init__(self):
         
        self.create_buttons(11,1) # function to create the 4 buttons for the gui
        
        row_head=2        
        row_q=3
        row_qdot=4
        row_tau=5
        column=1
        
        row_head_2=7  
        row_q_in=8
        row_qdot_in=9
        row_tau_in=10
                  

        for i in range(self.number_of_cables):         
            # Create displays for each motor
            self.q_val.append(StringVar())       
            self.qdot_val.append(StringVar())  
            self.torque_val.append(StringVar())  
            
            name="Joint "+str(i)
            self.create_heading(i,row_head,column,name)
            self.PositionOutputs.append(self.make_joint_display(i,row_q,column,self.q_val))            
            self.VelocityOutputs.append(self.make_joint_display(i,row_qdot,column,self.qdot_val))                     
            self.TorqueOutputs.append(self.make_joint_display(i,row_tau,column,self.torque_val))
            
            self.create_heading(i,row_head_2,column,name)
            self.PositionInputs.append(self.make_entry_button(i,row_q_in,column,True))
            self.VelocityInputs.append(self.make_entry_button(i,row_qdot_in,column,False))
            self.TorqueInputs.append(self.make_entry_button(i,row_tau_in,column,True))
            column=column+1
     
        self.main_heading(1,0,"Current Joint state",False)
        self.main_heading(6,0,"Desired Joint state",True)   
    def main_heading(self,r,c,name,desired):
        labelText=StringVar()
        labelText.set(name)
        if(desired): 
            labelDir=Label(self.Frame1, textvariable=labelText,height=1, width=20,font="-weight bold",bg="blue",relief=RAISED).grid(row=r,column=c, columnspan=3)
        else:
            labelDir=Label(self.Frame1, textvariable=labelText,height=1, width=20,font="-weight bold",bg="gray",relief=RAISED).grid(row=r,column=c, columnspan=3)
        
    def create_heading(self,i,r,c,name):
        labelText=StringVar()
        labelText.set(name)
        labelDir=Label(self.Frame1, textvariable=labelText,height=1, width=10).grid(row=r,column=c) 
        
    def make_joint_display(self,i,r,c,q_val):     
        q_val[i].set("Value")        
        Label_in_value_box=Label(self.Frame1, textvariable=q_val[i],relief=RIDGE,height=1,width=15,bg="white").grid(row=r,column=c)          
        return Label_in_value_box 
        
    def make_entry_button(self,i,r,c,isControllable): 
        directory=StringVar(None)
        if(not isControllable):
            entry=Entry(self.Frame1,textvariable=directory,width=15, relief=SUNKEN,bg="white")
        else:
            entry=Entry(self.Frame1,textvariable=directory,width=15, relief=SUNKEN,bg="blue")
        entry.insert(END,0.0)
        entry.grid(row=r,column=c)
        return entry 

    def create_buttons(self,r,c):       
        
        StartPub=[r,c]
        StopPub=[r+1,c]
        ValJoint=[r,c+1]
        FillJoint=[r+1,c+1]
        Quit=[r+1,c+2]
        
        self.StartPublishingJointState=Button(self.Frame1,text="Start",command=self.startpub,width=10).grid(row=StartPub[0],column=StartPub[1])
        self.StopPublishingJointState=Button(self.Frame1,text="Stop ",command=self.stoppub,width=10).grid(row=StopPub[0],column=StopPub[1])
        self.validatejointbutton=Button(self.Frame1,text="Validate",command=self.joint_event,width=10).grid(row=ValJoint[0],column=ValJoint[1])  
        self.filljointbutton=Button(self.Frame1,text="Fill",command=self.fill_event,width=10).grid(row=FillJoint[0],column=FillJoint[1])  
        self.quitbutton=Button(self.Frame1,text="Quit",command=self.exit_gui,bg="red",activebackground="red",width=10).grid(row=Quit[0],column=Quit[1]) 
       
       
        self.publisher = rospy.Publisher('/desired_joint_position', JointState, queue_size=5)
        rospy.Subscriber('/joint_state', JointState,self.JointStateCallback)        
        self.publishInfo=False
        self.publishDesiredInfo=False
    # Function for creating joint state display

        
    # joint state callback showing current motor positions   
    def JointStateCallback(self,msg):         
        self.current_joint_state=msg
        for i in range(self.number_of_cables):
            self.q_val[i].set(str(msg.position[i]))
            self.qdot_val[i].set(str(msg.velocity[i]))
            self.torque_val[i].set(str(msg.effort[i]))
            
    def start(self):
        self.master.mainloop()

    def fill_event(self):
        try:
            for i in range(self.number_of_cables):
                self.PositionInputs[i].delete(0,END)
                self.PositionInputs[i].insert(0,self.q_val[i].get())
                self.VelocityInputs[i].delete(0,END)
                self.VelocityInputs[i].insert(0,self.qdot_val[i].get())
                self.TorqueInputs[i].delete(0,END)
                self.TorqueInputs[i].insert(0,self.torque_val[i].get())                
        except ValueError:
            print "Bad input"    
             
  
                  
    def joint_event(self):
        try:
            for i in range(self.number_of_cables):
                self.q_desired[i]=float(self.PositionInputs[i].get())
                self.qdot_desired[i]=float(self.VelocityInputs[i].get())    
                self.torque_desired[i]=float(self.TorqueInputs[i].get()) 
             
        except ValueError:
            print "Bad input"        
            
    def startpub(self):
        self.publishInfo=True
    def stoppub(self):
        self.publishInfo=False     
        
                
    def loop(self):
        hz = 20
        r = rospy.Rate(hz)
        msg = JointState()
        for i in range(8):
            msg.name.append("q"+str(i+1))
            
        while not rospy.is_shutdown() and not self.endthread:
            msg.header.stamp=rospy.Time.now()
            msg.position = self.number_of_cables * [0.0]
            msg.velocity = self.number_of_cables * [0.0]
            msg.effort = self.number_of_cables * [0.0]
            for i in range(self.number_of_cables):
                msg.position[i]=self.q_desired[i]
                msg.velocity[i]=self.qdot_desired[i]
                msg.effort[i]=self.torque_desired[i]
            if(self.publishInfo):
                self.publisher.publish(msg)     
                
            self.master.update_idletasks()    
            r.sleep()         
            
            
    def exit_gui(self):
       print "quiting gui"
       self.endthread=True
       self.master.destroy()
       self.master.quit()
        
        

if __name__ == '__main__':

    gui=jointgui()    
    Thread(target=gui.loop).start()
    gui.start()
    
    
    
 
