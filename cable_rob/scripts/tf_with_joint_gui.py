#!/usr/bin/env python


import roslib
import rospy

import tf

from Tkinter import *
import tkFileDialog

import numpy as np
from sensor_msgs.msg import JointState
from threading import Thread
from tf2_msgs.msg import TFMessage
# Beginning of main program   



 
class jointgui():
    
    r=1
    
    pub=False

    SCALE=10
    rospy.init_node('tf_controller_gui')
    master = Tk()
    master.title('tf gui controller')
#    master.geometry('+%d+%d' % (24*SCALE, 18*SCALE))
#    master.minsize(width=24*SCALE, height=18*SCALE)
#    master.maxsize(width=24*SCALE, height=18*SCALE)
    wTp_desired=6*[0.0]
    wTp_current=6*[0.0]
    q_current=8*[0.0]
    q_desired=8*[0.0]
    torque=8*[0.0]
    QInputs=[]
    wTp_inputs=[]
    wTp_outputs=[]
    QOutputs=[]
    q_label=[] 
    wTp_label=[]
    dof=["x","y","z","rx","ry","rz"]
    endthread=False # variable for ending the trhead
    estimated_frame_name="estimated_platform_frame"
    
    def __init__(self):
        
        self.parent_frame = 'world'
        self.child_frame = 'desired_platform'
        self.master.grid_columnconfigure(1, weight=1)
        self.master.grid_columnconfigure(2, weight=1)
        self.master.grid_columnconfigure(3, weight=2)
        
        labelText=StringVar()
        labelText.set("Joint positions (deg)")
        labelDir=Label(self.master, textvariable=labelText,height=1, font="-weight bold").grid(row=0,column=1) 
        labelText.set("Estimated Location")
        labelDir=Label(self.master, textvariable=labelText,height=1, font="-weight bold").grid(row=0,column=5) 
        for i in range(8):            
            # We have to assign elements
            self.q_label.append(StringVar())     
            self.wTp_label.append(StringVar())     
                
            # Create displays for each motor        
            self.QOutputs.append(self.make_joint_display(i))
            self.QInputs.append(self.make_entry_button(i))

            if(i<6): # Create the tf functions                
                self.wTp_inputs.append(self.make_entry_button_for_transform(self.dof[i]))
                self.wTp_outputs.append(self.make_tf_display(i))
                
            self.r+=1
                          
        
        
        button_column=8
        button_row=1
        self.StartPublishingJointState=Button(text="Start publishing joint state",command=self.startpub).grid(row=button_row,column=button_column)
        button_row=button_row+1
        self.StopPublishingJointState=Button(text="Stop publishing joint state",command=self.stoppub).grid(row=button_row,column=button_column)
        button_row=button_row+1
        self.validatejointbutton=Button(text="Validate joint positions",command=self.joint_event).grid(row=button_row,column=button_column)  
        button_row=button_row+1
        self.SetDesiredTransform=Button(text="Publish Desired transform",command=self.gototransform).grid(row=button_row,column=button_column)        
        button_row=button_row+1               
        self.StopDesiredTransform=Button(text="Stop publishing Desired transform",command=self.stoptransform).grid(row=button_row,column=button_column)
        button_row=button_row+1
        self.validatebutton=Button(text="Validate Transformation ",command=self.validatepositions).grid(row=button_row,column=button_column) 
        button_row=button_row+1
        self.quitbutton=Button(text="Quit",command=self.exit_gui,bg="red",activebackground="red").grid(row=button_row,column=button_column) 
        


        
#        self.v.set("hello")
#        self.w=Label(self.master,textvariable=self.v,bg="white").grid(row=0,column=6)
        
        self.publisher = rospy.Publisher('/desired_joint_position', JointState, queue_size=5)
        rospy.Subscriber('/joint_state', JointState,self.JointStateCallback)
        rospy.Subscriber('/tf',TFMessage, self.tfCallback) # Subscribe to the robot IO states
        
        self.publishInfo=False
        self.publishDesiredInfo=False

       


    # Function for creating joint state display
    def make_joint_display(self,i):     
        name="q"+str(i)
        Label_next_to_value_box=StringVar()
        Label_next_to_value_box.set(name)
        Label_in_value_box=Label(self.master, textvariable=Label_next_to_value_box,height=4).grid(row=self.r,sticky='E')  
        self.q_label[i].set("Value")
        Label_in_value_box=Label(self.master, textvariable=self.q_label[i],height=1,width=15,bg="white").grid(row=self.r,column=1)        
        return Label_in_value_box 
        
    def make_entry_button(self,i): 
        directory=StringVar(None)
        entry=Entry(self.master,textvariable=directory,width=10)
        entry.grid(row=self.r,column=2)
        return entry 

    def make_entry_button_for_transform(self,name): 
        Label_next_to_entry_box=StringVar()
        Label_next_to_entry_box.set(name)
        Label_in_value_box=Label(self.master, textvariable=Label_next_to_entry_box,height=4).grid(row=self.r,column=3)        
        directory=StringVar(None)
        entry=Entry(self.master,textvariable=directory,width=10)
        entry.grid(row=self.r,column=4)   
        return entry 
        
    def make_tf_display(self,i):             
        self.wTp_label[i].set("Value")
        Label_in_value_box=Label(self.master, textvariable=self.wTp_label[i],height=1,width=20,bg="white").grid(row=self.r,column=5)        
        return Label_in_value_box     
        

    def tfCallback(self,msg):
        for i in range(len(msg.transforms)):
            if(msg.transforms[i].child_frame_id==self.estimated_frame_name):
                 self.wTp_label[0].set(str(msg.transforms[i].transform.translation.x)) # Reassigned label to current value
                 self.wTp_label[1].set(str(msg.transforms[i].transform.translation.y)) # Reassigned label to current value
                 self.wTp_label[2].set(str(msg.transforms[i].transform.translation.z)) # Reassigned label to current value
                 quaternion = (msg.transforms[i].transform.rotation.x, msg.transforms[i].transform.rotation.y,msg.transforms[i].transform.rotation.z,msg.transforms[i].transform.rotation.w)
                 euler = tf.transformations.euler_from_quaternion(quaternion)
                 self.wTp_label[3].set(euler[0]) # Reassigned label to current value
                 self.wTp_label[4].set(euler[1]) # Reassigned label to current value
                 self.wTp_label[5].set(euler[2]) # Reassigned label to current value
             #     ()
             #     (msg.transforms[i].transform.translation.y)
             #     (msg.transforms[i].transform.translation.z)

             
             #   (msg.transforms[i].transform.rotation.y)
             #   (msg.transforms[i].transform.rotation.z)
             #   (msg.transforms[i].transform.rotation.w))
                
            #for i in range(6):
            
            
        
    # joint state callback showing current motor positions   
    def JointStateCallback(self,msg):         
        for i in range(8):
            self.q_label[i].set(str(msg.position[i]))
            
    def start(self):
        self.master.mainloop()
                  
    def joint_event(self):
        try:
            for i in range(8):
                self.q_desired[i]=float(self.QInputs[i].get())                    
 
                    #self.v.set(str(self.q_desired[i]))
#                self.q1=float(self.q1input.get())
#                self.q2=float(self.q2input.get())
#                self.q3=float(self.q3input.get())
#                self.q4=float(self.q4input.get())
#                self.q5=float(self.q5input.get())
#                self.q6=float(self.q6input.get())
#                self.q7=float(self.q7input.get())
#                self.q8=float(self.q8input.get())                
        except ValueError:
            print "Bad input"        
            
    def gototransform(self): # Set the desired platform        
        self.publishDesiredInfo=True

    def stoptransform(self): # Set the desired platform
        self.publishDesiredInfo=False

    def startpub(self):
        self.publishInfo=True
    def stoppub(self):
        self.publishInfo=False     
        
    def validatepositions(self):
            try:
                for i in range(6):
                    self.wTp_desired[i]=float(self.wTp_inputs[i].get())                                  
            except ValueError:
                print "Bad input"    
                
    def loop(self):
        hz = 20
        r = rospy.Rate(hz)
        msg = JointState()
        for i in range(8):
            msg.name.append("q"+str(i+1))
            
        while not rospy.is_shutdown() and not self.endthread:
            br = tf.TransformBroadcaster()
            msg.header.stamp=rospy.Time.now()
            msg.position = 8 * [0.0]
            msg.effort = 8 * [0.0]
            self.torque[6]=0.1
            for i in range(8):
                msg.position[i]=self.q_desired[i]
                msg.effort[i]=self.torque[i]
            if(self.publishInfo):
                self.publisher.publish(msg)     
                
            if(self.publishDesiredInfo):
                br.sendTransform((self.wTp_desired[0],
                                 self.wTp_desired[1],
                                 self.wTp_desired[2]),
                                 tf.transformations.quaternion_from_euler(self.wTp_desired[3],
                                                                          self.wTp_desired[4],
                                                                          self.wTp_desired[5]),
                                 rospy.Time.now(),
                                 self.child_frame,
                                 self.parent_frame)
                # Add in the tf transform now
            
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
    
    
    
 