import os
import math
import asyncio
import cozmo
import PIL.Image
import PIL.ImageFont
import PIL.ImageTk
import cv2
import numpy
import tkinter 
from tkinter import *
from tkinter import ttk
from cozmo.util import degrees, distance_mm, speed_mmps
import numpy as np

# <??? Run Configuration 
os.environ['COZMO_PROTOCOL_LOG_LEVEL'] = 'DEBUG'
os.environ['COZMO_LOG_LEVEL'] = 'DEBUG'
USE_LOGGING = False
WRITE_TO_FILE = False
SCALE=0.4
# TO INSTALL OPENCV ON MAC: <- ???
# brew install opencv3 --HEAD --with-contrib --with-python3
# to ~/.bash_profile add:
#   PYTHONPATH="/usr/local/Cellar/opencv3/HEAD-dd379ec_4/lib/python3.5/site-packages/:$PYTHONPATH"
#   export PYTHONPATH
# ???>
'''
Edge Test <- old comment
-Experimenting with Cozmo's camera and OpenCV
-Displays TKinter window with both pre and post-processed live camera feed
-Current frame can also written to file
@author Team Cozplay
'''
class EdgeTest:
    #The initialization of EdgeTest object
    def __init__(self):
        self._robot = None  # ???
        self.flagOK=0       # this flag is 1 after setup in "set_up_cozm
        self.ovalID=-1  
        self.ovalFrontID=-1
        self.Img=0
        self.x_init=0
        self.y_init=0
        self.initPosFlag=0
        self._tk_root =  tkinter.Tk() # tkinter for the input/output stream
        self._tk_root1 = tkinter.Tk() # tkinter for the map
        self._tk_label_input = 0      # widget Label, Tk variable 
        self._tk_label_output = 0     # widget Label, Tk variable 
        self._tk_plot = tkinter.Canvas(self._tk_root1,width=500, height=500) # widget Canvas, Tk variable
        #self.edg = 0
        #self.pil_edges = 0
        self.listOfObservedCubes=[]
        self.listOfObservedCubes_ovalID=[]
        # <???
        if USE_LOGGING:
            cozmo.setup_basic_logging()
        # ???>
        #connecting to cozmo
        cozmo.connect(self.run)
        
    async def set_up_cozmo(self, coz_conn):
        asyncio.set_event_loop(coz_conn._loop)         # ???
        self._robot = await coz_conn.wait_for_robot()  # ???
        self._robot.camera.image_stream_enabled = True # enables the video stream
        # < create event handlers for "EvtNewCameraImage" and "EvtObjectObserved"
        # the last parameters are event handling functions 
        self._robot.add_event_handler(cozmo.world.EvtNewCameraImage, self.on_new_camera_image)
        self._robot.add_event_handler(cozmo.objects.EvtObjectObserved, self.on_object_observed)
        # create event handlers for "EvtNewCameraImage" and "EvtObjectObserved">
        # < This code moves the Comzo's head 
        action1=self._robot.set_head_angle(cozmo.util.Angle(degrees=0))
        await action1.wait_for_completed()
        action1=self._robot.set_head_angle(cozmo.util.Angle(degrees=20))
        await action1.wait_for_completed()
        action1=self._robot.set_head_angle(cozmo.util.Angle(degrees=0))
        await action1.wait_for_completed()
        # This code moves the Comzo's head > 
        self.flagOK=1 # set the flag to 1

    # Image procssing function 
    # Auto-paramter Canny edge detection adapted from:
    # http://www.pyimagesearch.com/2015/04/06/zero-parameter-automatic-canny-edge-detection-with-python-and-opencv/
    def auto_canny(self, img, sigma=0.33):
        blurred = cv2.GaussianBlur(img, (3, 3), 0)
        v = numpy.median(blurred)
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edged = cv2.Canny(blurred, lower, upper)
        return edged

    # event handler for "EvtNewCameraImage"
    def on_new_camera_image(self, event, *, image:cozmo.world.CameraImage, **kw):
        raw_image = image.raw_image
        # Convert PIL Image to OpenCV Image
        # See: http://stackoverflow.com/questions/14134892/convert-image-from-pil-to-opencv-format
        cv2_image = cv2.cvtColor(numpy.array(raw_image), cv2.COLOR_RGB2BGR)
        # < openCV processing 
        # Example: edge filter + green line 
        # Edge filter 
        #if flag=1:
        #    self.Img=cv_image
        #    flag=0;
        end
        cv2_edges = self.auto_canny(cv2_image)
        # Gray image -> Color image
        edg=cv2.cvtColor(cv2_edges, cv2.COLOR_GRAY2RGB)
        # add line green line (10,10), (100,100)
        cv2.line(edg,(10,10),(100,100),(0,255,0),2)
        pil_edges = PIL.Image.fromarray(edg)
        # openCV processing>
        # < The map plot
        if  self.initPosFlag==1:
            thetaScreen=self._robot.pose.rotation.angle_z.radians # robot angle
            xScreen=self._robot.pose.position.x-self.x_init+400   # robot position on the map
            yScreen=self._robot.pose.position.y-self.y_init+400
            # robot's head
            xScreenFront=self._robot.pose.position.x-self.x_init+400+15*math.cos(thetaScreen)
            yScreenFront=self._robot.pose.position.y-self.y_init+400+15*math.sin(thetaScreen)
            if self.ovalID==-1:
                self.ovalID=self._tk_plot.create_oval(
                    (xScreen-20)*SCALE,(yScreen-20)*SCALE,
                    (xScreen+20)*SCALE,(yScreen+20)*SCALE,width=3)
                self.ovalFrontID=self._tk_plot.create_oval(
                    (xScreenFront-10)*SCALE,(yScreenFront-10)*SCALE,
                    (xScreenFront+10)*SCALE,(yScreenFront+10)*SCALE,width=1,fill="red")
            else:
                self._tk_plot.delete(self.ovalID)
                self._tk_plot.delete(self.ovalFrontID)
                self.ovalID=self._tk_plot.create_oval(
                    (xScreen-20)*SCALE,(yScreen-20)*SCALE,
                    (xScreen+20)*SCALE,(yScreen+20)*SCALE,width=3)
                self.ovalFrontID=self._tk_plot.create_oval(
                    (xScreenFront-10)*SCALE,(yScreenFront-10)*SCALE,
                  (xScreenFront+10)*SCALE,(yScreenFront+10)*SCALE,width=1,fill="red")
        #  The map plot >
        if self.flagOK==1:
            # Display input and output stream
            display_image_input = PIL.ImageTk.PhotoImage(image=image.annotate_image())
            display_image_output = PIL.ImageTk.PhotoImage(image=pil_edges)
            self._tk_label_input.imgtk = display_image_input
            self._tk_label_input.configure(image=display_image_input)
            self._tk_label_output.imgtk = display_image_output
            self._tk_label_output.configure(image=display_image_output)
            self._tk_root.update() # the window update 

    # event handler for "EvtObjectObserved"
    def on_object_observed(self, event, obj, pose, **kw):
        # Check the object id 
        if (obj.object_id >=1) & (obj.object_id <=3):
            # if the box was observed
            if obj.object_id in self.listOfObservedCubes:
                #draw the oval that corresponds to the cube
                indxTmp=self.listOfObservedCubes.index(obj.object_id)
                xScreen=obj.pose.position.x-self.x_init+400
                yScreen=obj.pose.position.y-self.y_init+400
                #draw the oval 
                self._tk_plot.coords(self.listOfObservedCubes_ovalID[indxTmp],
                                    (xScreen-10)*SCALE,(yScreen-10)*SCALE,
                                    (xScreen+10)*SCALE,(yScreen+10)*SCALE)
            else: #if the cube was not observed
                # add the cube to the list of observed cubes
                self.listOfObservedCubes.append(obj.object_id)
                xScreen=obj.pose.position.x-self.x_init+400
                yScreen=obj.pose.position.y-self.y_init+400
                #draw the oval 
                ovalID=self._tk_plot.create_oval(
                    (xScreen-10)*SCALE,(yScreen-10)*SCALE,
                    (xScreen+10)*SCALE,(yScreen+10)*SCALE,width=1,fill="blue")
                # add the oval to the list 
                self.listOfObservedCubes_ovalID.append(ovalID)                
                #print(self.listOfObservedCubes) <- debug
    
    async def run(self, coz_conn):
        # Set up Cozmo
        await self.set_up_cozmo(coz_conn)       
        self.x_init=self._robot.pose.position.x #initial position
        self.y_init=self._robot.pose.position.y #initial position
        #initial position flag        
        self.initPosFlag=1

        #packs the Tk widgets
        self._tk_label_input =  tkinter.Label(self._tk_root)  #input image
        self._tk_label_output = tkinter.Label(self._tk_root) #output image
        self._tk_label_input.pack() 
        self._tk_label_output.pack()
        self._tk_plot.pack()

        action1=self._robot.set_lift_height(1.0)
        await action1.wait_for_completed()
        action1=self._robot.set_lift_height(1.0)
        await action1.wait_for_completed()
        action1=self._robot.say_text('Cozmo', play_excited_animation=True)
        await action1.wait_for_completed()       
        print('Initialization Completed')
        
        action1= self._robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
        action2= self._robot.world.wait_until_observe_num_objects(num=3, object_type=cozmo.objects.LightCube, timeout=60)
        cubes=await action2
        action1.stop()
        max_dst, targ = 0, None
        for cube in cubes:
            translation = self._robot.pose - cube.pose
            dst = translation.position.x ** 2 + translation.position.y ** 2
            if dst > max_dst:
               max_dst, targ = dst, cube
        if len(cubes) < 3:
            print("Error: need 3 Cubes but only found", len(cubes), "Cube(s)")
        else:
            action3 = self._robot.pickup_object(targ, num_retries=3).wait_for_completed()
            await action3 
EdgeTest()

