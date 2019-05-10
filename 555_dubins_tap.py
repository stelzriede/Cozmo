#
# Jacob Stelzriede
#
# *on start, cozmo will look around in place until a cube is found
# *once the cube is found it will flash green and wait for a tap.
# *once tapped, cozmo will navigate to that opject.
#           **  currently just moves to hard coded value
# **second part will be to add my own "go_to_pose" function from Dubins Kinematics
#
# ***third part would be to cycle through the cubes
#
#
#

import asyncio
import sys
import numpy
import math

import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps


class BlinkyCube(cozmo.objects.LightCube):
	'''Subclass LightCube and add a light-chaser effect.'''
	def __init__(self, *a, **kw):
		super().__init__(*a, **kw)
		self._chaser = None

	def start_light_chaser(self):
		'''Cycles the lights around the cube with 1 corner lit up green,
		changing to the next corner every 0.1 seconds.
		'''
		if self._chaser:
			raise ValueError("Light chaser already running")
		async def _chaser():
			while True:
				for i in range(4):
					cols = [cozmo.lights.off_light] * 4
					cols[i] = cozmo.lights.green_light
					self.set_light_corners(*cols)
					await asyncio.sleep(0.1, loop=self._loop)
		self._chaser = asyncio.ensure_future(_chaser(), loop=self._loop)

	def stop_light_chaser(self):
		if self._chaser:
			self._chaser.cancel()
			self._chaser = None

# Make sure World knows how to instantiate the subclass
cozmo.world.World.light_cube_factory = BlinkyCube

def cozmo_program(robot: cozmo.robot.Robot):

	if (robot.lift_height.distance_mm > 45):
		robot.set_lift_height(0.0, in_parallel=True).wait_for_completed()

	cube = None
	look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)

	try:
		cube = robot.world.wait_for_observed_light_cube(timeout=60)
	except asyncio.TimeoutError:
		print("Didn't find a cube :-(")
		return
	finally:
		print("Cube Found")
		look_around.stop()
	cube.start_light_chaser()
	
	try:
		print("Waiting for cube to be tapped")
		cube.wait_for_tap(timeout=10)
		print("Cube tapped")
		#Insert whatever action you wish to complete here.
		AC=30 
		# Set's the robot x and y to the center of the robot 
		angle_init=robot.pose.rotation.angle_z.radians
		x_init=robot.pose.position.x-AC*math.cos(angle_init)
		y_init=robot.pose.position.y-AC*math.sin(angle_init)
		#angle_init=0
		#x_init=-AC*math.cos(angle_init)
		#y_init=-AC*math.sin(angle_init)

		xC=0
		yC=0
		phiR=0

		# For extraction of seen light cube:
		xP=cube.pose.position.x #desired x pos
		yP=cube.pose.position.y #desired x pos
		angP=cube.pose.rotation.angle_z.radians #desired angle

		# For manual destination:
		#xP=-150      #desired x position
		#yP=-200     #desired y position
		#angP=(135)*math.pi/180  #desired angle

		# Control Variables
		kp=4/10
		gamma=1/5
		h=3/5
		e=math.sqrt(math.pow(yP-yC,2)+math.pow(xP-xC,2))
		e_init=e
		print("e start: ", e)
		recCoord=[]
		timeOutObj=cozmo.util.Timeout(60)
		print(e)
		# 'threshold' variable talked about in paper
		while ( e > 40):
			# Cozmo's heading angle and position
			phiCR=robot.pose.rotation.angle_z.radians
			xC=(robot.pose.position.x-AC*math.cos(phiCR)-x_init)
			yC=(robot.pose.position.y-AC*math.sin(phiCR)-y_init)

			# Cozmos heading from Initial angle
			phiR=phiCR-angle_init 
			phi=math.atan2(math.sin(phiR),math.cos(phiR))

			# Angle from ref heading
			theta=math.atan2(yP-yC,xP-xC)-angP;
			theta=math.atan2(math.sin(theta),math.cos(theta))
			#print(theta)
			# Angle between cozmo's heading and object
			alpha=theta-(phi-angP);
			alpha=math.atan2(math.sin(alpha),math.cos(alpha))
			#print(alpha)
			# distance from object
			e=math.sqrt(math.pow(yP-yC,2)+math.pow(xP-xC,2))
			#print("e in: ", e)
			# ?
			if (alpha == 0):
				omegaRef=kp*alpha+gamma*math.cos(alpha)*(alpha+h*theta)
			else:
				omegaRef=kp*alpha+gamma*math.cos(alpha)*math.sin(alpha)/alpha*(alpha+h*theta)
			#added if statement, was just the else condition before
			if (e/e_init > .3):
				vRef=gamma*math.cos(alpha)*e_init
			else:
				vRef=gamma*math.cos(alpha)*e
			#print(vRef)
			vL=(2*vRef-omegaRef*90.95)
			vR=(2*vRef+omegaRef*90.95)
			#print(vL, vR)
			print(robot.pose.position.x, ", ",robot.pose.position.y,", ",robot.pose.rotation.angle_z.radians)
			action1=robot.drive_wheel_motors(vL,vR)
			#robot.drive_wheel_motors(vL,vR)
			#print(xC, yC, phiR*180/math.pi)
			recCoord.append([xC, yC, phiR, theta, alpha, omegaRef, vRef, vL, vR])
			#await asyncio.sleep(0)
			#asyncio.sleep(0.1, loop=robot._loop)
		robot.stop_all_motors()
			#out=open("coordEx.txt","w")
			#csv_out=csv.writer(out,delimiter=",")
			#for row in recCoord:
			#    csv_out.writerow(row)
			#    out.close()

		# end of added action section
	except asyncio.TimeoutError:
		print("No-one tapped our cube :-(")
	finally:
		cube.stop_light_chaser()
		cube.set_lights_off()



cozmo.run_program(cozmo_program, use_viewer=True)

#        action = robot.go_to_object(cube, distance_mm(60.0))
#        action.wait_for_completed()