#!/usr/bin/env python

import rospy

import time
import uav_trajectory
import numpy as np
import scipy
from scipy.integrate import odeint
import math
import subprocess
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pycrazyswarm import *


def calctraj(x0,v0):					## THIS FUNCTION USES AN ODE SOLVER TO
										## CALCULATE THE TRAJECTORY OF THE
										## DRONE. IT RETURNS XYZ POSITIONS
										## TO FOLLOW.

	init = np.append(x0,v0,axis=0)		# PUT INITIAL POSITION AND VELOCITY
										# INTO A SINGLE VECTOR	
	#print "init:", init
	tspan = np.linspace(0,4,num=50)		# TIMESPAN TO INTEGRATE OVER.

	sol = odeint(eomtraj,init,tspan)	# SOLVE ODE AS DEFINED IN eomtraj() 
										# FUNCTION

	positions = sol[:,0:3]				# REMOVE VELOCITIES FROM THE ARRAY.
	return positions

def eomtraj(x0,t):						## THIS FUNCTION DEFINES THE ODE TO
										## SOLVE.

	m = 10								# MASS 10 KG
	rho = 1.23							# AIR DENSITY KG/M^3
	Cd = 0.4							# COEFFICIENT OF DRAG
	A = .01								# FRONTAL AREA M^2
	g = 9.81							# ACCELERATION DUE TO GRAVITY M/S^2

	xdot = np.zeros([len(x0)])			# INITIALIZE STATE VECTOR
	xi = x0[0]
	yi = x0[1]
	zi = x0[2]
	vxi = x0[3]
	vyi = x0[4]
	vzi = x0[5]

	xdot[0] = vxi						# DEFINE ODE'S
	xdot[1] = vyi
	xdot[2] = vzi
	xdot[3] = -rho*Cd*A*math.pow(x0[3],2)/(2*m)
	xdot[4] = -rho*Cd*A*math.pow(x0[4],2)/(2*m)
	xdot[5] = (-m*g - np.sign(x0[5])*.5*rho*Cd*A*math.pow(x0[5],2))/m
	return xdot

'''def low_level_land(self):					## THIS FUNCTION LANDS THE DRONE
											## WITH LOW LEVEL CONTROL.
	rate = rospy.Rate(10)
	drone_landing_pose = self.position()
	
	while drone_landing_pose[2] > -.1:
		self.sp = drone_landing_pose
		drone_landing_pose[2] = drone_landing_pose[2]-0.005
		print "Landing", drone_landing_pose
		self.fly()
		rate.sleep()

 	print 'reached the floor, shutdown'   	
  	time.sleep(0.1)
 	return'''




if __name__ == '__main__':

	swarm = Crazyswarm()
	timeHelper = swarm.timeHelper
	allcfs = swarm.allcfs
	# INITIALIZE FIGURE
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	#ax2 = fig.add_subplot(122)

	print "program started"

	trajbal = uav_trajectory.Trajectory()
	height = 1.5			# DESIRED HOVER HEIGHT
	waittime = 0.75			# DURATION OF EACH goTo COMMAND. OBSOLETE 12 JUL 19 
	#rate = rospy.Rate(35)	# RATE OF LOOP ITERATIONS IN HZ.


	## EXECUTE TAKEOFF TO HOVER HEIGHT.
	allcfs.takeoff(targetHeight=height, duration=6.0)
	timeHelper.sleep(6.5)

	for cf in allcfs.crazyflies:
		pos = np.array([-1.0,0,height])
		cf.goTo(pos, 0, 3.0)
		timeHelper.sleep(3.5)

	for cf in allcfs.crazyflies:
		posdes = cf.position()		# STORE CURRENT POSITION FOR REFERENCE.

	delta = 0.05					# DISTANCE TO ACCOUNT FOR NOISE IN POSITION.

	k = 50							# SCALAR MULTIPLIER TO MODIFY INITIAL
									# VELOCITY IN ODE SOLVER.


	print 'Ready for slingshot'
	while not rospy.is_shutdown():
		# IF THE DISTANCE BETWEEN THE DRONE'S CURRENT POSITION
		# AND THE REFERENCE POSITION IS GREATER THAN delta.
		if np.linalg.norm(posdes - allcfs.crazyflies[0].position()) > delta:
			print "Slingshot mode"
			displace = posdes - allcfs.crazyflies[0].position()	# GET 3D VECTOR OF THE DISPLACEMENT.
			velinit = k * displace 					# SCALE DISPLACEMENT TO GET
													# INITIAL VELOCITY.
			traj = 0								# INITIALIZE TRAJECTORY VARIABLE.

			# USE A DO-WHILE LOOP WHEN CALCULATING TRAJECTORY TO ALLOW
			# FOR CONTINUOUS UPDATING.
			while True:
				# CALCULATE TRAJECTORY BASED ON DISPLACEMENT
				for cf in allcfs.crazyflies:
					# USE posdes AS INITIAL POSITION IN SOLVER
					# BECAUSE DRONE WILL RETURN TO THAT POSITION
					# BEFORE BEGINNING TRAJECTORY.
					traj = calctraj(posdes,velinit)

				#print "traj:",traj
				
				# WAIT UNTIL DRONE HAS BEEN RELEASED AND RETURNED
				# TO THE REFERENCE POSITION.
				time.sleep(0.5)
				if np.linalg.norm(allcfs.crazyflies[0].position() - posdes) < delta:
					print 'exiting slingshot mode'
					break

			#timeHelper.sleep(2.0)
			count = 0	# BEGIN A COUNTER FOR USE IN PLOTTING TRAJECTORY.


			# EXECUTE TRAJECTORY.
			for waypoint in range(len(traj[:,0])):
				if traj[waypoint,2] >= 0.05:
					count += 1
					
					#droneob.sp = traj[waypoint,:] + [0,0,0]
					#print 'fly to', traj[waypoint,:]
					#allcfs.goTo(traj[waypoint,:],0,waittime)
					#cf2.goTo(goal = traj[waypoint,:] + [0,0,0],yaw=0.0,duration=waittime,relative=False)
					#print "goalpos", traj[waypoint,:]
					#time.sleep(waittime)
					#rate.sleep()
				else:
					break


			trajexp = traj[0:count,:]
			trajexp[-1,2] = 0.1
			print 'Last waypoint:', trajexp[-1,:]
			print 'exporting csv'
			np.savetxt("trajexp.csv", trajexp, delimiter=",")

			print 'calculating polynomial'
			x = subprocess.call('./genTrajectory -i /home/derek/my-crazyflie/crazyswarm/ros_ws/src/crazyswarm/scripts/trajexp.csv --v_max 1.0 --a_max 1.0 -o polys.csv', shell=True, cwd='/home/derek/uav_trajectories/build/')
			print 'polynomial calcuated'

			trajbal.loadcsv('/home/derek/uav_trajectories/build/polys.csv')
			for cf in allcfs.crazyflies:
				cf.uploadTrajectory(0, 0, trajbal)

			print 'Beginning trajectory'

			TIMESCALE = 1.0

			for cf in allcfs.crazyflies:
				cf.startTrajectory(0, timescale=TIMESCALE)

			timeHelper.sleep(trajbal.duration*TIMESCALE + 0.25)

			allcfs.land(targetHeight=0.02, duration=4.0)

			timeHelper.sleep(5.0)

			#time.sleep(3.0)
			# BUILD TIME VECTOR FOR PLOTTING
			#tspan = np.linspace(0,4,num=500)
			# PREPARE PLOT
			#ax.plot(traj[0:count,0], traj[0:count,1], traj[0:count,2])
			# EXECUTE LANDING
			rospy.signal_shutdown('landed')
			#print 'reached the floor, shutdown'   	
			time.sleep(0.1)
	    
	

  	# SHOW PLOT. DO NOT DO THIS WHILE THE DRONE IS FLYING.
  	# IT WILL PAUSE THE PROGRAM.
  	#plt.show()



	#cf.goTo(goal = [0.0, 0.0, 1.0], yaw=0.2, duration = 2.0, relative = False)
	#time.sleep(4.0)

	#cf.goTo(goal = [0.0, 0.0, 0.0], yaw=0.5, duration = 5.0, relative = False)
	#for drone in swarm:
	#	drone.land(targetHeight = 0.0, duration = 3.0)
	#time.sleep(3.3)

	




	'''traj1 = uav_trajectory.Trajectory()
	traj1.loadcsv("takeoff.csv")

	traj2 = uav_trajectory.Trajectory()
	traj2.loadcsv("figure8.csv")

	#print(traj1.duration)
	for drone in swarm:
		drone.uploadTrajectory(0, 0, traj1)
		drone.uploadTrajectory(1, len(traj1.polynomials), traj2)

	for drone in swarm:
		#drone.startTrajectory(0, timescale=1.0)
		drone.takeoff(targetHeight=.5,duration=4.0)
	#time.sleep(traj1.duration * 2.0)
	time.sleep(6.0)

	
		#cf.startTrajectory(0, timescale=1.0)
	cf2.startTrajectory(0, timescale=1.0)
	time.sleep(traj1.duration * 1.5)

	cf1.startTrajectory(1, timescale=1.0)
	cf2.startTrajectory(1, timescale=1.0, reverse=True)
	#for drone in swarm:
	 #   drone.startTrajectory(1, timescale=2.0)
	time.sleep(traj2.duration * 2.0)

	#cf.startTrajectory(1, timescale=2.0)
	#time.sleep(traj2.duration * 2.0)

	for drone in swarm:
		#drone.startTrajectory(0, timescale=1.0, reverse=True)
		drone.land(targetHeight=0.0,duration=6.0)
	time.sleep(7.0)'''

	#cf.startTrajectory(0, timescale=1.0, reverse=True)
	#time.sleep(traj1.duration * 1.0)

	


###################################################################
######################MATLAB CODE##################################

'''
%% 1
% Calculate the trajectory of a drone given an initial velocity and
% direction
x0 = [0 0 0];
v0 = [10,10,10];
init = [ x0 v0 ];
tspan = [ 0 5 ];

[tout, yout] = ode45(@eomtraj,tspan,init);

for i = 1:length(yout(:,1))
	if yout(i,3) < 0
		yout(i,3) = 0;
	end
end

figure(1)
plot3(yout(:,1),yout(:,2),yout(:,3))
axis equal
hold on

index = find(yout(:,3)==0,2);
index = index(2);

newy = yout(1:index,1:3);

p = polyfitn(newy(:,1:2),newy(:,3),2);

model = polyvaln(p,newy(:,1:2));

plot3(newy(:,1),newy(:,2),model)



function xdot = eomtraj(t,x0)
m = 10;
rho = 1.23;
Cd = 0.4;
A = .01;
g = 9.81;

xdot = zeros(length(x0),1);
xi = x0(1);
yi = x0(2);
zi = x0(3);
vxi = x0(4);
vyi = x0(5);
vzi = x0(6);

xdot(1) = vxi;
xdot(2) = vyi;
xdot(3) = vzi;
xdot(4) = -rho*Cd*A*x0(4)^2/(2*m);
xdot(5) = -rho*Cd*A*x0(5)^2/(2*m);
xdot(6) = (-m*g - sign(x0(6))*.5*rho*Cd*A*x0(6)^2)/m;



end'''

#######################END MATLAB CODE##############################
####################################################################
