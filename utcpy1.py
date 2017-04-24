
"""
	Author: Myneni Venu Gopal.
	Designation: Associate ITS Engineer.
	Company: ITSPE.
	Location: Hyderabad, Indian Union, Asia, Planet Earth, Solar System, Milky Way Galaxy.
	Date: 23 November 2016.
	Last Updated Date: 2 December 2016.

	Purpose: This script is for Urban Traffic Control(UTC) using PSO algorithm and SUMO traffic
			simulator.
"""

###########################################################################
# Some of the modules needed for running the program are imported here.
import os, subprocess, sys, time, csv, math
import random
import numpy as np
import pandas as pd
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import math
import copy
import pickle
import anydbm
import xml.etree.ElementTree as etree
import time

sys.path.append("/usr/local/src/sumo-0.27.0/tools") # Computer Specific.
from sumolib import checkBinary
import traci
import traci.constants as tc

yellow_pos_dict =dict() # dictionary to store yellow light positions.
###########################################################################

#####################################################
# Initialization.
def create_base_dict(network):
	logic_dict =dict()
	"""
		Creates and returns a dictionary with trafficJunction ids as keys
		and split times as a list for each key.
		INPUT: sumo-networkfile.
		OUTPUT: a dictionary
	"""
	
	tree =etree.parse(network)
	root =tree.getroot()
	logic_list =root.findall("tlLogic")
	for i in logic_list:
		logic_dict[i.attrib["id"]]=[int(float(i.attrib["offset"]))]
		for j in i:
			ele =float(j.attrib["duration"])
			logic_dict[i.attrib["id"]].append(int(ele))
	return logic_dict

def create_first_swarm(network,n,lowlmt=5,uplmt=40):
	global yellow_pos_dict
	"""
		Creates and returns a list with dictinaries as elements.
		Each element is similar to logic_dict with differences in
		values which are randomly generated.
		INPUT:sumo-network file; no of particles of the swarm; min-green time; max-green time
		OUTPUT: a list with dictionaries as elements.(initial particles list)
	"""
	swarm_list =[]
	base_dict =create_base_dict(network)
	for i in range(n):
		if i==0:
			for key in base_dict.keys():
				yellow_pos_dict[key] =[]
				for i in range(0,len(base_dict[key])):
					if base_dict[key][i]==4:
						yellow_pos_dict[key].append(i)
			swarm_list.append(copy.deepcopy(base_dict))
		else:
			for key in base_dict.keys():
				for i in range(0,len(base_dict[key])):
					if base_dict[key][i]==4:
						pass
					else:
						base_dict[key][i] =random.randrange(lowlmt,uplmt)
			swarm_list.append(copy.deepcopy(base_dict))

	print yellow_pos_dict
	return swarm_list

def create_first_velocity(network, n, lowlmt=0, uplmt=2):
	swarm_list =[]
	"""
		Creates and returns a list with dictinaries as elements.
		Each element is similar to logic_dict with differences in
		values which are randomly generated.
		INPUT:sumo-network file; no of particles of the swarm; min of vel range; max of vel range
		OUTPUT: a list with dictionaries as elements.(initial velocities list)
	"""
	
	base_dict =create_base_dict(network)
	for i in range(n):
		for key in base_dict.keys():
			for i in range(0,len(base_dict[key])):
				base_dict[key][i] =random.randrange(lowlmt,uplmt)
		swarm_list.append(copy.deepcopy(base_dict))
	return swarm_list


def update_networkfile(network,adict):
	tree =etree.parse(network)
	"""
		Updates the network file by changing the splits of traffic junctions as
		given by adict(a dictionary).
		INPUT:sumo-network file; a dictionary with new split values.
		OUTPUT: just updates the sumo-network file, so returns nothing.
	"""
	root =tree.getroot()
	logic_list =root.findall("tlLogic")
	for ele in logic_list:
		if ele.attrib["id"] in adict.keys():
			ele.attrib["offset"]=str(adict[ele.attrib["id"]][0])
			for i in range(len(ele)):
				ele[i].attrib["duration"] =str(adict[ele.attrib["id"]][i+1])
	tree.write(network)

def get_fitness(outputfile):
	tree =etree.parse(outputfile)
	"""
		Takes in tripinfovalue file as input to calculate fitness.
		Returns a tuple (tot_time, no of vehicles which reached destination within sim time)
		INPUT: sumo file containing trip details after a sim has ended.
		OUTPUT: (tot_traveltime, no of vehicles completing the trip)
	"""
	root =tree.getroot()
	tot_time =0
	for i in root:
		tot_time +=float(i.attrib["duration"])
	return (tot_time, len(root))

def update_particle(currParticle, localBest, globalBest,currVelocity):
	global yellow_pos_dict
	"""
		This part is the core of our PSO Algorithm.
		Updates particle based on localBest, globalBest and velocity.
		Returns a new particle.
		INPUT:currParticle(a dictionary); localBest(a dictionary);
			   globalBest(a dictionary); currVelocity(a dictionary).
		OUTPUT:updatedParticle(a dictionary).
	"""
	newParticle =dict()
	outParticle =dict()

	for i in currVelocity.keys():

		x =currVelocity[i]
		y =[a-b for a,b in zip(localBest[i],currParticle[i])]
		z =[a-b for a,b in zip(globalBest[i],currParticle[i])]

		# w=1.2
		# u1 =random.uniform(0,2)
		# u2 =random.uniform(0,2)
		w=0.6571
		u1=0.6319
		u2=0.6239

		x =[a*w for a in x]
		y =[a*u1 for a in y]
		z =[a*u2 for a in z]

		x= [a+b+c for a,b,c in zip(x,y,z)] #updated velocity.
		currVelocity[i] =copy.deepcopy(x)

		newParticle[i] =[a+b for a,b in zip(x,currParticle[i])] #updated particle position.
		outParticle[i] =[]
		#ensuring that updated splits are within the range of [5,60]
		for k in newParticle[i]:
			if newParticle[i].index(k) in yellow_pos_dict[i]:
				outParticle[i].append(4)
			else:
				k =random.choice([math.ceil(k),math.floor(k)])
				if k<0:
					k=k*(-1)
				if k not in range(5,60):
					p=abs(k-5)
					q=abs(k-60)
					if p<q:
						k=5
					else:
						k=60
					outParticle[i].append(k)
				else:
					outParticle[i].append(k)


	print "particle changed from"
	print currParticle
	print "to particle"
	print outParticle
	return outParticle


########################################################################################
def run_one_simulation(steps =1200):
	PORT = 8813
	"""
		Runs one sumo simulation.
		INPUT:no of time steps(1 step = 1 second)
		OUTPUT:trip info file
	"""
	# The whole traci-sumo interface thing.
	if not traci.isEmbedded():
		sumoBinary = checkBinary("/usr/bin/sumo") # Computer Specific.
		sumoConfig = "/home/venu/Desktop/ATC/atc_files/utc/utc_01.sumo.cfg" # Computer Specific.
		if len(sys.argv) > 1:
			retCode = subprocess.call("%s -c %s --python-script %s" % (sumoBinary, sumoConfig, __file__), shell=True, stdout=sys.stdout)
			sys.exit(retCode)
		else:
			sumoProcess = subprocess.Popen("%s -c %s" % (sumoBinary, sumoConfig), shell=True, stdout=sys.stdout)
			traci.init(PORT,3)

	# Simulation code.
	step =0
	while step <steps:
		traci.simulationStep()
		step +=1
	traci.close()
##########################################################################

def run_full_simulation(network, outputfile,swarmSize, iterations=50):
	noofupdates =0
	updatesat =[]
	networkupdates =0
	"""
		Putting together every function defined above and running the whole program.
		Prints the best signal timings as determined by the simulation.
		INPUT:sumo-network file; sumo-trip info file; swarmsize;no of iterations.
		OUTPUT:prints the best signal timing set.
	"""
	fitness_plotter =[]
	fitness_tracker =dict()
	time0 =time.time()
	for i in range(iterations):
		time1 =time.time()
		print "Iteration Number = "+str(i)
		if i ==0:#for the first step only
			particle_list =create_first_swarm(network,swarmSize,lowlmt=5,uplmt=60)
			best_particle_list =[]# initialize list of best particles as empty. 
			best_particle_values =[]# initialze list of best local fitnesses as empty.
			curr_velocity_list =create_first_velocity(network,n=swarmSize,lowlmt=0,uplmt=1)
			for j in particle_list:
				update_networkfile(network, j) #should update network file before simulation.
				networkupdates+=1
				try:
					run_one_simulation(steps=1200) #finding fitness val.
				except:
					pass
				try:
					fitness =get_fitness(outputfile)[1]
				except:
					fitness =0
					print "fitness parsing failed for "+str(j)
				fitness_tracker[particle_list.index(j)]=[]
				fitness_tracker[particle_list.index(j)].append(fitness)
				print str(fitness) +" First Run Fitness Values for "+str(j)
				print "                    "
				print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
				best_particle_list.append(j) # initial best local particles.
				best_particle_values.append(fitness) #initial vales for best local particles.
			global_best_particle =best_particle_list[best_particle_values.index(max(best_particle_values))]
			current_particle_list =copy.deepcopy(best_particle_list)
			fitness_plotter.append(max(best_particle_values))
		


		else:
			for k in range(0,len(current_particle_list)):#updating before a new iteration.
				current_particle_list[k]=update_particle(current_particle_list[k],best_particle_list[k],
					global_best_particle,curr_velocity_list[k])
			for k in range(0,len(current_particle_list)):#new iteration.
				update_networkfile(network,current_particle_list[k])#updating the network.
				networkupdates+=1
				try:
					run_one_simulation(steps=1200)
				except:
					pass
				try:
					fitness =get_fitness(outputfile)[1]
				except:
					fitness =0
					print "fitness parsing failed for "+str(k)

				fitness_tracker[k].append(fitness)

				if fitness >best_particle_values[k]:
					noofupdates+=1
					updatesat.append(i)
					print "Updated Fitness = "+str(fitness) + "for particle "+ str(k)
					best_particle_values[k] =fitness
					best_particle_list[k] =current_particle_list[k]
			global_best_particle =best_particle_list[best_particle_values.index(max(best_particle_values))]
			fitness_plotter.append(max(best_particle_values))

		# print " best values so far"
		# print best_particle_values
		# print "Time Elapsed in minutes"
		# print (time.time()-time0)/60.0
		# print "Additional Time Required in minutes"
		# print (time.time()-time1)*(iterations-i)/60.0
	

	print fitness_plotter
	print "no of local best updates = "+str(noofupdates)
	print "no of network updates = "+str(networkupdates)
	print "updates for local bests occured at iteartion number = "+str(updatesat)
	print fitness_tracker

###########################################################################################################################################################
run_full_simulation("/home/venu/Desktop/ATC/atc_files/utc/utc_01.net.xml","/home/venu/Desktop/ATC/atc_files/utc/utc_01.summary.xml",100,10)










#####################################################

