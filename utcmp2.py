###########################################################################
# Some of the modules needed for running the program are imported here.
import os, subprocess, sys, time, csv, math
import random
import numpy as np
import pandas as pd
import math
import copy
import pickle
import anydbm
import xml.etree.ElementTree as etree
import time
import datetime
import matplotlib.pyplot as plt
import threading
import multiprocessing as mp

sys.path.append("/usr/local/src/sumo-0.27.0/tools") # Computer Specific.
from sumolib import checkBinary
import traci
import traci.constants as tc

yellow_pos_dict =dict() # dictionary to store yellow light positions.
###########################################################################
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

###################################################################
def get_fitness(outputfile):
        print "parsing "+ outputfile
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
########################################################################################
def run_one_simulation(config_num =8813, steps =1200):
	PORT = config_num +8810
	"""
		Runs one sumo simulation.
		INPUT:no of time steps(1 step = 1 second)
		OUTPUT:trip info file
	"""
	# The whole traci-sumo interface thing.
	if not traci.isEmbedded():
		sumoBinary = checkBinary("/usr/bin/sumo") # Computer Specific.
		sumoConfig = "/home/venu/utcpso/UTC-PSO/"+str(config_num)+".sumo.cfg" # Computer Specific.
		print sumoConfig
		if len(sys.argv) > 1:
			retCode = subprocess.call("%s -c %s --python-script %s" % (sumoBinary, sumoConfig, __file__), shell=True, stdout=sys.stdout)
			sys.exit(retCode)
		else:
			#sumoProcess = subprocess.Popen("%s -c %s" % (sumoBinary, sumoConfig), shell=True, stdout=sys.stdout)
			sumoProcess = subprocess.Popen([sumoBinary,'-c', os.path.join(sumoConfig),'--remote-port', str(PORT)],stdout=sys.stdout, stderr=sys.stderr)
			traci.init(PORT,3)
			print str(PORT) + " is port"

	# Simulation code.
	step =0
	while step <steps:
		traci.simulationStep()
		step +=1
	traci.close()
##########################################################################

print "changed"

adict =dict()
for i in range(0,15):
        adict[i] =[]

#########################################################################################################################
update_counter =0
db =anydbm.open("plotVals.db","c")
for i in range(0,15):
        db[str(i)]=""
db.close()
##########################################################################################################################
def each_iteration(network,outputfile,k,v,cmn_current_particle_list,cmn_best_particle_values,cmn_best_particle_list,lock):
    
 
    update_networkfile(network, cmn_current_particle_list[k])
    try:
        run_one_simulation(k,steps =1200)
    except:
        pass
        


    try:
        fitness =get_fitness(outputfile)[1]
    except:
        fitness =-10
    
    fitness_tracker[k].append(fitness)
    adict[k].append(fitness)
    print adict
   
    if fitness >v.value:
        v.value =fitness
        print " v.value update " +str(v.value)

        
    if fitness >cmn_best_particle_values[k]:
        cmn_best_particle_values[k] =fitness
        cmn_best_particle_list[k] =cmn_current_particle_list[k]  
        print " COMMON FITNESS UPDATE " +str(cmn_best_particle_values[k])

################################################################################################   
""" Some Global Variables"""
particle_list =list() # A list of lists. Each item is an array of traffic_signal stage timings. Ex: [ [10,25,4,25,4,25,4], [0,30,4,30,4,30,4,30], ..........]
best_particle_list =list() # A list of lists. Each item is a local best array of traffic_signal stage timings. Similar to particle_list in structure.
best_particle_values =list() # A list of integers. Each item is the best_performance index for that traffic signal so far.
curr_velocity_list =list() # A list of lists. Similar to particle_list in structure.
fitness_tracker =dict()
fitness_plotter =list()
global_best_particle =list() # A list showing the traffic_signal stage timings with the maximum performance_index so far.
current_particle_list =list() # A list of lists. Each item is an array of current traffic_signal stage timings. Similar to particle_list in structure.

best_val =-20 # Seed value of the performance_index.
########################################### Only Run Once. #########################################

def initiation(network,outputfile,swarmSize):
    global particle_list, best_particle_list, best_particle_values, curr_velocity_list,failures
    global fitness_plotter, fitness_tracker
    global best_val
    
    
    particle_list =create_first_swarm(network, swarmSize, lowlmt =5, uplmt =60)
    curr_velocity_list =create_first_velocity(network, n =swarmSize, lowlmt =0, uplmt =1)

    counter =0

    for j in range(0,len(particle_list)):
        update_networkfile("network"+str(j)+".xml", particle_list[j])
        try:
            run_one_simulation(j)
        except:   
            counter +=1
            
        try:
            fitness =get_fitness("output"+str(j)+".xml")[1]
        except:
            fitness =0

        fitness_tracker[j]=[]
	fitness_tracker[j].append(fitness)

	best_particle_list.append(particle_list[j]) # initial best local particles.
	best_particle_values.append(fitness) #initial vales for best local particles.
    print "$$$$$$$$$$$$$$$$$$ B_P_Values @ Initiation $$$$$$$$$$"
    print best_particle_values
    print "$$$$$$$$$$$$$$$$$$$ B_P @ Initiation $$$$$$$$$$$$$$$$$$$$$$$$"
    
	
    global global_best_particle
    global_best_particle =best_particle_list[best_particle_values.index(max(best_particle_values))]
    
    
    print global_best_particle
    global current_particle_list
    print "$$$$$$$$$$$$$$$$$$$ Current_partivle_list $$$$$$$$$$$$$"
    current_particle_list =copy.deepcopy(best_particle_list)
    print current_particle_list
    global fitness_plotter
    fitness_plotter.append(max(best_particle_values))

    best_val =max(best_particle_values)
          
#######################################################################################################################################################
def run_full_simulation(network, outputfile,swarmSize, iterations=50):

        time1 =time.time() 
        v_list =[]

        global failures
        global best_val
        
	noofupdates =0
	updatesat =[]
	final_print =[]
	networkupdates =0
	"""
		Putting together every function defined above and running the whole program.
		Prints the best signal timings as determined by the simulation.
		INPUT:sumo-network file; sumo-trip info file; swarmsize;no of iterations.
		OUTPUT:prints the best signal timing set.
	"""
	fitness_plotter =[]
	fitness_tracker =dict()
	
	for i in range(iterations):
		if i ==0:#for the first step only
			initiation(network, outputfile, swarmSize)
			
			manager =mp.Manager()
                        cmn_current_particle_list =manager.list()

                        manager1 =mp.Manager()
                        cmn_best_particle_values =manager1.list()

                        manager2 =mp.Manager()
                        cmn_best_particle_list =manager2.list()

                        manager3 =mp.Manager()
                        cmn_global_best_particle =manager3.list()
                        cmn_global_best_particle.append(global_best_particle)

                        for item in best_particle_values:
                                cmn_best_particle_values.append(item)
                
                        for item in current_particle_list:
                                cmn_current_particle_list.append(item)

                        for item in best_particle_list:
                                cmn_best_particle_list.append(item)

                                
		
		else:
                        v =mp.Value("i",best_val)                 
                        lock =mp.Lock()
			for k in range(0,len(current_particle_list)):#updating before a new iteration.
				cmn_current_particle_list[k]=update_particle(cmn_current_particle_list[k],cmn_best_particle_list[k],
					cmn_global_best_particle,curr_velocity_list[k])
			processes =[mp.Process(target=each_iteration, args=("network"+str(k)+".xml","output"+str(k)+".xml", k,v,cmn_current_particle_list,cmn_best_particle_values,cmn_best_particle_list,lock)) for k in range(len(current_particle_list))]
			for p in processes:
				p.start()
			for p in processes:
				p.join()
		if i !=0:
                        best_val =v.value
                        print "Best so far, Round "+str(i) + " is "+str(best_val)
                        v_list.append(best_val)
  
		cmn_global_best_particle =cmn_best_particle_list[cmn_best_particle_values.index(max(cmn_best_particle_values))]
		fitness_plotter.append(max(cmn_best_particle_values))

	print " <<<<<<<<<<<<<<<<<<<< END OF SIMS RESULTS >>>>>>>>>>>>>>>>>>>>"
	print "--------------------------------------------------------------------------------"
	print "--------------------------------------------------------------------------------"
        print " v.value list is :"
	print v_list
	print "cmn_best list is: "
        print fitness_plotter
        print " best_particle is:"
        print cmn_global_best_particle
        time2 =time.time()
        print " Time (mins) taken for " +str(swarmSize*iterations) +" simulations is:"
        timeTak =math.ceil((time2 -time1)/60.0)
	print timeTak
	print "----------------------------------------------------------------------------------"
	print "-----------------------------------------------------------------------------------"

        # Stores test_results in a database"
	db =anydbm.open("atc_pso_test_results.db", "c")
	avalue =pickle.dumps(fitness_plotter)
	akey ="S"+str(swarmSize)+"I"+str(iterations)
	db[akey]=avalue
	db.close()

        # Plots comparative graphs
	plt.plot(v_list,linestyle ="--", marker ="*", color ="r", label="v_list")
	plt.plot(fitness_plotter,linestyle ="--", marker ="o", label="cmn_best list")
	plt.xlabel(" Simulation Steps ")
	plt.ylabel(" Performnace Index ")
	plt.title(akey+ "; Time: "+ str(timeTak) +" mins; " + str(swarmSize*iterations/timeTak))
        plt.legend(loc ="upper left")
	plt.show()

###########################################################################################################################################################
if __name__ =="__main__":
        run_full_simulation("/home/venu/utcpso/UTC-PSO/33_4l.net.xml","/home/venu/utcpso/UTC-PSO/33_4l.summary.xml",15,160)
#############################################################################################################################

