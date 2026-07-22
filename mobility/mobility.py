#Mo3D mobility simulation framework based on the Mo3 model as defined in:
#L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility Model for
#future generation mobile wireless networks", IEEE Access, Volume 10, April 1, 2022, pp. 34085 - 34115. DOI: 10.1109/ACCESS.2022.3161541
#and extended in:
#D. Ferretti, L. De Nardis and M.-G. Di Benedetto, "Mo3D - a Mobility Framework for Mobility Modeling in 3D Indoor Environments,"
#submitted to Software X, 2026.
import sys
import math
import random
import pdb
import copy
import re
import numpy as np
from pandas.core.frame import DataFrame
import matplotlib.pyplot as plt

from mobility.node import Node
from typing import List

from utils.plot_data import plot_obstacle_avoidance

class Mobility:
    """
        Create mobility vectors and initialize them.
    """
    def __init__(self, params, tot_number_of_nodes: int, areaLength, areaWidth, areaHeight, nodeList: List[Node], obstacleList: List):
        self.type = params.get('mobility').get('individual_mobility_model')
        self.dt=params.get('mobility').get('mobility_update_period')
        self.rng = np.random.default_rng()
        self.x_min=0
        self.x_max=areaLength
        self.y_min=0
        self.y_max=areaWidth
        self.z_min=0
        self.z_max=areaHeight
        self.nNodes=tot_number_of_nodes
        self.pathSave=params.get('mobility').get('pathSave')
        self.updateCounter=0;
    	#Input parameters shared by multiple modules
        self.v_min=params.get('mobility').get('v_min') #m/s
        self.v_max=params.get('mobility').get('v_max') #m/s
        self.g_max=params.get('mobility').get('g_max') #
        self.d_max=params.get('mobility').get('d_max') 
        self.a_max=params.get('mobility').get('a_max') #m/s^2 
        self.dMatrix=np.inf*np.ones((self.nNodes,self.nNodes))
        self.sameLineCollisionFlag=0
    	#Individual Mobility module input parameters
        self.T_IM=params.get('mobility').get('T_IM') #s 
    	
    	#Correlated Mobility module input parameters
        self.T_CM=params.get('mobility').get('T_CM') #s
        self.Dc=params.get('mobility').get('Dc')
        self.rho_min=params.get('mobility').get('rho_min')
        self.groupingStrategy=params.get('mobility').get('groupingStrategy') #1: centroid of group mates; 2: closest mate not in the connected set
        self.BMfilename=params.get('mobility').get('BMfilename')
        self.BM=np.eye(self.nNodes,self.nNodes,dtype=int)
        self.nextSwitchTime=0
        self.CM_BM_fPos=0
    	
    	#Collision Avoidance module input parameters
        self.T_CA=params.get('mobility').get('T_CA') #s
        self.d_CA_trigger=params.get('mobility').get('d_CA_trigger')
        self.d_CA_min=params.get('mobility').get('d_CA_min')
        self.theta_CA=params.get('mobility').get('theta_CA')
    	
    	#Obstacle Avoidance module input parameters
        self.T_OA=params.get('mobility').get('T_OA') #s
        self.d_OA_trigger=params.get('mobility').get('d_OA_trigger')
        self.theta_OA=params.get('mobility').get('theta_OA')
        self.OLfilename=params.get('mobility').get('OLfilename')
        self.ObsList=obstacleList;
        self.numObstacles=len(obstacleList)
    	
    	#Upper Bound Enforcement module input parameters
        self.T_UB=self.dt
    	
    	#Mobility features configuration
        self.correlatedMobilityFlag=params.get('mobility').get('correlatedMobilityFlag')
        self.collisionAvoidanceFlag=params.get('mobility').get('collisionAvoidanceFlag')
        self.obstacleAvoidanceFlag=params.get('mobility').get('obstacleAvoidanceFlag')
        self.upperBoundsEnforcementFlag=params.get('mobility').get('upperBoundsEnforcementFlag')
        self.tridimMobility=params.get('mobility').get('tridimMobility')
        #if self.tridimMobility==True:
        #    self.obstacleAvoidanceFlag=False
    		
    	#Initialization of speed vectors
        self.v=np.tile(0.0, self.nNodes)
        self.theta=np.tile(0.0, self.nNodes)
        self.theta_noOA=np.tile(0.0, self.nNodes)
        self.phi=np.tile(0.0, self.nNodes)

	    #Initialization of speed vectors to store the free mode settings when entering forced mode in CM
        self.CM_free_mode_v=np.tile(0.0, self.nNodes)
        self.CM_free_mode_theta=np.tile(0.0, self.nNodes)
        self.CM_free_mode_phi=np.tile(0.0, self.nNodes)
        self.CM_forced_mode=np.zeros(self.nNodes)
        
		#Initialization of positions
        self.x=np.tile(0.0, self.nNodes)
        self.y=np.tile(0.0, self.nNodes)
        self.z=np.tile(0.0, self.nNodes)
        for i in range(self.nNodes):
            node=nodeList[i]
            self.x[i] = node.get_coordinates()[0]
            self.y[i] = node.get_coordinates()[1]
            if self.tridimMobility==True:
                self.z[i] = node.get_coordinates()[2]
            else:
                self.z[i] = self.z_max/2
                node.set_coordinates(self.x[i],self.y[i],self.z[i])
 		#Initialization of path record      
        if self.pathSave==True:
            self.xPath=self.x.reshape((1,self.nNodes))
            self.yPath=self.y.reshape((1,self.nNodes))
            self.zPath=self.z.reshape((1,self.nNodes))           
        self.endSim=0;
        
    def generate_mobility_vectors(self):
    	for i in range(self.nNodes):
    		self.v[i]=self.v_min+(self.v_max-self.v_min)*self.rng.random()
    		self.theta[i]=-math.pi+2*math.pi*self.rng.random()
    		if self.tridimMobility==True:
    			self.phi[i]=-(math.pi/2)+2*(math.pi/2)*self.rng.random()
    		else:
    			self.phi[i]=0.0
    	#print('Initial speed vectors:')
    	#for i in range(self.nNodes):
        #    print('Node: ',i)
        #    print('Speed: ',self.v[i])
        #    print('theta: ',self.theta[i])
        #    print('phi: ',self.phi[i])
        
       		
    def individual_mobility(self):
        if self.type == 'Boundless':
            self.boundless()
        else:
            sys.exit('The ' + self.type + ' individual mobility model is not supported')

    def boundless(self):
        for i in range(self.nNodes):
			#Determine the new speed and check if it is within the allowed range [v_min,v_max]
            if self.a_max*self.T_IM>self.v_max:
                DeltaV=-(self.v_max-self.v_min)+2*(self.v_max-self.v_min)*self.rng.random()
            else:
                DeltaV=-self.a_max*self.T_IM+2*self.a_max*self.T_IM*self.rng.random()
            #breakpoint()
            self.v[i]=self.v[i]+DeltaV
            if self.v[i]<self.v_min:
                self.v[i]=self.v_min
            if self.v[i]>self.v_max:
                self.v[i]=self.v_max

			#Determine the new direction and check if it is within the allowed range [-pi,pi]
            DeltaTheta=-self.g_max*self.T_IM+2*self.g_max*self.T_IM*self.rng.random()
            self.theta[i]=self.theta[i]+DeltaTheta
            while self.theta[i]<-math.pi:
                self.theta[i]=self.theta[i]+2*math.pi
            while self.theta[i]>math.pi:
                self.theta[i]=self.theta[i]-2*math.pi

			#If 3D mobility is on, determine the new elevation and check if it is within the allowed range [-pi/2,pi/2]
            if self.tridimMobility==True:
                DeltaPhi=-self.d_max*self.T_IM+2*self.d_max*self.T_IM*self.rng.random()
                self.phi[i]=self.phi[i]+DeltaPhi
                if self.phi[i]<-math.pi/2:
                    self.phi[i]=-math.pi/2
                if self.phi[i]>math.pi/2:
                    self.phi[i]=math.pi/2

    def correlated_mobility(self):
        #self.BM[0,1]=1;
        #self.BM[0,2]=1;
        #self.BM[0,3]=1;        
        #self.BM[1,0]=1;
        #self.BM[1,2]=1;
        #self.BM[1,3]=1;
        #self.BM[2,0]=1;
        #self.BM[2,1]=1;
        #self.BM[2,3]=1;
        #self.BM[3,0]=1;
        #self.BM[3,1]=1;
        #self.BM[3,2]=1;
        #Example valid for 4 nodes with Binding Matrix
        #1 0 0 0
        #1 1 1 0
        #1 1 1 0
        #0 0 0 1
        groupingConditionVector=np.zeros(self.nNodes);
        for i in range(self.nNodes):
            self.dMatrix[i,i]=0# We set the distance of a node to itself to 0, in order to count it in its own connected set.
            #Step 1 - Determine bindings according to the current binding matrix
            groupMates=self.BM[i,:]
            groupMatesIDs=np.flatnonzero(groupMates)
            groupSize=groupMatesIDs.size
            #Step 2 - Check binding conditions and determine the connected set
            connectedSetIDs=self.dMatrix[i,groupMatesIDs]<=self.Dc
            connectedSet=np.flatnonzero(connectedSetIDs)
            connectedSetSize=connectedSet.size
            #Step 3 - Check grouping condition
            if groupSize==1: #If a node is alone in its group rho is automatically set to 1
                rho=1
            else:
                rho=(connectedSetSize-1)/(groupSize-1)
            #print('Node: ',i)
            #If the grouping condition is not satisfied adopt Forced mode
            if rho<self.rho_min:
                #print('Forced mode')
                #If entering Forced mode now, store the speed vector to restore it
                if self.CM_forced_mode[i]==0:
                    self.CM_forced_mode[i]=1
                    self.CM_free_mode_v[i]=copy.deepcopy(self.v[i])
                    self.CM_free_mode_theta[i]=copy.deepcopy(self.theta[i])
                    self.CM_free_mode_phi[i]=copy.deepcopy(self.phi[i])
                if self.groupingStrategy==1:
                    #Move toward the centroid - not described in the research paper. This approach determines the centroid of the positions of 
                    #the mates and sets the centroid as target destination
                    otherGroupMateIDs=np.flatnonzero(groupMatesIDs!=i)
                    xMates=self.x[otherGroupMateIDs]
                    yMates=self.y[otherGroupMateIDs]
                    zMates=self.z[otherGroupMateIDs]
                    xTarget=np.mean(xMates)
                    yTarget=np.mean(yMates)
                    zTarget=np.mean(zMates)
                    dTarget=np.sqrt(np.power(self.x[i]-xTarget,2)+np.power(self.y[i]-yTarget,2)+np.power(self.z[i]-zTarget,2))
                elif self.groupingStrategy==2:
                    #Move toward the closest mate not part of the connected
                    #set, defined as target destination
                    matesNotInTheSetIDs=self.dMatrix[i,groupMatesIDs]>self.Dc
                    matesNotInTheSet=np.flatnonzero(matesNotInTheSetIDs)
                    candidateTargets=groupMatesIDs[matesNotInTheSet]
                    dTarget=min(self.dMatrix[i,candidateTargets])
                    closestMateID=self.dMatrix[i,candidateTargets]==dTarget
                    closestMate=np.flatnonzero(closestMateID)
                    xTarget=self.x[candidateTargets[closestMate]]
                    yTarget=self.y[candidateTargets[closestMate]]
                    zTarget=self.z[candidateTargets[closestMate]]
                    #print('Target: ',candidateTargets[closestMate])
                #Set speed and direction based on the target destination
                self.v[i]=self.v_max;
                self.theta[i]=math.atan2(yTarget[0]-self.y[i],xTarget[0]-self.x[i]);
                self.phi[i]=math.asin((zTarget[0]-self.z[i])/dTarget);
                #breakpoint()
            else:
                #print('Free mode')
                #If leaving Forced mode now, restore the last free mode speed vector 
                if self.CM_forced_mode[i]==1:
                    self.CM_forced_mode[i]=0
                    self.v[i]=copy.deepcopy(self.CM_free_mode_v[i])
                    self.theta[i]=copy.deepcopy(self.CM_free_mode_theta[i])
                    self.phi[i]=copy.deepcopy(self.CM_free_mode_phi[i])
            self.dMatrix[i,i]=np.inf

    def upper_bounds_enforcement(self, v0, theta0, phi0):
        for i in range(self.nNodes):
			#Check on v
            vCheck=1
            maxDeltav=self.a_max*self.dt
            if maxDeltav<(self.v_max-self.v_min) and vCheck==1: #We only check v if violations can occur
                vOut=copy.deepcopy(self.v[i])
                #print("UB violation possible on v, with max=", maxDeltav)
                Deltav=vOut-v0[i]
                DeltavSign=np.sign(Deltav);
                if math.fabs(Deltav)>maxDeltav:
                    #print("UB violation on v, node ",i)
                    #print("Original v ",vOut)
                    #breakpoint()
                    Deltav=DeltavSign*maxDeltav
                    vOut=v0[i]+Deltav
                    #print("Corrected v ",vOut)
                    self.v[i]=copy.deepcopy(vOut)
                #else:
                	#print("No UB violation on v, deltav=", Deltav, ", maxDeltav=", maxDeltav)
    		#Check on theta
            maxDeltatheta=self.g_max*self.dt*0.999
            maxDeltathetaCheck=self.g_max*self.dt
            thetaCheck=1
            if maxDeltatheta<math.pi and thetaCheck==1: #We only check theta if violations can occur
                #print("UB violation possible on theta, with max=", maxDeltatheta)
                thetaOut=copy.deepcopy(self.theta[i])
                Deltatheta=math.fabs(self.theta[i]-theta0[i])
                if Deltatheta>math.pi:
                    Deltatheta=2*math.pi-Deltatheta;
                if Deltatheta>maxDeltatheta:
                    #print("UB violation on theta, node ",i)
                    #print("Original theta ",thetaOut)
                    #breakpoint()
                    if self.theta[i]>theta0[i]:
                        if ((self.theta[i]>0) and (theta0[i]>=self.theta[i]-math.pi))or(self.theta[i]<=0):
                            thetaOut=theta0[i]+maxDeltatheta
                        else:
                            thetaOut=theta0[i]-maxDeltatheta
                            while thetaOut<-math.pi:
                                thetaOut=thetaOut+2*math.pi        
                    else:
                        if ((theta0[i]>0) and (self.theta[i]>=theta0[i]-math.pi))or(theta0[i]<=0):
                            thetaOut=theta0[i]-maxDeltatheta
                        else:
                            thetaOut=theta0[i]+maxDeltatheta
                            while thetaOut>math.pi:
                                thetaOut=thetaOut-2*math.pi
                    #print("Corrected theta ",thetaOut)
                #else:
                	#print("No UB violation on theta, deltaTheta=", Deltatheta, ", maxDeltaTheta=", maxDeltatheta)
				#The application of UB on theta is tricky, so let's check if everything is ok...
                DeltathetaCheck=math.fabs(thetaOut-theta0[i])
                if DeltathetaCheck>math.pi:
                    DeltathetaCheck=2*math.pi-DeltathetaCheck
                if DeltathetaCheck>maxDeltathetaCheck:
                    sys.exit('Check after Upper Bound application failed')
                self.theta[i]=copy.deepcopy(thetaOut)
                
			#Check on phi
            maxDeltaPhi=self.d_max*self.dt;
            phiCheck=1
            if maxDeltaPhi<math.pi and phiCheck==1: #We only check phi if violations can occur
                phiOut=copy.deepcopy(self.phi[i])
                DeltaPhi=phiOut-phi0[i]
                DeltaPhiSign=np.sign(DeltaPhi)
                if math.fabs(DeltaPhi)>maxDeltaPhi:
                    #print("UB violation on phi, node ",i)
                    #breakpoint()
                    #DeltaPhi=DeltaPhiSign*maxDeltaPhi;
                    phiOut=phi0[i]+DeltaPhiSign*maxDeltaPhi;
                    if phiOut<-math.pi/2:
                        phiOut=-math.pi/2
                        #breakpoint()
                    if phiOut>math.pi/2:
                        phiOut=math.pi/2
                        #breakpoint()
                    self.phi[i]=copy.deepcopy(phiOut)
                #else:
                    #self.phi[i]=phi0[i]+DeltaPhi;

    def collision_avoidance_2D(self):
	#The Collision Avoidance module determines whether the trajectories of
	#nodes cross, and if the distance between nodes is < d_CA_min when
	#either of them is at the crossing point. If this is the case the speed
	#of nodes is increased or decreased as needed to ensure that the
	#distance above is at least d_CA_min.
	#This is the 2D version of the module, kept for reference, but replaced 
	#by the 3D version. If 3D mobility is disabled the two versions lead to the same 
	#results.
		#vOut=vIn;
		#thetaOut=theta;
        self.sameLineCollisionFlag=0;
        #vCheck=copy.deepcopy(self.v)
        for i in range(self.nNodes):
    	#PATH CROSSING IDENTIFICATION
    	#Step 1: determine the set of nodes that may potentially collide with node i.
            detectedNodesIDs=self.dMatrix[i,:]<self.d_CA_trigger;
            #breakpoint()
            thetaDetected=self.theta[detectedNodesIDs];
            vDetected=self.v[detectedNodesIDs];
            detectedNodes=np.flatnonzero(detectedNodesIDs)
            detectedNodesNum=detectedNodes.size
            dX=np.zeros(detectedNodesNum);
            dY=np.zeros(detectedNodesNum);
            #D=np.zeros(detectedNodesNum);
            intersectingNodes=np.full(detectedNodesNum, False)
            for j in range(detectedNodesNum):
                dX[j]=self.x[detectedNodes[j]]-self.x[i];
                dY[j]=self.y[detectedNodes[j]]-self.y[i];
                #D[j]=np.cos(thetaDetected[j])*np.sin(self.theta[i])-np.sin(thetaDetected[j])*np.cos(self.theta[i]);
                A=np.array([[np.cos(self.theta[i]), -np.cos(thetaDetected[j])],[np.sin(self.theta[i]), -np.sin(thetaDetected[j])]])
                Ab=np.array([[np.cos(self.theta[i]), -np.cos(thetaDetected[j]), dX[j]],[np.sin(self.theta[i]), -np.sin(thetaDetected[j]),dY[j]]])
                rankA=np.linalg.matrix_rank(A);
                rankAb=np.linalg.matrix_rank(Ab);
                if rankA<rankAb: #No solution: the two rays lie on parallel lines
                    continue #No collision risk, nothing to do
                else:
                    if rankA==2:#One solution: the two rays lie on intersecting lines
                        intersectingNodes[j]=True; #We add the node to the list of intersecting nodes, later we will check if the node should be added to the PCN set
                    else: #Infinite solutions: the two rays lie on the same line
                        if thetaDetected[j]==self.theta[i]:#rays in the same direction. Let's check if node i is ahead or behind node detectedNodes[j]
                            if (math.fabs(thetaDetected[j]-math.pi/2)>0.001) and (math.fabs(thetaDetected[j]-3*math.pi/2)>0.001):#We will use the cosine to check unless theta is too close to pi/2 or 3*pi/2
                               rCheck=dX/np.cos(thetaDetected[j]);
                            else:
                               rCheck=dY/np.sin(thetaDetected[j]);
                            if rCheck<0: #node i is ahead of node detectedNodes[j]
                            	if self.v[i]>=self.vDetected[j]: #node i is moving faster than node detectedNodes[j]
                            	    continue #No collision risk, nothing to do
                            	else: #node i is moving slower than node detectedNodes[j], so it could be reached, leading to a collision. Alter the direction of the node under consideration and exit
                            	    self.sameLineCollisionFlag=1;
                            	    self.theta[i]=self.theta[i]+self.theta_CA
                            	    return
                            else: #node i is behind node detectedNodes[j]
 	                            if self.v[i]>=self.vDetected[j]: #node i is moving faster than node detectedNodes[j], so it could reached it, leading to a collision. Alter the direction of the node under consideration and exit
 	                                self.sameLineCollisionFlag=1;
 	                                self.theta[i]=self.theta[i]+self.theta_CA;
 	                                return
 	                            else: #node i is moving slower than node detectedNodes[j]
 	                                continue #No collision risk, nothing to do
                        else: #rays in opposite directions. Let's check if they are heading one towards the other or speeding away.
                            if (math.fabs(thetaDetected[j]-math.pi/2)>0.001) and (math.fabs(thetaDetected[j]-3*math.pi/2)>0.001):#We will use the cosine to check unless theta is too close to pi/2 or 3*pi/2
                               rCheck=dX/np.cos(thetaDetected[j]);
                            else:
                               rCheck=dY/np.sin(thetaDetected[j]);
                            if rCheck<0: #nodes are speeding away
                                continue #No collision risk, nothing to do
                            else:# Nodes are heading one towards the other: risk of frontal collision 
                                self.sameLineCollisionFlag=1;
                                self.theta[i]=self.theta[i]+self.theta_CA;
                                return

            intersectingDetectedNodes=detectedNodes[intersectingNodes];
            thetaDetectedIntersecting=thetaDetected[intersectingNodes]
            dXIntersecting=dX[intersectingNodes]
            dYIntersecting=dY[intersectingNodes]
            intersectingDetectedNodesNum=intersectingDetectedNodes.size
            D=np.zeros(intersectingDetectedNodesNum);
            r=np.zeros(intersectingDetectedNodesNum);
            s=np.zeros(intersectingDetectedNodesNum);

            for j in range(intersectingDetectedNodesNum):
                D[j]=np.cos(thetaDetectedIntersecting[j])*np.sin(self.theta[i])-np.sin(thetaDetectedIntersecting[j])*np.cos(self.theta[i]);
                r[j]=(dYIntersecting[j]*np.cos(thetaDetectedIntersecting[j])-dXIntersecting[j]*np.sin(thetaDetectedIntersecting[j]))/D[j];
                s[j]=(dYIntersecting[j]*np.cos(self.theta[i])-dXIntersecting[j]*np.sin(self.theta[i]))/D[j];
            #breakpoint()
            collidingNodesIDs=np.logical_and.reduce((r>0, s>0, r<np.inf, s<np.inf));
            collidingNodes=np.flatnonzero(collidingNodesIDs)
            collidingNodesNum=collidingNodes.size
    	#Step 2: determine the crossing points and check if they fall in the
    	#movement area.
            xC=self.x[i]+np.cos(self.theta[i])*r[collidingNodes];
            yC=self.y[i]+np.sin(self.theta[i])*r[collidingNodes];
            relevantCollidingNodes=np.logical_and.reduce((xC>=self.x_min,xC<=self.x_max,yC>=self.y_min,yC<=self.y_max));
            #if relevantCollidingNodes.size>1:
            #    breakpoint()
            #relevantCollidingNodesIDs=collidingNodes[relevantCollidingNodes];
            relevantCollidingNodesIDs=detectedNodes[collidingNodes[relevantCollidingNodes]];

            if not relevantCollidingNodesIDs.any():#No possible collisions: the PCN set is empty, so let's skip to the next node to save time.
                continue
	    #SPEED BOUNDS IDENTIFICATION
            vLowerBounds=self.v_min*np.ones(relevantCollidingNodesIDs.size);
            vUpperBounds=self.v_max*np.ones(relevantCollidingNodesIDs.size);
		#Step 1: determine the time of arrival at the crossing points
            #breakpoint()
            diC=np.zeros(relevantCollidingNodesIDs.size);
            TiC=np.zeros(relevantCollidingNodesIDs.size);
            dothersC=np.zeros(relevantCollidingNodesIDs.size);
            TothersC=np.zeros(relevantCollidingNodesIDs.size);
            for j in range(relevantCollidingNodesIDs.size):
                diC[j]=np.sqrt(np.power(self.x[i]-xC[j],2)+np.power(self.y[i]-yC[j],2));
                TiC[j]=diC[j]/self.v[i];
                dothersC[j]=np.sqrt(np.power(self.x[relevantCollidingNodesIDs[j]]-xC[j],2)+np.power(self.y[relevantCollidingNodesIDs[j]]-yC[j],2));
                TothersC[j]=dothersC[j] / self.v[relevantCollidingNodesIDs[j]];
            #breakpoint()
            earlyNodes=TothersC<TiC;
            lateNodes=TothersC>TiC;
            simNodes=TothersC==TiC;
            #breakpoint()
            if any(simNodes):
                lowerIDSimNodes=np.logical_and(simNodes,(relevantCollidingNodesIDs<i));
                higherIDSimNodes=np.logical_and(simNodes,(relevantCollidingNodesIDs>i));
                earlyNodes=np.logical_or(earlyNodes,lowerIDSimNodes);
                lateNodes=np.logical_or(lateNodes,higherIDSimNodes);
		#Step 2: determine the bounds to ensure that the distance when one of
		#the nodes is at the crossing point is at least d_CA_min
            lateNodesIDs=relevantCollidingNodesIDs[lateNodes];
            earlyNodesIDs=relevantCollidingNodesIDs[earlyNodes];
            lateNodesIndexes=np.flatnonzero(lateNodes);
            earlyNodesIndexes=np.flatnonzero(earlyNodes);
            for l in range(lateNodesIDs.size):
                vLowerBounds[lateNodesIndexes[l]]=(self.v[lateNodesIDs[l]]*self.d_CA_min+diC[lateNodesIndexes[l]])/dothersC[lateNodesIndexes[l]];
            for l in range(earlyNodesIDs.size):
                vUpperBounds[earlyNodesIndexes[l]]=(self.v[earlyNodesIDs[l]]*(diC[earlyNodesIndexes[l]]))/(dothersC[earlyNodesIndexes[l]]+self.d_CA_min);
		#Step 3: find the vi' that satisfies all the bounds and is closest to
		#vi.
            lowerBound=max(vLowerBounds);
            upperBound=min(vUpperBounds);
            if lowerBound>upperBound:
			#We are in trouble, there is no solution. Let's set the speed to
			#the bound that solves most collision risks, and let's check again
			#at the next update
                if lateNodesIDs.size>earlyNodesIDs.size:
                    self.v[i]=min(lowerBound,self.v_max);
                    #vCheck[i]=min(lowerBound,self.v_max);
                else:
                    self.v[i]=max(upperBound,self.v_min);
                    #vCheck[i]=max(upperBound,self.v_min);
            else:
			#A range of allowed speeds exists. Let's check whether the current
			#speed is already within it.
                if self.v[i]>=lowerBound:
                    if self.v[i]<=upperBound:
                    	continue #no collision risk is identified, v[i] is already in the allowed range
                    else:
                        self.v[i]=upperBound; #going too fast, let's slow down to avoid collisions
                        #vCheck[i]=upperBound
                else:
            	    self.v[i]=lowerBound; #going too slow, let's speed up to avoid collisions		
            	    #vCheck[i]=lowerBound
        #print('vCheck:', vCheck)    

    def collision_avoidance(self):
	#The Collision Avoidance module determines whether the trajectories of
	#nodes cross, and if the distance between nodes is < d_CA_min when
	#either of them is at the crossing point. If this is the case the speed
	#of nodes is increased or decreased as needed to ensure that the
	#distance above is at least d_CA_min.
		#vOut=vIn;
		#thetaOut=theta;
        self.sameLineCollisionFlag=0;
        for i in range(self.nNodes):
            #print('i=',i)
    	#PATH CROSSING IDENTIFICATION
    	#Step 1: determine the set of nodes that may potentially collide with node i.
            detectedNodesIDs=self.dMatrix[i,:]<self.d_CA_trigger;
            #print('Detected nodes IDs= ',detectedNodesIDs)

            thetaDetected=self.theta[detectedNodesIDs];
            phiDetected=self.phi[detectedNodesIDs]; 
            vDetected=self.v[detectedNodesIDs]; 
            detectedNodes=np.flatnonzero(detectedNodesIDs)
            #print('Detected nodes = ',detectedNodes)
            detectedNodesNum=detectedNodes.size
            dX=np.zeros(detectedNodesNum);
            dY=np.zeros(detectedNodesNum);
            dZ=np.zeros(detectedNodesNum);
            l=np.zeros(detectedNodesNum);
            m=np.zeros(detectedNodesNum);
            n=np.zeros(detectedNodesNum);
            rMinDist=np.zeros(detectedNodesNum);
            sMinDist=np.zeros(detectedNodesNum);
            xMinDistI=np.zeros(detectedNodesNum);
            yMinDistI=np.zeros(detectedNodesNum);
            zMinDistI=np.zeros(detectedNodesNum);
            xMinDistJ=np.zeros(detectedNodesNum);
            yMinDistJ=np.zeros(detectedNodesNum);
            zMinDistJ=np.zeros(detectedNodesNum);
            relevantNodes=np.full(detectedNodesNum, False)
            closeNodes=np.full(detectedNodesNum, False)
            intersectingNodes=np.full(detectedNodesNum, False)
            #D=np.zeros(detectedNodesNum);
            l_i=np.cos(self.theta[i])*np.cos(self.phi[i]);
            m_i=np.sin(self.theta[i])*np.cos(self.phi[i]);
            n_i=np.sin(self.phi[i]);
            dirI=np.array([l_i, m_i,n_i]);
            for j in range(detectedNodesNum):
                #print('detectedNodes[j]=',detectedNodes[j])
                dX[j]=self.x[detectedNodes[j]]-self.x[i];
                dY[j]=self.y[detectedNodes[j]]-self.y[i];
                dZ[j]=self.z[detectedNodes[j]]-self.z[i];
                l[j]=np.cos(thetaDetected[j])*np.cos(phiDetected[j]);
                m[j]=np.sin(thetaDetected[j])*np.cos(phiDetected[j]);
                n[j]=np.sin(phiDetected[j]);
                dirJ=np.array([l[j], m[j],n[j]]);
                dS=np.array([dX[j], dY[j],dZ[j]])
                A=np.array([[l_i, -l[j]],[m_i, -m[j]],[n_i, -n[j]]])
                Ab=np.array([[l_i, -l[j], dX[j]],[m_i, -m[j], dY[j]],[n_i, -n[j], dZ[j]]] )
                rankA=np.linalg.matrix_rank(A);
                rankAb=np.linalg.matrix_rank(Ab);
                if rankA<rankAb: #No solution
                    if rankA==2: #The two rays lie on lines in different planes
                        #print('Rays on different planes');
                        #breakpoint()
                        aEq=np.dot(dirI,dirI);
                        bEq=np.dot(dirI,dirJ);
                        cEq=np.dot(dirJ,dirJ);
                        dEq=np.dot(dirI,dS);
                        eEq=np.dot(dirJ,dS);
                        #breakpoint()
                        rMinDist[j]=(-cEq*dEq+eEq*bEq)/(np.power(bEq,2)-aEq*cEq)
                        sMinDist[j]=(aEq/bEq)*(rMinDist[j])-dEq/bEq
                        xMinDistI[j]=self.x[i]+l_i*rMinDist[j];
                        yMinDistI[j]=self.y[i]+m_i*rMinDist[j];
                        zMinDistI[j]=self.z[i]+n_i*rMinDist[j];
                        xMinDistJ[j]=self.x[detectedNodes[j]]+l[j]*sMinDist[j];
                        yMinDistJ[j]=self.y[detectedNodes[j]]+m[j]*sMinDist[j];
                        zMinDistJ[j]=self.z[detectedNodes[j]]+n[j]*sMinDist[j];
                        dIJ=np.sqrt(np.power(xMinDistI[j]-xMinDistJ[j],2)+np.power(yMinDistI[j]-yMinDistJ[j],2)+np.power(zMinDistI[j]-zMinDistJ[j],2));
                        #breakpoint()
                        if dIJ< self.d_CA_min:
                            #print('Minimum distance below threshold: possible collision risk');
                            relevantNodes[j]=True; #The minimum distance between the rays is below the minimum threshold: there is a collision risk
                            closeNodes[j]=True;
                        else:
                            #print('No collision risk');
                            continue #No collision risk, nothing to do
                    if rankA==1: #The two rays lie on parallel lines
                        #print('Rays on parallel lines: no collision risk');
                        continue #No collision risk, nothing to do
                else:
                    if rankA==2:#One solution: the two rays lie on co-planar, intersecting lines
                        #print('Rays on the same plane and intersecting: possible collision risk');
                        relevantNodes[j]=True;
                        intersectingNodes[j]=True; #We add the node to the list of intersecting nodes, later we will check if the node should be added to the PCN set
                        #breakpoint()
                        Ainv=np.linalg.pinv(A)
                        solRays=np.dot(Ainv,dS);
                        rMinDist[j]=solRays[0];
                        sMinDist[j]=solRays[1];
                    else: #Infinite solutions: the two rays lie on the same line
                        #print('Rays on the same line: ');
                        if np.dot(dirI,dirJ)==1: #rays in the same direction. Let's check if node i is ahead or behind node detectedNodes[j]
                            if (l_i>0.0001):#We will use the l coefficient unless it is too small
                               rCheck=dX[j]/l_i
                            elif (m_i>0.0001):#We will use the m coefficient unless it is too small
                               rCheck=dY[j]/m_i;
                            else: #We will use the n coefficient
                               rCheck=dZ[j]/n_i;
                            #print('rCheck= ',rCheck)
                            if rCheck<0: #node i is ahead of node detectedNodes[j]
                            	if self.v[i]>=self.vDetected[j]: #node i is moving faster than node detectedNodes[j]
                            	    continue #No collision risk, nothing to do
                            	else: #node i is moving slower than node detectedNodes[j], so it could be reached, leading to a collision. Alter the direction of the node under consideration and exit
                            	    self.sameLineCollisionFlag=1;
                            	    self.theta[i]=self.theta[i]+self.theta_CA
                            	    return
                            else: #node i is behind node detectedNodes[j]
 	                            if self.v[i]>=self.vDetected[j]: #node i is moving faster than node detectedNodes[j], so it could reached it, leading to a collision. Alter the direction of the node under consideration and exit
 	                                self.sameLineCollisionFlag=1;
 	                                self.theta[i]=self.theta[i]+self.theta_CA;
 	                                #print('Potential collision risk, changing theta');
 	                                return
 	                            else: #node i is moving slower than node detectedNodes[j]
 	                                #print('No collision risk');
 	                                continue #No collision risk, nothing to do
                        else: #rays in opposite directions. Let's check if they are heading one towards the other or speeding away.
                            if (l_i>0.0001):#We will use the l coefficient unless it is too small
                               rCheck=dX[j]/l_i
                            elif (m_i>0.0001):#We will use the m coefficient unless it is too small
                               rCheck=dY[j]/m_i;
                            else: #We will use the n coefficient
                               rCheck=dZ[j]/n_i;
                            #print('rCheck= ',rCheck)    
                            if rCheck<0: #nodes are speeding away
                                #print('No collision risk');
                                continue #No collision risk, nothing to do
                            else:# Nodes are heading one towards the other: risk of frontal collision 
                                self.sameLineCollisionFlag=1;
                                self.theta[i]=self.theta[i]+self.theta_CA;
                                #print('Potential collision risk, changing theta');
                                return
            #print('xMinDistI = ',xMinDistI)
            #print('xMinDistJ = ',xMinDistJ)
            #print('Relevant nodes = ',relevantNodes)
            relevantDetectedNodes=detectedNodes[relevantNodes];
            #print('Relevant Detected nodes = ',relevantDetectedNodes)
            intersectingDetectedNodes=intersectingNodes[relevantNodes];
            closeDetectedNodes=closeNodes[relevantNodes];
            thetaDetectedRelevant=thetaDetected[relevantNodes]
            phiDetectedRelevant=phiDetected[relevantNodes]
            vDetectedRelevant=vDetected[relevantNodes]
            dXRelevant=dX[relevantNodes]
            dYRelevant=dY[relevantNodes]
            dZRelevant=dZ[relevantNodes]
            relevantDetectedNodesNum=relevantDetectedNodes.size
            D=np.zeros(relevantDetectedNodesNum);
            r=rMinDist[relevantNodes]
            s=sMinDist[relevantNodes]

            collidingNodesIDs=np.logical_and.reduce((r>0, s>0, r<np.inf, s<np.inf));
            #print('Colliding nodes IDs= ',collidingNodesIDs)   
            collidingNodes=np.flatnonzero(collidingNodesIDs)
            #print('Colliding nodes = ',collidingNodes)
            collidingNodesNum=collidingNodes.size
            xCI=(xMinDistI[relevantNodes])[collidingNodesIDs]
            yCI=(yMinDistI[relevantNodes])[collidingNodesIDs]
            zCI=(zMinDistI[relevantNodes])[collidingNodesIDs]                        
            xCJ=(xMinDistJ[relevantNodes])[collidingNodesIDs]
            yCJ=(yMinDistJ[relevantNodes])[collidingNodesIDs]
            zCJ=(zMinDistJ[relevantNodes])[collidingNodesIDs]
            #print('xCI = ',xCI)
    	#Step 2: determine the crossing points and check if they fall in the
    	#movement area.
            for j in range(collidingNodesNum):
    	        if intersectingDetectedNodes[collidingNodes[j]]: #this is a node on a trajectory that is co-planar with and intersecting the trajectory of node i
    	            xCI[j]=self.x[i]+l_i*r[collidingNodes[j]];
    	            yCI[j]=self.y[i]+m_i*r[collidingNodes[j]];
    	            zCI[j]=self.z[i]+n_i*r[collidingNodes[j]];
    	            xCJ[j]=xCI[j];
    	            yCJ[j]=yCI[j];
    	            zCJ[j]=zCI[j];    	    
#            xC=self.x[i]+np.cos(self.theta[i])*r[collidingNodes];
#            yC=self.y[i]+np.sin(self.theta[i])*r[collidingNodes];
            relevantCollidingNodes=np.logical_and.reduce((xCI>=self.x_min,xCI<=self.x_max,yCI>=self.y_min,yCI<=self.y_max,zCI>=self.z_min,zCI<=self.z_max,xCJ>=self.x_min,xCJ<=self.x_max,yCJ>=self.y_min,yCJ<=self.y_max,zCJ>=self.z_min,zCJ<=self.z_max));
            #if relevantCollidingNodes.size>1:
            #    breakpoint()
            #relevantCollidingNodesIDs=collidingNodes[relevantCollidingNodes];
            #if collidingNodesNum==0:
            #    breakpoint()
            #print('Relevant colliding nodes = ',relevantCollidingNodes)
            relevantCollidingNodesIDs=detectedNodes[collidingNodes[relevantCollidingNodes]];

            if not relevantCollidingNodesIDs.any():#No possible collisions: the PCN set is empty, so let's skip to the next node to save time.
                continue
	    #SPEED BOUNDS IDENTIFICATION
            vLowerBounds=self.v_min*np.ones(relevantCollidingNodesIDs.size);
            vUpperBounds=self.v_max*np.ones(relevantCollidingNodesIDs.size);
		#Step 1: determine the time of arrival at the crossing points
            #breakpoint()
            diC=np.zeros(relevantCollidingNodesIDs.size);
            TiC=np.zeros(relevantCollidingNodesIDs.size);
            dothersC=np.zeros(relevantCollidingNodesIDs.size);
            TothersC=np.zeros(relevantCollidingNodesIDs.size);
            for j in range(relevantCollidingNodesIDs.size):
                diC[j]=np.sqrt(np.power(self.x[i]-xCI[j],2)+np.power(self.y[i]-yCI[j],2)+np.power(self.z[i]-zCI[j],2));
                TiC[j]=diC[j]/self.v[i];
                dothersC[j]=np.sqrt(np.power(self.x[relevantCollidingNodesIDs[j]]-xCJ[j],2)+np.power(self.y[relevantCollidingNodesIDs[j]]-yCJ[j],2)+np.power(self.z[relevantCollidingNodesIDs[j]]-zCJ[j],2));
                TothersC[j]=dothersC[j] / self.v[relevantCollidingNodesIDs[j]];
            #breakpoint()
            earlyNodes=TothersC<TiC;
            lateNodes=TothersC>TiC;
            simNodes=TothersC==TiC;
            #breakpoint()
            if any(simNodes):
                lowerIDSimNodes=np.logical_and(simNodes,(relevantCollidingNodesIDs<i));
                higherIDSimNodes=np.logical_and(simNodes,(relevantCollidingNodesIDs>i));
                earlyNodes=np.logical_or(earlyNodes,lowerIDSimNodes);
                lateNodes=np.logical_or(lateNodes,higherIDSimNodes);
		#Step 2: determine the bounds to ensure that the distance when one of
		#the nodes is at the crossing point is at least d_CA_min
            lateNodesIDs=relevantCollidingNodesIDs[lateNodes];
            earlyNodesIDs=relevantCollidingNodesIDs[earlyNodes];
            lateNodesIndexes=np.flatnonzero(lateNodes);
            earlyNodesIndexes=np.flatnonzero(earlyNodes);
            for l in range(lateNodesIDs.size):
                vLowerBounds[lateNodesIndexes[l]]=(self.v[lateNodesIDs[l]]*self.d_CA_min+diC[lateNodesIndexes[l]])/dothersC[lateNodesIndexes[l]];
            for l in range(earlyNodesIDs.size):
                vUpperBounds[earlyNodesIndexes[l]]=(self.v[earlyNodesIDs[l]]*(diC[earlyNodesIndexes[l]]))/(dothersC[earlyNodesIndexes[l]]+self.d_CA_min);
		#Step 3: find the vi' that satisfies all the bounds and is closest to
		#vi.
            lowerBound=max(vLowerBounds);
            upperBound=min(vUpperBounds);
            if lowerBound>upperBound:
			#We are in trouble, there is no solution. Let's set the speed to
			#the bound that solves most collision risks, and let's check again
			#at the next update
                if lateNodesIDs.size>earlyNodesIDs.size:
                    self.v[i]=min(lowerBound,self.v_max);
                else:
                    self.v[i]=max(upperBound,self.v_min);
            else:
			#A range of allowed speeds exists. Let's check whether the current
			#speed is already within it.
                if self.v[i]>=lowerBound:
                    if self.v[i]<=upperBound:
                    	continue #no collision risk is identified, v[i] is already in the allowed range
                    else:
                        self.v[i]=upperBound; #going too fast, let's slow down to avoid collisions
                else:
            	    self.v[i]=lowerBound; #going too slow, let's speed up to avoid collisions		
        #print('v_CA:', self.v)  


    def load_binding_matrix(self):
    #Function loading the Binding Matrix used by the Correlated Mobility module
        print('Loading Binding Matrix');
        fileID=open(self.BMfilename)
        fileID.seek(self.CM_BM_fPos)
        nextSwitchTime=0
        fLine=fileID.readline()
        while (fLine[0]=='%') or (fLine[0]=='\n'): #Skip comments and empty lines
            fLine=fileID.readline()
        regex=r'\d+'
        regex2=r'[-+]?(?:\d*\.*\d+)'
        BM_line=re.findall(regex,fLine)
        self.BM[0,:]=[int(x) for x in BM_line]
        for i in range(1,self.nNodes):
            fLine=fileID.readline()
            BM_line=re.findall(regex,fLine)
            self.BM[i,:]=[int(x) for x in BM_line]
        #breakpoint()
        fLine=fileID.readline()
        while ((not (not fLine)) and ((fLine[0]=='%') or (fLine[0]=='\n'))): #Skip comments and empty lines
            fLine=fileID.readline()
        if not fLine:
            self.nextSwitchTime=-1 #File over, this was the last BM to be loaded
            print('File over, this was the last BM to be loaded');
        else:
            if fLine[0:10]=='nextSwitch':
                nextSwitchTimeStr=re.findall(regex2,fLine)
                self.nextSwitchTime=float(nextSwitchTimeStr[0])
            else:
                print('BM file formatting error')
                #breakpoint()
            self.CM_BM_fPos=fileID.tell()
        fileID.close()
        #breakpoint()

    def load_obstacle_list_2D(self):
    #Function loading the obstacle list used by the Obstacle Avoidance module of the Mo3 mobility model, as defined in
    #L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
    #future generation mobile wireless networks", submitted to IEEE Access
        print('Loading obstacle list file');
        self.ObsList.clear();
        self.numObstacles=0;
        fileID=open(self.OLfilename)
        fLine=fileID.readline()
        while (not (not fLine)):
            if((fLine[0]=='%') or (fLine[0]=='\n')): #Skip comments and empty lines
                fLine=fileID.readline()
            else:
                if fLine[0:8]=='Obstacle':
                    #regex2='[+-]?([0-9]*[.])?[0-9]+'
                    regex2=r'[-+]?(?:\d*\.*\d+)';
                    obsDataRE=re.findall(regex2,fLine)
                    #breakpoint()
                    obsData=[float(x) for x in obsDataRE]
                    #breakpoint()
                    if(not len(obsData)==5):
                	    print('Error in obstacle data format, skipping the line');
                    else:
                        self.ObsList.append(obsData)
                        self.numObstacles=self.numObstacles+1
                fLine=fileID.readline()
        fileID.close()    
        print('File over, ', self.numObstacles, ' obstacles found');

    def load_obstacle_list(self):
    #Function loading the obstacle list used by the Obstacle Avoidance module of the Mo3 mobility model, as defined in
    #L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
    #future generation mobile wireless networks", submitted to IEEE Access
        print('Loading obstacle list file');
        self.ObsList.clear();
        self.ObsHeightList.clear();
        self.numObstacles=0;
        fileID=open(self.OLfilename)
        fLine=fileID.readline()
        while (not (not fLine)):
            if((fLine[0]=='%') or (fLine[0]=='\n')): #Skip comments and empty lines
                fLine=fileID.readline()
            else:
                if fLine[0:8]=='Obstacle':
                    #regex2='[+-]?([0-9]*[.])?[0-9]+'
                    regex2=r'[-+]?(?:\d*\.*\d+)';
                    obsDataRE=re.findall(regex2,fLine)
                    #breakpoint()
                    obsData=[float(x) for x in obsDataRE]
                    #breakpoint()
                    if(not len(obsData)==7):
                	    print('Error in obstacle data format, skipping the line');
                    else:
                        self.ObsList.append(obsData)
                        self.numObstacles=self.numObstacles+1
                fLine=fileID.readline()
        fileID.close()    
        print('File over, ', self.numObstacles, ' obstacles found');

    def rectangle_min_distance(self,obsIndex):
    #Function determining the minimum distance from a rectangular obstacle as part of the Obstacle Avoidance module
    #of the Mo3 mobility model, as defined in
    #L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
    #future generation mobile wireless networks", submitted to IEEE Access
        detected=np.zeros(self.nNodes)
        xmin=self.ObsList[obsIndex][1]-self.ObsList[obsIndex][4]/2;
        xmax=self.ObsList[obsIndex][1]+self.ObsList[obsIndex][4]/2;
        ymin=self.ObsList[obsIndex][2]-self.ObsList[obsIndex][5]/2;
        ymax=self.ObsList[obsIndex][2]+self.ObsList[obsIndex][5]/2;
        zmin=self.ObsList[obsIndex][3]-self.ObsList[obsIndex][6]/2;
        zmax=self.ObsList[obsIndex][3]+self.ObsList[obsIndex][6]/2;
        for i in range(self.nNodes):
            try:
                del minDistance  # Remove any previous value
            except NameError:
                pass  # It wasn't defined before
            #breakpoint()
            if self.z[i]>=zmax: #The UE is at a height higher than the height of an obstacle placed on the floor
                if (self.x[i]>xmin) and (self.x[i]<=xmax) and (self.y[i]>ymin) and (self.y[i]<=ymax): #The UE is exactly above the obstacle   
                   if self.phi[i]<0: #The UE is going down, we might collide with the obstacle from above...
                       minDistance=np.sqrt(np.power(self.z[i]-zmax,2))
                       if minDistance<=self.d_OA_trigger: #We can detect the obstacle
                           self.phi[i]=0 #We stabilize the height (but the UB module might change this if the direction correction is too wide)
                
                continue #We can skip the rest, we will not change theta
            if self.z[i]<=zmin: #The UE is at a height lower than the height of an obstacle hanging from the ceiling
                if (self.x[i]>xmin) and (self.x[i]<=xmax) and (self.y[i]>ymin) and (self.y[i]<=ymax): #The UE is exactly below the obstacle   
                   if self.phi[i]>0: #The UE is going up, we might collide with the obstacle from below...
                       minDistance=np.sqrt(np.power(self.z[i]-zmin,2))
                       if minDistance<=self.d_OA_trigger: #We can detect the obstacle
                           self.phi[i]=0 #We stabilize the height (but the UB module might change this if the direction correction is too wide)
                
                continue #We can skip the rest, we will not change theta
                
            if (self.x[i]<=xmin) and (self.y[i]<=ymin):#BottomLeft nodes
                minDistance=np.sqrt(np.power(self.x[i]-xmin,2)+np.power(self.y[i]-ymin,2))
            if (self.x[i]<=xmin) and (self.y[i]>ymin) and (self.y[i]<=ymax): #Left nodes
                minDistance=np.sqrt(np.power(self.x[i]-xmin,2))
            if (self.x[i]<=xmin) and (self.y[i]>ymax): #TopLeft nodes
                minDistance=np.sqrt(np.power(self.x[i]-xmin,2)+np.power(self.y[i]-ymax,2))
            if (self.x[i]>xmin) and (self.x[i]<=xmax) and  (self.y[i]<=ymin): #Bottom nodes
                minDistance=np.sqrt(np.power(self.y[i]-ymin,2))
            if (self.x[i]>xmin) and (self.x[i]<=xmax) and  (self.y[i]>ymax): #Top nodes
                minDistance=np.sqrt(np.power(self.y[i]-ymax,2))
            if (self.x[i]>xmax) and  (self.y[i]<=ymin): #BottomRight nodes
                minDistance=np.sqrt(np.power(self.x[i]-xmax,2)+np.power(self.y[i]-ymin,2))
            if (self.x[i]>xmax) and  (self.y[i]>ymin) and (self.y[i]<=ymax): #Right nodes
                minDistance=np.sqrt(np.power(self.x[i]-xmax,2))
            if (self.x[i]>xmax) and  (self.y[i]>ymax): #TopRight nodes
                minDistance=np.sqrt(np.power(self.x[i]-xmax,2)+np.power(self.y[i]-ymax,2))
            try:
                minDistance
            except NameError:
                print('Obstacle: index=', obsIndex,' xmin=', xmin,' xmax=',xmax,'ymin=', ymin,' ymax=',ymax,' zmin=',zmin,' zmax=',zmax,'. UE id: ',i,' Position: x=',self.x[i],' y=',self.y[i],' z=',self.z[i]);
                breakpoint();           
    # Do something.
            detected[i] = minDistance<=self.d_OA_trigger;
            #if detected[i]:
            #    print('Obstacle detected: distance=',minDistance,' index=', obsIndex,' xmin=', xmin,' xmax=',xmax,'ymin=', ymin,' ymax=',ymax,' zmin=',zmin,' zmax=',zmax,'. UE id: ',i,' Position: x=',self.x[i],' y=',self.y[i],' z=',self.z[i]);
            #else:
            #    print('Obstacle not detected: distance=',minDistance,' index=', obsIndex,' xmin=', xmin,' xmax=',xmax,'ymin=', ymin,' ymax=',ymax,' zmin=',zmin,' zmax=',zmax,'. UE id: ',i,' Position: x=',self.x[i],' y=',self.y[i],' z=',self.z[i]);
        return detected
        
    def rectangle_forbidden_range(self,obsIndex):
    #Function determining the forbidden range for a rectangular obstacle as part of the Obstacle Avoidance module
    #of the Mo3 mobility model, as defined in
    #L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
    #future generation mobile wireless networks", submitted to IEEE Access

        xmin=self.ObsList[obsIndex][1]-self.ObsList[obsIndex][4]/2;
        xmax=self.ObsList[obsIndex][1]+self.ObsList[obsIndex][4]/2;
        ymin=self.ObsList[obsIndex][2]-self.ObsList[obsIndex][5]/2;
        ymax=self.ObsList[obsIndex][2]+self.ObsList[obsIndex][5]/2;
        thetamin=np.zeros(self.nNodes);
        thetamax=np.zeros(self.nNodes);
        for i in range(self.nNodes):
            if (self.x[i]<=xmin) and (self.y[i]<=ymin):#BottomLeft nodes
                thetamin[i]=math.atan2(ymin-self.y[i],xmax-self.x[i]);
                thetamax[i]=math.atan2(ymax-self.y[i],xmin-self.x[i]);
            if (self.x[i]<=xmin) and (self.y[i]>ymin) and (self.y[i]<=ymax): #Left nodes
                thetamin[i]=math.atan2(ymin-self.y[i],xmin-self.x[i]);
                thetamax[i]=math.atan2(ymax-self.y[i],xmin-self.x[i]);
            if (self.x[i]<=xmin) and (self.y[i]>ymax): #TopLeft nodes
                thetamin[i]=math.atan2(ymin-self.y[i],xmin-self.x[i]);
                thetamax[i]=math.atan2(ymax-self.y[i],xmax-self.x[i]);
            if (self.x[i]>xmin) and (self.x[i]<=xmax) and  (self.y[i]<=ymin): #Bottom nodes
                thetamin[i]=math.atan2(ymin-self.y[i],xmax-self.x[i]);
                thetamax[i]=math.atan2(ymin-self.y[i],xmin-self.x[i]);
            if (self.x[i]>xmin) and (self.x[i]<=xmax) and  (self.y[i]>ymax): #Top nodes
                thetamin[i]=math.atan2(ymax-self.y[i],xmin-self.x[i]);
                thetamax[i]=math.atan2(ymax-self.y[i],xmax-self.x[i]);
            if (self.x[i]>xmax) and  (self.y[i]<=ymin): #BottomRight nodes
                thetamin[i]=math.atan2(ymax-self.y[i],xmax-self.x[i]);
                thetamax[i]=math.atan2(ymin-self.y[i],xmin-self.x[i]);
            if (self.x[i]>xmax) and  (self.y[i]>ymin) and (self.y[i]<=ymax): #Right nodes
                thetamin[i]=math.atan2(ymin-self.y[i],xmax-self.x[i]);
                thetamax[i]=math.atan2(ymax-self.y[i],xmax-self.x[i]);
            if (self.x[i]>xmax) and  (self.y[i]>ymax): #TopRight nodes
                thetamin[i]=math.atan2(ymax-self.y[i],xmin-self.x[i]);
                thetamax[i]=math.atan2(ymin-self.y[i],xmax-self.x[i]);
        thetarange=np.zeros((2,self.nNodes))
        thetarange[0][:]=thetamin
        thetarange[1][:]=thetamax
        #breakpoint()
        return thetarange

    def ellipse_min_distance(self,obsIndex):
    #Function determining the minimum distance from a ellipsoidal obstacle as part of the Obstacle Avoidance module
    #of the Mo3 mobility model, as defined in
    #L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
    #future generation mobile wireless networks", submitted to IEEE Access

        minDistance = np.inf*np.ones(self.nNodes);
        detected=np.zeros(self.nNodes)
        xobs0=self.ObsList[obsIndex][1]
        yobs0=self.ObsList[obsIndex][2]
        a=self.ObsList[obsIndex][4]
        b=self.ObsList[obsIndex][5]
        for i in range(self.nNodes):
            x0=self.x[i]
            y0=self.y[i]
            thetaCenter=math.atan2(yobs0-y0,xobs0-x0);

            #Intersection between ellipse and ray starting in the node position and passing through to the center
            #of the ellipse.
            Delta1=np.power(np.power(b,2)*2*(x0-xobs0)*math.cos(thetaCenter)+np.power(a,2)*2*(y0-yobs0)*math.sin(thetaCenter),2)-4*(np.power(b,2)*np.power(math.cos(thetaCenter),2)+np.power(a,2)*np.power(math.sin(thetaCenter),2))*(np.power(b,2)*np.power((x0-xobs0),2)+np.power(a,2)*np.power(y0-yobs0,2)-np.power(a,2)*np.power(b,2))
            #Delta1=(b^2*2*(x0-xobs0).*cos(thetaCenter)+a^2*2*(y0-yobs0).*sin(thetaCenter)).^2-4*(b^2*(cos(thetaCenter).^2)+a^2*(sin(thetaCenter).^2)).*(b^2*(x0-xobs0).^2+a^2*(y0-yobs0).^2-a^2*b^2);
            u11=(-(np.power(b,2)*2*(x0-xobs0)*math.cos(thetaCenter)+np.power(a,2)*2*(y0-yobs0)*math.sin(thetaCenter))+np.sqrt(Delta1))/(2*(np.power(b,2)*np.power(math.cos(thetaCenter),2)+np.power(a,2)*np.power(math.sin(thetaCenter),2)))
            #u11=(-(b^2*2*(x0-xobs0).*cos(thetaCenter)+a^2*2*(y0-yobs0).*sin(thetaCenter))+sqrt(Delta1))./(2*(b^2*(cos(thetaCenter).^2)+a^2*(sin(thetaCenter).^2)));
            u12=(-(np.power(b,2)*2*(x0-xobs0)*math.cos(thetaCenter)+np.power(a,2)*2*(y0-yobs0)*math.sin(thetaCenter))-np.sqrt(Delta1))/(2*(np.power(b,2)*np.power(math.cos(thetaCenter),2)+np.power(a,2)*np.power(math.sin(thetaCenter),2)))
            #u12=(-(b^2*2*(x0-xobs0).*cos(thetaCenter)+a^2*2*(y0-yobs0).*sin(thetaCenter))-sqrt(Delta1))./(2*(b^2*(cos(thetaCenter).^2)+a^2*(sin(thetaCenter).^2)));

            #Coordinates of first intersection point
            xC11=x0+u11*math.cos(thetaCenter);
            yC11=y0+u11*math.sin(thetaCenter);
            #Coordinates of second intersection point
            xC12=x0+u12*math.cos(thetaCenter);
            yC12=y0+u12*math.sin(thetaCenter);

            dist1=np.sqrt(np.power(x0-xC11,2)+np.power(y0-yC11,2));
            dist2=np.sqrt(np.power(x0-xC12,2)+np.power(y0-yC12,2));
            if (dist1<dist2):
                minDistance=dist1
            else:
                minDistance=dist2
            detected[i] = minDistance<=self.d_OA_trigger;
        return detected

    def find_tangent_directions(self,nodeIndex,obsIndex):
    #Function determining the directions of the tangents to an ellipsoidal obstacle as part of the Obstacle Avoidance module
    #of the Mo3 mobility model, as defined in
    #L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
    #future generation mobile wireless networks", submitted to IEEE Access
        xobs0=self.ObsList[obsIndex][1]
        yobs0=self.ObsList[obsIndex][2]
        a=self.ObsList[obsIndex][4]
        b=self.ObsList[obsIndex][5]
        x0=self.x[nodeIndex]
        y0=self.y[nodeIndex]
        c0=((np.power(a,2))*(y0*yobs0)-(np.power(a,2))*(np.power(yobs0,2))+np.power(b,2)*xobs0*x0-np.power(b,2)*np.power(xobs0,2)+np.power(a,2)*np.power(b,2))/((np.power(a,2))*(y0-yobs0));
        #c0=((a^2)*(y0*yobs0)-(a^2)*(yobs0^2)+b^2*xobs0*x0-b^2*xobs0^2+a^2*b^2)/((a^2)*(y0-yobs0));
        c1=-(np.power(b,2)*(x0-xobs0))/(np.power(a,2)*(y0-yobs0));
        #c1=-(b^2*(x0-xobs0))/(a^2*(y0-yobs0));
        c2=np.power(a,2);
        #c2=a^2;
        c3=-2*np.power(a,2)*yobs0;
        #c3=-2*a^2*yobs0;
        c4=np.power(b,2);
        #c4=b^2;
        c5=-2*np.power(b,2)*(xobs0);
        #c5=-2*b^2*(xobs0);
        c6=np.power(a,2)*np.power(yobs0,2)+np.power(b,2)*np.power(xobs0,2)-np.power(a,2)*np.power(b,2);
        #c6=a^2*yobs0^2+b^2*xobs0^2-(a^2)*(b^2);
        
        aCoeff=c2*np.power(c1,2)+c4;
        #aCoeff=c2*c1^2+c4;
        bCoeff=2*c2*c1*c0+c3*c1+c5;        
        #bCoeff=2*c2*c1*c0+c3*c1+c5;
        cCoeff=c2*np.power(c0,2)+c3*c0+c6;
        #cCoeff=c2*c0^2+c3*c0+c6;
        Delta=np.power(bCoeff,2)-4*aCoeff*cCoeff;
        x_sol_1=(-bCoeff+np.sqrt(Delta))/(2*aCoeff);
        x_sol_2=(-bCoeff-np.sqrt(Delta))/(2*aCoeff);
        y_sol_1=c1*x_sol_1+c0;
        y_sol_2=c1*x_sol_2+c0;
        theta_t1=math.atan2(y_sol_1-y0,x_sol_1-x0);
        theta_t2=math.atan2(y_sol_2-y0,x_sol_2-x0);
        solVector=[theta_t1,theta_t2,x_sol_1,y_sol_1,x_sol_2,y_sol_2]
        return solVector

    def ellipse_forbidden_range(self,obsIndex):
    #Function determining the forbidden range for a ellipsoidal obstacle as part of the Obstacle Avoidance module
    #of the Mo3 mobility model, as defined in
    #L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
    #future generation mobile wireless networks", submitted to IEEE Access

        thetamin=np.zeros(self.nNodes);
        thetamax=np.zeros(self.nNodes);
        xobs0=self.ObsList[obsIndex][1]
        yobs0=self.ObsList[obsIndex][2]
        a=self.ObsList[obsIndex][4]
        b=self.ObsList[obsIndex][5]
        for i in range(self.nNodes):
            x0=self.x[i]
            y0=self.y[i]
            c0=((np.power(a,2))*(y0*yobs0)-(np.power(a,2))*(np.power(yobs0,2))+np.power(b,2)*xobs0*x0-np.power(b,2)*np.power(xobs0,2)+np.power(a,2)*np.power(b,2))/((np.power(a,2))*(y0-yobs0));
            #c0=((a^2)*(y0*yobs0)-(a^2)*(yobs0^2)+b^2*xobs0*x0-b^2*xobs0^2+a^2*b^2)/((a^2)*(y0-yobs0));
            c1=-(np.power(b,2)*(x0-xobs0))/(np.power(a,2)*(y0-yobs0));
            #c1=-(b^2*(x0-xobs0))/(a^2*(y0-yobs0));
            c2=np.power(a,2);
            #c2=a^2;
            c3=-2*np.power(a,2)*yobs0;
            #c3=-2*a^2*yobs0;
            c4=np.power(b,2);
            #c4=b^2;
            c5=-2*np.power(b,2)*(xobs0);
            #c5=-2*b^2*(xobs0);
            c6=np.power(a,2)*np.power(yobs0,2)+np.power(b,2)*np.power(xobs0,2)-np.power(a,2)*np.power(b,2);
            #c6=a^2*yobs0^2+b^2*xobs0^2-(a^2)*(b^2);
            aCoeff=c2*np.power(c1,2)+c4;
            #aCoeff=c2*c1^2+c4;
            bCoeff=2*c2*c1*c0+c3*c1+c5;        
            #bCoeff=2*c2*c1*c0+c3*c1+c5;
            cCoeff=c2*np.power(c0,2)+c3*c0+c6;
            #cCoeff=c2*c0^2+c3*c0+c6;
            Delta=np.power(bCoeff,2)-4*aCoeff*cCoeff;
            if Delta>0:#Real solutions exist
                x_sol_1=(-bCoeff+math.sqrt(Delta))/(2*aCoeff);
                x_sol_2=(-bCoeff-math.sqrt(Delta))/(2*aCoeff);
                y_sol_1=c1*x_sol_1+c0;
                y_sol_2=c1*x_sol_2+c0;
                thetamax[i]=math.atan2(y_sol_1-y0,x_sol_1-x0);
                thetamin[i]=math.atan2(y_sol_2-y0,x_sol_2-x0);
        
        thetarange=np.zeros((2,self.nNodes))
        thetarange[0][:]=thetamin
        thetarange[1][:]=thetamax
        #breakpoint()
        return thetarange
        
    def obstacle_avoidance(self):
        forbiddenRange=[];
        for obs_id in range(self.numObstacles):
            thetarange=np.zeros((2,self.nNodes));
    #For each obstacle check whether its distance from the node is
    #less than d_OA_trigger, and determine the corresponding forbidden range
            match self.ObsList[obs_id][0]:
                case 1:
                    #print('Rectangle')
                    detected=self.rectangle_min_distance(obs_id);
                    thetarange=self.rectangle_forbidden_range(obs_id);
                case 2:
                    #print('Ellipse')
                    detected=self.ellipse_min_distance(obs_id);
                    thetarange=self.ellipse_forbidden_range(obs_id);
                case _:
                    print('Unknown')
            #breakpoint()
            thetamin=thetarange.min(axis=0);
            thetamax=thetarange.max(axis=0);
            xmin=self.ObsList[obs_id][1]-self.ObsList[obs_id][4]/2;
            xmax=self.ObsList[obs_id][1]+self.ObsList[obs_id][4]/2;
            thetaproduct=np.multiply(thetarange[0][:],thetarange[1][:]);
            #breakpoint()
            splitForbiddenRangeNodes=(thetaproduct<0) & (self.x>=xmax);
            #breakpoint()
            singleForbiddenRangeNodes=(thetaproduct>=0) | (self.x<xmin);
            for k in range(self.nNodes):
                #breakpoint()
                if (obs_id==0):
                    forbiddenRange.append([])
                #If the obstacle was detected,add the corresponding forbidden range
                #to the list of forbidden ranges,otherwise ignore it
                if detected[k]:
                    if (singleForbiddenRangeNodes[k]):
                        #forbiddenRange[k][2*obs_id:]=[thetamin[k] thetamax[k]];
                        #forbiddenRange[k][2*obs_id+1]=[[0 0]];
                        forbiddenRange[k].append([thetamin[k], thetamax[k]]);
                        forbiddenRange[k].append([math.pi, math.pi]);
                    if(splitForbiddenRangeNodes[k]):
                        #forbiddenRange[k][2*obs_id]=[[-pi, thetamin[k]]];
                        #forbiddenRange[k][2*obs_id+1]=[[thetamax[k],pi]];
                        forbiddenRange[k].append([-math.pi, thetamin[k]]);
                        forbiddenRange[k].append([thetamax[k], math.pi]);
                else:
                    #forbiddenRange[k][2*(obs_id-1)+1]=[[0 0]];
                    #forbiddenRange[k][2*(obs_id-1)+2]=[[0 0]];
                    forbiddenRange[k].append([math.pi, math.pi]);
                    forbiddenRange[k].append([math.pi, math.pi]);
        # breakpoint()
        
        #Merge the forbidden ranges
        mergedForbiddenRange = []
        sortedForbiddenRange = []
        for k in range(self.nNodes):
            sortedForbiddenRangeK = sorted(forbiddenRange[k], key=lambda x: x[0]);
            #print('UE id: ',k,' Theta: ',self.theta[k])
            # for l in range(len(sortedForbiddenRangeK)-1):
            #     print('Range index: ',l,' theta_min: ',forbiddenRange[k][l][0],' thetamax: ',forbiddenRange[k][l][1]);
            mergedForbiddenRangeK = sorted(forbiddenRange[k], key=lambda x: x[0]);
            mergeComplete=False;
            lastSortedIndex=0;
            while  not mergeComplete:
                if (mergedForbiddenRangeK[lastSortedIndex][0]<= mergedForbiddenRangeK [lastSortedIndex+1][0]) and(mergedForbiddenRangeK[lastSortedIndex+1][0]<=mergedForbiddenRangeK[lastSortedIndex][1]):#ranges overlapping
                    #breakpoint();
                    merged_range=[mergedForbiddenRangeK[lastSortedIndex][0], max(mergedForbiddenRangeK[lastSortedIndex][1], mergedForbiddenRangeK[lastSortedIndex+1][1])];                    
                    oldMergedForbiddenRange=mergedForbiddenRangeK;
                    mergedForbiddenRangeK=[];
                    match lastSortedIndex:
                        case 0:
                            mergedForbiddenRangeK=[];
                        case 1:
                            mergedForbiddenRangeK.append(oldMergedForbiddenRange[0][:]);    
                        case _:
                            for i in range(0,lastSortedIndex):
                                mergedForbiddenRangeK.append(oldMergedForbiddenRange[i][:]);
                            #mergedForbiddenRangeK.append(oldMergedForbiddenRange[0:lastSortedIndex-1][:]);
                    mergedForbiddenRangeK.append(merged_range);
                    if len(oldMergedForbiddenRange)==(lastSortedIndex+1):
                        mergeComplete=True;
                    else:
                        for i in range(lastSortedIndex+2,len(oldMergedForbiddenRange)):
                            mergedForbiddenRangeK.append(oldMergedForbiddenRange[i][:]);
                    lastSortedIndex=0;
                else:
                    lastSortedIndex+=1;
                    if lastSortedIndex>=len(mergedForbiddenRangeK)-1:
                        mergeComplete=True;
                if len(mergedForbiddenRangeK)==1:
                    mergeComplete=True;
            mergedForbiddenRange.append(mergedForbiddenRangeK);
            sortedForbiddenRange.append(sortedForbiddenRangeK);
            # for l in range(len(sortedForbiddenRange)-1):
            #     if forbiddenRange[k][l]!=[0,0]:
            #         thetarange1=forbiddenRange[k][l];
            #         merged = False
            #         for m in range(l+1,len(forbiddenRange[k])):
            #             if forbiddenRange[k][m]!=[0,0]:
            #                 thetarange2=forbiddenRange[k][m]
            #                 if (thetarange2[0] <= thetarange1[0] <=thetarange2[1]) or (thetarange2[0] <= thetarange1[1] <=thetarange2[1]) or \
            #                     (thetarange1[0] <= thetarange2[0] <=thetarange1[1]) or (thetarange1[0] <= thetarange2[1] <=thetarange1[1]):
            #                     merged_range = [min(thetarange1[0], thetarange2[0]), max(thetarange1[1], thetarange2[1])]
            #                     if not any(merged_range == entry[1] for entry in mergedForbiddenRange[k]):
            #                         mergedForbiddenRange[k].append((l, merged_range))   
            #                         thetarange1 = merged_range  # to avoid that if l-m are merged but then another m' is merged to l, i have that l-m and l-m' are overlapped
            #                         forbiddenRange[k][m] = merged_range # inserted on the place of m, so that is re-checked later
            #                     merged = True
            #         if not merged and (not any(thetarange1 == entry[1] for entry in mergedForbiddenRange[k])):
            #                 mergedForbiddenRange[k].append((l, forbiddenRange[k][l])) 
        #Check if the current direction will lead to a collision, and if this is
        #the case change it.
        prev_theta = np.zeros(self.nNodes)
        for k in range(self.nNodes):
            direction_change = False
            prev_theta[k] = self.theta[k]
            #for l, merged_range in mergedForbiddenRange[k]:
            for l in range(len(mergedForbiddenRange[k])):
                #print('UE id: ',k,' Theta: ',self.theta[k],' theta_min: ',mergedForbiddenRange[k][l][0],' thetamax: ',mergedForbiddenRange[k][l][1]);
                if self.theta[k] >= mergedForbiddenRange[k][l][0] and self.theta[k] <= mergedForbiddenRange[k][l][1]:
                    if mergedForbiddenRange[k][l][1] < math.pi:
                        thetaUp = math.fabs(mergedForbiddenRange[k][l][1] - self.theta[k])
                        # thetaUpCandidate = self.theta[k] + thetaUp
                    else:
                        thetaUp = np.inf
                        # thetaUpCandidate = None
                    if mergedForbiddenRange[k][l][0] > -math.pi:
                        thetaDown = math.fabs(self.theta[k] - mergedForbiddenRange[k][l][0])
                        # thetaDownCandidate = self.theta[k] - thetaDown
                    else:
                        thetaDown = np.inf
                        # thetaDownCandidate = None
                    
                    # Min distance from next forbidden range
                    min_dist_up = np.inf
                    min_dist_down = np.inf
                    for other_range in mergedForbiddenRange[k]:
                        # From upper border to beginning of next range
                        if other_range[0] > mergedForbiddenRange[k][l][1]:
                            dist_up = other_range[0] - mergedForbiddenRange[k][l][1]
                            min_dist_up = min(min_dist_up, dist_up)
                        # From lower border to beginning of next range
                        if other_range[1] < mergedForbiddenRange[k][l][0]:
                            dist_down = mergedForbiddenRange[k][l][0] - other_range[1]
                            min_dist_down = min(min_dist_down, dist_down)

                    # Margins (half of min distance or theta_OA)
                    dynamic_theta_OA_up = min_dist_up / 2 if min_dist_up != np.inf else self.theta_OA
                    dynamic_theta_OA_down = min_dist_down / 2 if min_dist_down != np.inf else self.theta_OA
                    # dynamic_theta_OA_up= self.theta_OA;
                    # dynamic_theta_OA_down= self.theta_OA;

                    if thetaUp < thetaDown:
                        self.theta[k] = mergedForbiddenRange[k][l][1] + dynamic_theta_OA_up
                        if self.theta[k] > math.pi:
                            self.theta[k] = self.theta[k] - 2 * math.pi
                        direction_change = True
                    else:
                        self.theta[k] = mergedForbiddenRange[k][l][0] - dynamic_theta_OA_down
                        if self.theta[k] < -math.pi:
                            self.theta[k] = self.theta[k] + 2 * math.pi
                        direction_change = True
            if direction_change:
                # # Uncomment the following lines to visualize the obstacle avoidance
                #print(f'Merged Range {l}: thetaUp=', thetaUp, ' thetaDown=', thetaDown)
                #print(f'Node {k} changed direction from {prev_theta[k]} to {self.theta[k]}')
                #plot_obstacle_avoidance(k, self.x[k], self.y[k], prev_theta[k], self.theta[k], forbiddenRange[k], mergedForbiddenRange[k])
                pass
        
    def update_mobility_vectors(self, t: float, nodeList: List[Node]):
        
        self.updateCounter=self.updateCounter+1
        
	    #Step 0: extract positions and calculate distances once and for all to save time later.
        self.dMatrix=np.inf*np.ones((self.nNodes,self.nNodes))
        for w in range(0,self.nNodes-1):
            for r in range(w+1,self.nNodes):
                self.dMatrix[w,r]=np.sqrt(np.power(self.x[w]-self.x[r],2)+np.power(self.y[w]-self.y[r],2)+np.power(self.z[w]-self.z[r],2))
                self.dMatrix[r,w]=self.dMatrix[w,r]
        v0=copy.deepcopy(self.v)
        #breakpoint()
        theta0=copy.deepcopy(self.theta)
        phi0=copy.deepcopy(self.phi)
    	
    	#Step 1 - Individual Mobility
        if abs(math.remainder(t,self.T_IM))<1e-10:
            #print('Individual mobility, t=',t)
            #print(self.dMatrix)
            self.individual_mobility()
            
    	#Step 2 - Correlated Mobility
        if self.correlatedMobilityFlag==True:
            if abs(math.remainder(t,self.T_CM))<1e-10:
                #print('Correlated mobility, t=',t)
                if self.nextSwitchTime>=0 and self.nextSwitchTime<=t:
                	self.load_binding_matrix()
                #breakpoint()
                #print(self.dMatrix)
                self.correlated_mobility()
            #else:
                #print('Remainder: ',math.remainder(t,self.T_CM))

        #for i in range(self.nNodes):
            #print('Node: ',i)
            #print('Speed: ',self.v[i])
            #print('theta: ',self.theta[i])
            #print('phi: ',self.phi[i])
            #print('Position: ',self.x[i],' ',self.y[i],' ',self.z[i])

    	#Step 3 - Collision Avoidance
        if self.collisionAvoidanceFlag==True:
             if abs(math.remainder(t,self.T_CA))<1e-10:
                #print('Collision avoidance, t=',t)
                self.collision_avoidance()
                #self.collision_avoidance_2D()
                while self.sameLineCollisionFlag==1:
            	    self.collision_avoidance()
        #breakpoint()
        
    	#Step 4 - Obstacle Avoidance
        if self.obstacleAvoidanceFlag==True:
            if abs(math.remainder(t,self.T_OA))<1e-10:
                #print('Obstacle avoidance, t=',t)
                self.obstacle_avoidance()
                #breakpoint()
                
    	#Step 5 - Upper Bounds Enforcement
        if self.upperBoundsEnforcementFlag==True:
            #print('Upper bounds enforcement, t=',t)
            self.upper_bounds_enforcement(v0,theta0,phi0)
        	
        #for i in range(self.nNodes):
            #print('Node: ',i)
            #print('Speed: ',self.v[i])
            #print('theta: ',self.theta[i])
            #print('phi: ',self.phi[i])
            #print('CM_status: ',self.CM_forced_mode[i])
            
		#Determine the displacement according to the updated speed vectors
        for i in range(self.nNodes):
            #print(i)
            dX=self.v[i]*np.cos(self.theta[i])*np.cos(self.phi[i])*self.dt;
            dY=self.v[i]*np.sin(self.theta[i])*np.cos(self.phi[i])*self.dt;
            dZ=self.v[i]*np.sin(self.phi[i])*self.dt;
            #print('Displacement of node ',i,': ',dX,dY,dZ)
            #print('cos(theta):', np.cos(self.theta[i]))
            #print('sin(theta):', np.sin(self.theta[i]))
            #print('cos(phi):', np.cos(self.phi[i]))
            #print('sin(phi):', np.sin(self.phi[i]))
    		#self.x[i]=self.x[i]+dX;
    		#self.y[i]=self.y[i]+dY;
    		#self.z[i]=self.z[i]+dZ;  	
    	    #Apply the perfect reflection law to compensate for movement area
    	    #boundary violations
            xViolation=False
            yViolation=False
            zViolation=False
            if self.x[i]+dX < self.x_min:
                self.x[i]=self.x_min-dX-self.x[i]
                self.theta[i]= math.pi-self.theta[i];
                xViolation=True
                #print('x_min Violation')
            if self.x[i]+dX > self.x_max:
                self.x[i]=self.x_max-dX+(self.x_max-self.x[i])
                self.theta[i]= math.pi-self.theta[i];
                xViolation=True
                #print('x_max Violation')
            if self.y[i]+dY < self.y_min:
                self.y[i]=self.y_min-dY-self.y[i]
                self.theta[i]= -self.theta[i];
                yViolation=True
                #print('y_min Violation')
            if self.y[i]+dY > self.y_max:
                #print(self.y[i],dY,self.y_max)
                self.y[i]=self.y_max-dY+(self.y_max-self.y[i])
                self.theta[i]= -self.theta[i];
                yViolation=True
                #print('y_max Violation')
            if self.z[i]+dZ < self.z_min:
                self.z[i]=self.z_min-dZ-self.z[i]
                self.phi[i]= -self.phi[i];
                zViolation=True
                #print('z_min Violation')
            if self.z[i]+dZ > self.z_max:
                self.z[i]=self.z_max-dZ+(self.z_max-self.z[i])
                self.phi[i]= -self.phi[i];
                zViolation=True
                #print('z_max Violation')
            while self.theta[i]<-math.pi:
                self.theta[i]=self.theta[i]+2*math.pi
            while self.theta[i]>math.pi:
                self.theta[i]=self.theta[i]-2*math.pi
            if xViolation==False:
                self.x[i]=self.x[i]+dX
            if yViolation==False:
                self.y[i]=self.y[i]+dY
            if zViolation==False:
                self.z[i]=self.z[i]+dZ
            node=nodeList[i]
            node.set_coordinates(self.x[i],self.y[i],self.z[i])

        # salvataggio     
        if self.pathSave==True:
            self.xPath=np.concatenate((self.xPath,self.x.reshape((1,self.nNodes))),axis=0)
            self.yPath=np.concatenate((self.yPath,self.y.reshape((1,self.nNodes))),axis=0)
            self.zPath=np.concatenate((self.zPath,self.z.reshape((1,self.nNodes))),axis=0)
        #if not self.CM_forced_mode.any():
        #	self.endSim=1;