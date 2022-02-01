
#include "h/MoMo.h"


extern int displayScaleFactor;

Define_Module(MoMo);

void MoMo::loadGroupMembers()
{
    //char singleLine[NMAX*10];
    string singleLineStr;
    int IDList[NMAX];
    int gModalityAcquired=0;
    bool lineFound=false;
    bool acquisitionComplete=false;
    ifstream ifs(groupsFileName);
    istringstream ostr1;

    //while( ifs.getline(singleLine,NMAX*10) && !lineFound)
    while( getline(ifs, singleLineStr) && !acquisitionComplete)
    {
        //if (strchr(singleLineStr,'#')==NULL)
        switch(singleLineStr[0])
        {
        case '#': //Comment
            EV << "Node["<<myID<<"]: comment line, skipping"<< endl;
            break;
        case 'N': //Provides number of group modalities
            int temp2;
            singleLineStr[0]=' ';
            ostr1.clear();
            ostr1.str(singleLineStr);
            ostr1 >> temp2;
            groupModalities=temp2;
            EV << "Node["<<myID<<"]: group modalities: "<< groupModalities << endl;
            break;
        case '-': //Provides proximity bounds for a node
            if(!lineFound)
                EV << "Node["<<myID<<"]: proximity bounds for another node, skipping it"<< endl;
            else
            {
                double temp;
                int i=0;
                ostr1.clear();
                ostr1.str(singleLineStr);
                while (ostr1 >> temp)
                {
                    //EV <<temp<<' '<<endl;

                    localMoMoProximityBound[IDList[i]][gModalityAcquired]=temp;
                    i++;
                }
                gModalityAcquired++;
                if (gModalityAcquired==groupModalities)
                    acquisitionComplete=true;
            }
            break;
        default: //Provides IDs of nodes that are considered group members by a node

            //if(singleLineStr[0]!='#')
            //{
            //EV <<"Node["<<myID<<"]: not comment line found"<< endl;
            int temp, i =0;
            //EV <<singleLineStr<<endl;
            ostr1.clear();
            ostr1.str(singleLineStr);
            while (ostr1 >> temp)
            {
                //EV <<temp<<' '<<endl;
                IDList[i]=temp;
                i++;
            }
            if(IDList[0]==myID)
            {
                lineFound=true;
                groupSize=i;
                gModalityAcquired=0;
            }
            //}
            //else
            //;//EV <<"Node["<<myID<<"]: comment line skipped"<< endl;
        }
    }
    if(lineFound)
    {
        EV <<"Node["<<myID<<"]: my group members are:"<< endl;
        for (int j=0;j<groupSize;j++)
        {
            EV <<IDList[j]<<' ';
            localGroupVector[IDList[j]]=true;
        }
        EV <<endl;
        if(acquisitionComplete)
        {
            EV <<"Node["<<myID<<"]: my bounds to group members are:"<< endl;
            for (int i=0;i<groupModalities;i++)
            {
                for (int j=0;j<groupSize;j++)
                {
                    EV <<localMoMoProximityBound[IDList[j]][i]<<' ';
                }
                EV <<endl;
            }
        }
    }
    else
        EV <<"Node["<<myID<<"]: line not found in group settings file"<< endl;

}
bool MoMo::rebound(double& x, double &y)
{
    bool update=false;

    d("rebound");
    if( x < minX)
    {
        dX *= (-1); // change the sign
        x = minX;
        directionVector[myID] = 3.14-directionVector[myID];
        update=true;
    }
    if( x > maxX)
    {
        x = maxX;
        dX *= (-1);
        update=true;
        directionVector[myID] = 3.14 -directionVector[myID];
    }
    if( y < minY)
    {
        dY *= -1;
        y = minY;

        directionVector[myID] = 6.28 - directionVector[myID] ;


        update=true;
    }
    if( y > maxY)
    {
        dY *= -1;
        y = maxY;
        directionVector[myID] = 6.28 - directionVector[myID];
        update=true;
    }
    if(update)
        EV <<"Rebound"<<endl;
    return update;
}

double MoMo::distance(int i, int j)
{
    double dist;
    dist=sqrt(pow(database[i].x-database[j].x,2)+pow(database[i].y-database[j].y,2));
    return dist;
}
bool MoMo::torus(double &x, double &y)
{
    bool update=false;
    d("toru");
    if( x < minX)
    {
        x = 2 * minX - x;
        update=true;
    }
    if( x > maxX)
    {
        x = 2 * maxX - x;
        update=true;
    }
    if( y < minY)
    {
        y = 2 * minY - y;
        update=true;
    }
    if( y > maxY)
    {
        y = 2 * maxY - y;
        update=true;
    }
    return update;
}
//Old version kept to be on the safe side
/*int MoMo::checkGroupingFactor(double x, double y)
{
	EV <<"Node["<<myID<<"]: checking GroupFactor"<<endl;

	int fellowshipSize=0;
	double minGroupMateDistance=2*maxX;
	int closestGroupMate=-1;
	for(int i=0;i<hostNum;i++)
	{
		if((i!=myID)&&(groupIDVector[i]==groupIDVector[myID]))
			if(distance(myID,i)<=momoProximityBound)
				fellowshipSize++;
			else
				if (distance(myID,i)<=minGroupMateDistance)
				{
					minGroupMateDistance=distance(myID,i);
					closestGroupMate=i;
				}
	}
	double groupFactor=(double) fellowshipSize/(groupSize-1);
	if(groupFactor>=rho)
	{
		forcedStatus=false;
		EV <<"Node["<<myID<<"]: free movement, with gf = "<<groupFactor<<", rho = "<<rho<<" and fellowshipSize = "<< fellowshipSize<< endl;

		return -1;
	}
	else
	{
		forcedStatus=true;
		EV <<"Node["<<myID<<"]: forced movement, with gf = "<<groupFactor<<", rho = "<<rho<<" and fellowshipSize = "<< fellowshipSize<< endl;
		return closestGroupMate;
	}
}
 */
int MoMo::checkGroupingFactor(double x, double y)
{
    EV <<"Node["<<myID<<"]: checking GroupFactor"<<endl;

    int fellowshipSize=0;
    double x_temp=0, y_temp=0;
    double minGroupMateDistance=2*maxX;
    int closestGroupMate=-1;
    double groupFactor;
    for(int i=0;i<hostNum;i++)
    {
        if((i!=myID)&&(localGroupVector[i]))
            if(distance(myID,i)<=localMoMoProximityBound[i][groupModality])
            {
                EV <<"Node["<<myID<<"]: distance from "<<i<<" = "<<distance(myID,i)<<", shorter than bound = "<< localMoMoProximityBound[i][groupModality]<< endl;
                fellowshipSize++;
            }
            else
            {
                EV <<"Node["<<myID<<"]: distance from "<<i<<" = "<<distance(myID,i)<<", longer than bound = "<< localMoMoProximityBound[i][groupModality]<< endl;
                x_temp+=database[i].x;
                y_temp+=database[i].y;
                if (distance(myID,i)<=minGroupMateDistance)
                {
                    minGroupMateDistance=distance(myID,i);
                    closestGroupMate=i;
                }

            }
    }
    if(groupSize>1)
        groupFactor=(double) fellowshipSize/(groupSize-1);
    else
        groupFactor=1;
    if(groupFactor>=rho)
    {
        forcedStatus=false;
        EV <<"Node["<<myID<<"]: free movement, with gf = "<<groupFactor<<", rho = "<<rho<<" and fellowshipSize = "<< fellowshipSize<< endl;

        return -1;
    }
    else
    {
        forcedStatus=true;
        EV <<"Node["<<myID<<"]: forced movement, with gf = "<<groupFactor<<", rho = "<<rho<<" and fellowshipSize = "<< fellowshipSize<< endl;
        return closestGroupMate;
    }
}
void MoMo::checkCollisions(double & deltaTheta, double & deltaV)
{
    //Step 1:determine the set of nodes that may potentially collide. This is done to limit computational cost, but in theory all nodes could be taken into account.
    int collidingNodes[hostNum];
    double xcoll[hostNum];
    double ycoll[hostNum];
    int numCollidingNodes=0;
    double x1,x2,y1,y2,theta1,theta2,thetalimit,distancefromCollisionPoint, distancefromCollisionPointJ,timeToCollisionPoint,timeToCollisionPointJ,requiredSpeed,speedCorrectionJ=0,directionCorrectionJ=0;
    bool visible,onCollisionCourse;
    double xC, yC;
    fstream caDebug;
    if(caDebugFlag)
        caDebug.open("collisionDebug.txt", ios::out|ios::app);
    deltaV=0;
    deltaTheta=0;
    EV <<"Node["<<myID<<"]: position: ["<<database[myID].x<<","<<database[myID].y<<"], direction "<<directionVector[myID]<<", speed "<<speedVector[myID] <<endl;
    for(int i=0;i<hostNum;i++)
    {
        if(i!=myID)
        {
            EV <<"Node["<<myID<<"]: checking node "<< i << " - position: ["<<database[i].x<<","<<database[i].y<<"] - direction "<< directionVector[i]<<" - speed "<< speedVector[i]<<endl;
            //First check: is the node visible?
            if(distance(myID,i)>checkCollisionDistance)
            {
                visible=false;
                EV <<"Node["<<myID<<"]: distance from "<<i<<" = "<<distance(myID,i)<<", longer than checkCollisionDistance = "<< checkCollisionDistance<< endl;

            }

            else
            {
                visible=true;
                EV <<"Node["<<myID<<"]: distance from "<<i<<" = "<<distance(myID,i)<<", shorter than checkCollisionDistance = "<< checkCollisionDistance<< endl;
                //Second check: is the node on a collision course?

                if (database[myID].x<database[i].x)
                {
                    x1=database[myID].x;
                    x2=database[i].x;
                    y1=database[myID].y;
                    y2=database[i].y;
                    theta1=directionVector[myID];
                    theta2=directionVector[i];
                }
                else
                {
                    x1=database[i].x;
                    x2=database[myID].x;
                    y1=database[i].y;
                    y2=database[myID].y;
                    theta1=directionVector[i];
                    theta2=directionVector[myID];
                }

                double dx=x2-x1;
                double dy=y2-y1;
                double det=cos(theta2)*sin(theta1)-sin(theta2)*cos(theta1);
                if(det==0)
                {
                    EV <<"Node["<<myID<<"]: "<<"Parallel rays, not colliding!"<<endl;
                    onCollisionCourse=false;
                }
                else
                {
                    double u=(dy*cos(theta2)-dx*sin(theta2))/det;
                    double v=(dy*cos(theta1)-dx*sin(theta1))/det;

                    if((u>0)&&(v>0))
                    {
                        EV <<"Node["<<myID<<"]: "<<"u = " <<u<<", v = "<<v<<", collision!"<<endl;
                        xC=x1+u*cos(theta1);
                        yC=y1+u*sin(theta1);
                        EV <<"Node["<<myID<<"]: collision at xC = " <<xC<<", yC = "<<yC<<endl;
                        onCollisionCourse=true;
                        collidingNodes[numCollidingNodes]=i;
                        xcoll[numCollidingNodes]=xC;
                        ycoll[numCollidingNodes]=yC;
                        numCollidingNodes++;
                    }
                    else
                    {
                        EV <<"Node["<<myID<<"]: "<<"u = " <<u<<", v = "<<v<<", not colliding!"<<endl;
                        onCollisionCourse=false;
                    }
                }
                if(caDebugFlag)
                {
                    caDebug<<"T="<<simTime().dbl()<<" "<<x1<<" "<<y1<<" "<<theta1<<" "<<x2<<" "<<" "<<y2<<" "<<theta2;
                    if(onCollisionCourse)
                        caDebug<<" 1"<<endl;
                    else
                        caDebug<<" 0"<<endl;
                }
                /*
                thetalimit=atan2((y1-y2),(x1-x2));
                EV <<"Node["<<myID<<"]: theta1 "<< theta1 << " theta2 "<< theta2<<" thetalimit "<< thetalimit<<endl;
                if(caDebugFlag)
                    caDebug<<"T="<<simTime().dbl()<<" ";
                if ((theta1>=0)&&(theta1<3.1415/2))
                {
                    EV <<"0<=theta1<pi/2, ";

                    if(y2<=y1)
                    {
                        if(caDebugFlag)
                            caDebug<<"Case_1A, ";
                        EV <<"y2="<<y2<<" <= y1="<<y1<<", ";
                        if((theta2>theta1)&&(theta2<thetalimit))
                        {
                            EV <<"theta2>theta1 and theta2<thetalimit -> on collision course!"<<endl;
                            onCollisionCourse=true;
                        }
                        else
                        {
                            EV <<"theta2<theta1 or theta2>thetalimit -> not on collision course!"<<endl;
                            onCollisionCourse=false;
                        }
                    }
                    else
                    {
                        if(caDebugFlag)
                            caDebug<<"Case_1B, ";
                        EV <<"y2="<<y2<<" > y1="<<y1<<", ";
                        if((theta2<theta1)&&(theta2<thetalimit))
                        {
                            EV <<"theta2<theta1 and theta2>thetalimit -> on collision course!"<<endl;
                            onCollisionCourse=true;
                        }
                        else
                        {
                            EV <<"theta2>theta1 or theta2<thetalimit -> not on collision course!"<<endl;
                            onCollisionCourse=false;
                        }
                    }
                }
                if ((theta1>=3.1415/2)&&(theta1<3.1416))//Here we want to be sure that we get any approximation of pi in the interval...
                {
                    EV <<"pi/2<=theta1<pi, ";
                    //caDebug<<"pi/2<=theta1<pi, ";
                    if(y2<y1)
                    {
                        EV <<"y2="<<y2<<" < y1="<<y1<<", ";
                        if(caDebugFlag)
                            caDebug<<"Case_2A, ";
                        if((theta2<theta1)&&(theta2>thetalimit))
                        {
                            EV <<"theta2<theta1 and theta2>thetalimit -> on collision course!"<<endl;
                            onCollisionCourse=true;
                        }
                        else
                        {
                            EV <<"theta2>theta1 or theta2<thetalimit -> not on collision course!"<<endl;
                            onCollisionCourse=false;
                        }
                    }
                    else
                    {
                        EV <<"y2="<<y2<<" >= y1="<<y1<<", ";
                        if(caDebugFlag)
                            caDebug<<"Case_2B, ";
                        if((theta2>theta1)||(theta2<thetalimit))
                        {
                            EV <<"theta2>theta1 or theta2<thetalimit -> on collision course!"<<endl;
                            onCollisionCourse=true;

                        }
                        else
                        {
                            EV <<"theta2<theta1 and theta2>thetalimit -> not on collision course!"<<endl;
                            onCollisionCourse=false;
                        }
                    }
                }
                if ((theta1>=-3.1416)&&(theta1<-3.1415/2))//Here we want to be sure that we get any approximation of -pi in the interval...
                {
                    EV <<"-pi<=theta1<-pi/2, ";
                    //caDebug<<"-pi<=theta1<-pi/2, ";
                    if(y2<=y1)
                    {
                        EV <<"y2="<<y2<<" <= y1="<<y1<<", ";
                        if(caDebugFlag)
                            caDebug<<"Case_3A, ";
                        if((theta2<theta1)||(theta2>thetalimit))
                        {
                            EV <<"theta2<theta1 or theta2>thetalimit -> on collision course!"<<endl;
                            onCollisionCourse=true;
                        }
                        else
                        {
                            EV <<"theta2>theta1 and theta2<thetalimit -> not on collision course!"<<endl;
                            onCollisionCourse=false;
                        }
                    }
                    else
                    {
                        EV <<"y2="<<y2<<" > y1="<<y1<<", ";
                        if(caDebugFlag)
                            caDebug<<"Case_3B, ";
                        if((theta2>theta1)&&(theta2<thetalimit))
                        {
                            EV <<"theta2>theta1 and theta2<thetalimit -> on collision course!"<<endl;
                            onCollisionCourse=true;
                        }
                        else
                        {
                            EV <<"theta2<theta1 or theta2>thetalimit -> not on collision course!"<<endl;
                            onCollisionCourse=false;
                        }
                    }
                }
                if ((theta1>=-3.1415/2)&(theta1<0))
                {
                    EV <<"-pi/2<=theta1<0, ";
                    //caDebug<<"-pi/2<=theta1<0, ";

                    if(y2<=y1)
                    {
                        if(caDebugFlag)
                            caDebug<<"Case_4A, ";
                        EV <<"y2="<<y2<<" <= y1="<<y1<<", ";
                        if((theta2<theta1)&&(theta2>thetalimit))
                        {
                            EV <<"theta2<theta1 and theta2>thetalimit -> on collision course!"<<endl;
                            onCollisionCourse=true;
                        }
                        else
                        {
                            EV <<"theta2>theta1 or theta2<thetalimit -> not on collision course!"<<endl;
                            onCollisionCourse=false;
                        }
                    }
                    else
                    {
                        if(caDebugFlag)
                            caDebug<<"Case_4B, ";
                        EV <<"y2="<<y2<<" > y1="<<y1<<", ";
                        if((theta2<theta1)&&(theta2>thetalimit))
                        {
                            EV <<"theta2<theta1 and theta2>thetalimit -> on collision course!"<<endl;
                            onCollisionCourse=true;
                        }
                        else
                        {
                            EV <<"theta2>theta1 or theta2<thetalimit -> not on collision course!"<<endl;
                            onCollisionCourse=false;
                        }
                    }
                }
                if(caDebugFlag)
                {
                    caDebug<<x1<<" "<<y1<<" "<<theta1<<" "<<x2<<" "<<" "<<y2<<" "<<theta2<<" "<<thetalimit;
                    if(onCollisionCourse)
                        caDebug<<" 1"<<endl;
                    else
                        caDebug<<" 0"<<endl;
                }*/
            }
        }
    }
    if(caDebugFlag)
        caDebug.close();
    double requiredMinSpeedVector[numCollidingNodes];
    double requiredMaxSpeedVector[numCollidingNodes];
    for (int j=0;j<numCollidingNodes;j++)
        //Step 2: for each (potentially) colliding node j determine the collision point and calculate corrections to speed and direction, if needed.
    {
        //xcoll[j]=(tan(directionVector[myID])*database[myID].x-tan(directionVector[collidingNodes[j]])*database[collidingNodes[j]].x+database[collidingNodes[j]].y-database[myID].y)/(tan(directionVector[myID])-tan(directionVector[collidingNodes[j]]));
        //ycoll[j]=tan(directionVector[myID])*xcoll[j]-tan(directionVector[myID])*database[myID].x+database[myID].y;
        distancefromCollisionPoint=sqrt(pow(database[myID].x-xcoll[j],2)+pow(database[myID].y-ycoll[j],2));
        distancefromCollisionPointJ=sqrt(pow(database[collidingNodes[j]].x-xcoll[j],2)+pow(database[collidingNodes[j]].y-ycoll[j],2));
        EV <<"Node["<<myID<<"]: processing colliding node "<< collidingNodes[j] << ", collision point: ["<< xcoll[j] << "," << ycoll[j]<<"]";
        if(distancefromCollisionPoint<checkCollisionDistance)
        {
            timeToCollisionPoint=distancefromCollisionPoint/speedVector[myID];
            timeToCollisionPointJ=distancefromCollisionPointJ/speedVector[collidingNodes[j]];

            EV <<", TTCP: "<<timeToCollisionPoint<<", TTCPJ:"<<timeToCollisionPointJ<<", MRT: "<< minReactionTime;

            //Approach 1: we add a correction to both direction and speed inversely proportional to the probability of collision
            /*   speedCorrectionJ=-(minReactionTime/(fabs(timeToCollisionPoint-timeToCollisionPointJ)))*((double) maxSpeed->doubleValue())*sgn(timeToCollisionPoint-timeToCollisionPointJ);
            directionCorrectionJ=(minReactionTime/fabs(timeToCollisionPoint-timeToCollisionPointJ))*(gmax*((double) mobilitySelectPeriod->doubleValue()))*sgn(directionVector[collidingNodes[j]]-directionVector[myID]);
             */

            //Approach 2: we only act on speed, without modifying the direction; we make sure that when the other node is at the potential point of collision (intersection point), the node is at at least minDistanceCA meters.
            if(timeToCollisionPoint>=timeToCollisionPointJ)//The node is already going to cross the intersection point after node j, so we further decrease its speed
            {
                requiredSpeed=speedVector[collidingNodes[j]]*(distancefromCollisionPoint)/(minDistanceCA+distancefromCollisionPointJ);
                EV <<", required speed (max): "<< requiredSpeed<<", current speed: "<<speedVector[myID]<<endl;
                requiredMaxSpeedVector[j]=requiredSpeed;
                requiredMinSpeedVector[j]=0;
                /*
                if(speedVector[myID]>=requiredSpeed)
                    speedCorrectionJ=requiredSpeed-speedVector[myID];
                else
                    speedCorrectionJ=0;
                 */

            }
            else//The node is already going to cross the intersection point before node j, so we further increase its speed
            {
                requiredSpeed=speedVector[collidingNodes[j]]*(minDistanceCA+distancefromCollisionPoint)/(distancefromCollisionPointJ);
                EV <<", required speed (min): "<< requiredSpeed<<", current speed: "<<speedVector[myID]<<endl;
                requiredMinSpeedVector[j]=requiredSpeed;
                requiredMaxSpeedVector[j]=(double) maxSpeed->doubleValue();
                /*if(speedVector[myID]<requiredSpeed)
                    speedCorrectionJ=requiredSpeed-speedVector[myID];
                else
                    speedCorrectionJ=0;
                 */
            }
            /*
            EV <<", speed correction: "<< speedCorrectionJ<<"; direction correction: "<< directionCorrectionJ<<endl;
            deltaV=deltaV+speedCorrectionJ;
             */
            deltaTheta=deltaTheta+directionCorrectionJ;


        }
        else
        {
            EV <<", further than checkCollisionDistance "<<checkCollisionDistance<<", ignoring it"<<endl;
            requiredMaxSpeedVector[j]=(double) maxSpeed->doubleValue();
            requiredMinSpeedVector[j]=0;
        }
    }
    if(numCollidingNodes>0)
    {
        double vDecrease,vIncrease,vMaxDecrease,vMaxIncrease;
        vMaxIncrease=0;
        vMaxDecrease=0;
        EV <<"Node["<<myID<<"]: vector of maxSpeed values:"<<endl;
        for (int j=0;j<numCollidingNodes;j++)
        {
            EV<<"v<"<<requiredMaxSpeedVector[j]<< "m/s"<<endl;
            if(requiredMaxSpeedVector[j]<speedVector[myID])
            {
                vDecrease=requiredMaxSpeedVector[j]-speedVector[myID];
                if (vDecrease>vMaxDecrease)
                    vMaxDecrease=vDecrease;
            }
        }
        EV <<"Node["<<myID<<"]: vector of minSpeed values:"<<endl;
        for (int j=0;j<numCollidingNodes;j++)
        {
            EV<<"v>"<<requiredMinSpeedVector[j]<< "m/s"<<endl;
            if(requiredMinSpeedVector[j]>speedVector[myID])
            {
                vIncrease=requiredMinSpeedVector[j]-speedVector[myID];
                if (vIncrease>vMaxIncrease)
                    vMaxIncrease=vIncrease;
            }
        }
        if(vMaxIncrease>0)//We are required to increase node speed
        {
            if(vMaxDecrease>0)//But we are also required to decrease it! There is a conflict, we will pick the choice that minimizes variation to node speed.
            {
                emit(collisionConflictSignal, 1);
                if(vMaxDecrease>vMaxIncrease)
                    deltaV=vMaxIncrease;
                else
                    deltaV=-vMaxDecrease;
            }
            else//No speed decrease required, we can increase it as needed
                deltaV=vMaxIncrease;
        }
        else//No speed increase required
        {
            if(vMaxDecrease>0)//we can decrease it if needed
                deltaV=-vMaxDecrease;

        }

    }
    if(deltaV!=0)
    {
        emit(collisionAvoidedSignal, 1);
        EV <<"Node["<<myID<<"]: speed changed of "<< deltaV<< " to avoid collision!"<<endl;
    }
    EV <<"Node["<<myID<<"]: total corrections before check: speedCorrection="<< deltaV << ", directionCorrection="<< deltaTheta<<endl;
    if(deltaV>=0)
    {

        deltaV=fabs(deltaV)>((double) mobilitySelectPeriod->doubleValue() )*amax?((double) mobilitySelectPeriod->doubleValue() )*amax:deltaV;
    }
    else
        deltaV=fabs(deltaV)>((double) mobilitySelectPeriod->doubleValue() )*amax?-((double) mobilitySelectPeriod->doubleValue() )*amax:deltaV;
    if(fabs(deltaTheta)>=gmax*((double) moveInterval->doubleValue()))
    {
        if(deltaTheta>0)
            deltaTheta=(gmax*((double) moveInterval->doubleValue())-1e-8);
        else
            deltaTheta=-(gmax*((double) moveInterval->doubleValue())-1e-8);
    }
    EV <<"Node["<<myID<<"]: total corrections after check: speedCorrection="<< deltaV << ", directionCorrection="<< deltaTheta<<endl;
    //Extension to approach 1, selecting only the modification to speed or direction: the one that introduces the smaller variation to original speedvector is selected
    /*    double speedPercVar=fabs(deltaV)/((double) maxSpeed->doubleValue());
    double directionPercVar=fabs(deltaTheta)/3.14159;
    if (speedPercVar<=directionPercVar)
    {
       deltaTheta=0;
    }
    else
    {
        deltaV=0;
    }*/
}
double MoMo::momoMobility(double& x, double& y)
{
    /*int fellowshipSize=0;
  double minGroupMateDistance=2*maxX;
  int closestGroupMate=-1;
  for(int i=0;i<hostNum;i++)
    {
      if((i!=myID)&&(groupIDVector[i]==groupIDVector[myID]))
	if(distance(myID,i)<=momoProximityBound)
	  fellowshipSize++;
	else
	  if (distance(myID,i)<=minGroupMateDistance)
	    {
	      minGroupMateDistance=distance(myID,i);
	      closestGroupMate=i;
	    }
    }
    double groupFactor=(double) fellowshipSize/(groupSize-1);*/
    double deltaTheta;
    double deltaV;
    double rotation;
    double speedVariation;
    int closestGroupMate=-1;
    double originalDirection=directionVector[myID];
    double originalSpeed=speedVector[myID];
    if(forcedStatus||checkUpdateFlag||(!groupBoundUpdate))
    {
        closestGroupMate=checkGroupingFactor(x, y);
        checkUpdateFlag=false;
    }
    if(!forcedStatus)//If group behavior is off nodes are always in Free mode
    {
        //EV <<"Node["<<myID<<"]: free movement, with gf = "<<groupFactor<<", rho = "<<rho<<" and fellowshipSize = "<< fellowshipSize<< endl;
        if(selectNewMobilityParameters[myID])
        {
            //choose the new speed

            double maxSpeedValue=(((double) mobilitySelectPeriod->doubleValue() )*amax <= (double) maxSpeed->doubleValue())? ((double) mobilitySelectPeriod->doubleValue() )*amax: (double) maxSpeed->doubleValue() ;
            double minSpeedValue=(double) minSpeed->doubleValue();
            //double delta_v=uniform(minSpeedValue, maxSpeedValue,1);
            //double newSpeed=speedVector[myID]+delta_v>=0?speedVector[myID]+delta_v:0;
            double newSpeed=uniform(minSpeedValue, maxSpeedValue,1);
            speedVector[myID]=(newSpeed<=(double) maxSpeed->doubleValue())?newSpeed:(double) maxSpeed->doubleValue();

            //choose the direction angle
            //double delta_g=uniform((-gmax)* (double) moveInterval->doubleValue(), gmax*((double) moveInterval->doubleValue()),2);
            //directionVector[myID]=directionVector[myID]+delta_g;
            if (fabs(gmax*((double) mobilitySelectPeriod->doubleValue()))>3.1415)
            {
                rotation=uniform(-3.1415,3.1415);
            }
            else
            {
                rotation=uniform((-gmax)* (double) mobilitySelectPeriod->doubleValue(), gmax*((double) mobilitySelectPeriod->doubleValue()),2);
            }
            directionVector[myID]=directionVector[myID]+rotation;
            if(fabs(directionVector[myID])>3.14159)
                directionVector[myID]=directionVector[myID]-sgn(directionVector[myID])*6.28318;
            selectNewMobilityParameters[myID]=false;
            //EV <<"Node ["<< myID<<"]: gmax = "<<gmax<<", delta_g = "<<(delta_g/3.1415)*180<<", amax = "<< amax<< " maxSpeedValue = "<< maxSpeedValue<<", delta_v ="<< delta_v<<endl;
            //EV <<"Node ["<< myID<<"]: gmax = "<<gmax<<", rotation = "<<(rotation/3.1415)*180<<", amax = "<< amax<< " maxSpeedValue = "<< maxSpeedValue<<", new Speed ="<< newSpeed<<endl;
            EV <<"Node["<< myID<<"]: gmax = "<<gmax<<", rotation = "<<rotation<<", amax = "<< amax<< " maxSpeedValue = "<< maxSpeedValue<<", new selected Speed ="<< newSpeed<< " new selected Direction = "<<directionVector[myID]<< endl;

        }
        else
        {
            rotation=0;
            EV <<"Node["<< myID<<"]: keeping same speed and direction"<<endl;
        }

    }
    else
    {
        double maxSpeedVariation=((double) mobilitySelectPeriod->doubleValue() )*amax;
        speedVector[myID]= ((maxSpeedVariation+speedVector[myID]) <= ((double) maxSpeed->doubleValue()))? (maxSpeedVariation+speedVector[myID]): (double) maxSpeed->doubleValue() ;
        double bestDirection=atan2(database[closestGroupMate].y-database[myID].y, database[closestGroupMate].x-database[myID].x);
        /*while(bestDirection<0)
	{
	  bestDirection+=6.283;
	}
      while(bestDirection>6.283)
	{
	  bestDirection-=6.283;
	  }*/

        rotation=  bestDirection- directionVector[myID];
        if(fabs(rotation)>3.14159)
            rotation=rotation-sgn(rotation)*6.28318;

        //EV <<"Node["<<myID<<"]: bestDirection = "<<(bestDirection/3.14159)*180<<", directionVector[myID] = "<<(directionVector[myID]/3.14159)*180<< ",  rotation = "<< (rotation/3.14195)*180<< endl;
        EV <<"Node["<<myID<<"]: bestDirection = "<<bestDirection<<", directionVector[myID] = "<<directionVector[myID]<< ",  rotation = "<< rotation<< endl;

        if(fabs(rotation)>=gmax*((double) moveInterval->doubleValue()))
        {
            if(rotation>0)
                directionVector[myID]=directionVector[myID]+(gmax*((double) moveInterval->doubleValue())-1e-8);
            else
                directionVector[myID]=directionVector[myID]-(gmax*((double) moveInterval->doubleValue())-1e-8);
            if(directionVector[myID]<-3.14159)
            {
                directionVector[myID]+=6.28318;
            }
            if(directionVector[myID]>3.14159)
            {
                directionVector[myID]-=6.28318;
            }
        }
        else
            directionVector[myID] = bestDirection;
        //EV <<"Node["<<myID<<"]: my position: <"<<database[myID].x<<","<<database[myID].y<<">; closest neighbor (Node ["<<closestGroupMate<<"]): <"<<database[closestGroupMate].x<<","<<database[closestGroupMate].y<<">; my direction: "<<(directionVector[myID]/3.14159)*180<<endl;
        EV <<"Node["<<myID<<"]: my position: <"<<database[myID].x<<","<<database[myID].y<<">; closest neighbor (Node ["<<closestGroupMate<<"]): <"<<database[closestGroupMate].x<<","<<database[closestGroupMate].y<<">; my direction: "<<directionVector[myID]<<endl;

    }
    if(collisionAvoidance)
    {
        checkCollisions(deltaTheta, deltaV);

        speedVector[myID]=speedVector[myID]+deltaV;
        EV <<"Node["<<myID<<"]: speed correction deltaV = "<<deltaV<< " applied, new speed before check = "<<speedVector[myID]<<endl;
        speedVector[myID]=(speedVector[myID]>((double) maxSpeed->doubleValue()))?((double) maxSpeed->doubleValue()):speedVector[myID];
        speedVector[myID]=(speedVector[myID]<((double) minSpeed->doubleValue()))?((double) minSpeed->doubleValue()):speedVector[myID];
        EV <<"Node["<<myID<<"]: speed correction deltaV = "<<deltaV<< " applied, original speed = "<<originalSpeed<<", new speed =  "<<speedVector[myID]<<endl;

        directionVector[myID]=directionVector[myID]+deltaTheta;
        if(directionVector[myID]<-3.14159)
        {
            directionVector[myID]+=6.28318;
        }
        if(directionVector[myID]>3.14159)
        {
            directionVector[myID]-=6.28318;
        }
        //EV <<"Node["<<myID<<"]: direction correction selected, new direction =  "<<directionVector[myID]<<endl;
    }


    EV <<"Node["<<myID<<"]: total speed variation: "<<speedVector[myID]-originalSpeed<<", original direction: "<<originalDirection<< ", final direction: "<<directionVector[myID]<<endl;
    //evaluate the total distance covered until the next update
    double dist = moveInterval->doubleValue() * speedVector[myID];
    //evaluate the distance covered in each direction
    dX = (dist * cos(directionVector[myID]));
    dY = (dist * sin(directionVector[myID]));

    //d("al:"<<alfa<<" d:"<<distance );

    //update the position <x,y>
    x = (x + dX);
    y = (y + dY);
    //do not go outside the map
    if(0 == moveKind)
    {
        correctedDirection=torus(x,y);
    }
    else
    {
        correctedDirection=rebound(x,y);
    }
    if(correctedDirection)
    {
        if(directionVector[myID]<-3.14159)
        {
            directionVector[myID]+=6.28318;
        }
        if(directionVector[myID]>3.14159)
        {
            directionVector[myID]-=6.28318;
        }
    }
    stepsNum++;
    partial+= speed;

    return (double) moveInterval->doubleValue();
}


void MoMo::initialize()
{

    cGate *g = gate("out");

    //pointer to the physic module that
    //stores the actual position
    bool TOMACS_FILES=(bool)  getParentModule()->getParentModule()->par("TOMACS_FILES");
    mobilityFlag=(bool) getParentModule()->getParentModule()->par("mobility");
    physic =(Physic*) g->getNextGate()->getOwnerModule();
    mh=getParentModule();
    myID = (int) mh->getIndex();
    hostNum= (int) getParentModule()->getParentModule()->par("dim");
    mobilePerc=(double) getParentModule()->getParentModule()->par("mobilePercentage");
    groupsFileName=(string) par("groupsFileName").stdstringValue();
    groupModality=0;
    group=true;
    caDebugFlag=false;
    graphicsFlag=par("graphicalOutput");
    collisionAvoidance=(bool) par("collisionAvoidance");
    collisionAvoidedSignal = registerSignal("avoidedCollisions");
    collisionConflictSignal = registerSignal("conflictingCollisions");
    fstream caDebug;
    if(caDebugFlag)
    {
        caDebug.open("collisionDebug.txt", ios::out);
        caDebug.close();
    }
    bool mobileSection=false;
    for (int i=0;i<NMAX;i++)
        localGroupVector[i]=false;
    if(TOMACS_FILES)
    {
        if(myID==0)
        {
            groupIDVector[myID]=0;
        }
        else
            if(myID==1)
            {
                groupIDVector[myID]=1;

            }
            else
            {
                groupIDVector[myID]=1;
            }
    }
    else
    {
        /*if(myID<10)
		groupIDVector[myID]=0;
	if((myID>=10) && (myID<21))
		groupIDVector[myID]=1;
	if(myID>=21)
		groupIDVector[myID]=2;
         */
        /*
	if(myID<10)
	{
		groupIDVector[myID]=0;
	}

	if((myID>=10) && (myID<15))
	{
		groupIDVector[myID]=1;
	}
	if((myID>=15) && (myID<19))
	{
		groupIDVector[myID]=2;
	}

	if((myID>=19) && (myID<21))
	{
		groupIDVector[myID]=3;
	}
	if((myID>=21) && (myID<26))
	{
		groupIDVector[myID]=4;
	}
	if((myID>=26) && (myID<30))
	{
		groupIDVector[myID]=5;
	}
	if((myID>=30) && (myID<32))
	{
		groupIDVector[myID]=6;
	}*/
        if(myID<4)
            groupIDVector[myID]=0;
        if((myID>=4) && (myID<8))
            groupIDVector[myID]=1;
        if((myID>=8) && (myID<12))
            groupIDVector[myID]=2;
        if(myID>=12)
            groupIDVector[myID]=3;
    }
    if(graphicsFlag)
    {
        cDisplayString &mhDS=mh->getDisplayString();
        if(group)
        {
            switch(groupIDVector[myID])
            {
            case 0:
            {
                mhDS.setTagArg("b",3,"white");
            }
            break;
            case 1:
            {
                mhDS.setTagArg("b",3,"red");
            }
            break;
            case 2:
            {
                mhDS.setTagArg("b",3,"yellow");
            }
            break;
            case 3:
            {
                mhDS.setTagArg("b",3,"green");
            }
            break;
            default:
                mhDS.setTagArg("b",3,"blue");
            }
        }
        else
            mhDS.setTagArg("b",3,"blue");
    }
    EV <<"Lowest mobileID: "<<floor(hostNum*(1-mobilePerc/100))<<endl;
    //if (myID<=ceil(hostNum*mobilePerc)&&(myID>0)&&(myID<hostNum-1)) //inverted to stop lowID nodes and move the others
    if (myID>=floor(hostNum*(1-mobilePerc/100)))
        mobileSection=true;
    if((mobilityFlag)&&(mobileSection))
    {
        EV <<"Node ["<< myID<<"]: mobility on"<<endl;
        alfa = 0;
        minX = 0;
        maxX = par("XRange");
        minY = 0;
        maxY = par("YRange");
        //steps = 0;
        correctedDirection=false;
        numGroups= (int) getParentModule()->getParentModule()->par("numGroups");

        //groupSize=hostNum/numGroups;//requires numGroups to be a divider of hostNum...
        //groupIDVector[myID]=myID/groupSize;
        localGroupVector[myID]=true;
        loadGroupMembers();
        //HACK: works only for the soccer scenario to distinguish the two teams...

        //int xDisp= (int) floor(displayScaleFactor*database[myID].x);
        //int yDisp = (int) floor(displayScaleFactor*database[myID].y);
        //	EV << "xDisp: "<<xDisp<<", yDisp: "<<yDisp<<endl;
        //sprintf(str,displayS, xDisp,yDisp, "black");

        //char str[50];
        //EV <<"Inspector: "<<"set position of node "<<i<<endl;
        //EV <<"Inspector: x: "<< mh->par("x").doubleValue()<<", y: "<< mh->par("y").doubleValue()<<endl;
        //EV <<"Inspector: x: "<< database[i].x<<", y: "<< database[i].y<<endl;

        //EV <<"Inspector: xDisp: "<<xDisp<<", yDisp: "<<yDisp<<endl;

        //delete mhDS;
        EV <<"My group is "<< groupIDVector[myID]<<endl;
        moveInterval = &par("moveInterval");
        minSpeed = &par("minSpeed");
        maxSpeed = &par("maxSpeed");
        mobilitySelectPeriod= &par("mobilitySelectPeriod");
        moveKind=&par("movKind");
        amax= (double) par("amax");
        gmax= (double) par("gmax");
        momoProximityBound= (double) par("proximityBound");
        groupBoundUpdatePeriod= (double) par("groupBoundUpdatePeriod");
        checkCollisionDistance= (double) par("checkCollisionDistance");
        minDistanceCA=(double) par("minDistanceCA");
        minReactionTime=minDistanceCA/((double) maxSpeed->doubleValue());
        rho=(double) par("rho");
        forcedStatus=false;
        checkUpdateFlag=false;

        cMessage *moveMsg = new cMessage("Move");

        //start moving
        moveMsg->setKind(MOVE);
        scheduleAt(simTime()+0.01, moveMsg);

        if(groupBoundUpdatePeriod!= moveInterval->doubleValue())
        {
            groupBoundUpdate=true;
            cMessage *updateMsg = new cMessage("Update");
            //start moving
            updateMsg->setKind(UPDATE);
            scheduleAt(simTime()+0.01, updateMsg);
        }
        else
            groupBoundUpdate=false;


        cMessage *selectMsg = new cMessage("Select");
        selectMsg->setKind(SELECT);
        //start moving
        scheduleAt(simTime()+0.005, selectMsg);

        //statistical variables
        stepsNum =0;
        partial =0;
    }
}

void MoMo::recordMobilityEffect(double x, double y, double alpha, double time)
{
    double directionVariation=directionVector[myID]-alpha;
    if(fabs(directionVariation)>3.14159)
        directionVariation=directionVariation-sgn(directionVariation)*6.28318;

    double requiredSpeed=sqrt(pow(y-database[myID].y,2)+pow(x-database[myID].x,2))/time;

    if((fabs(directionVariation)>gmax*time))
    {
        //        EV <<"Node["<<myID<<"]: old direction = "<< (alpha/3.14159)*180<<endl;
        //        EV <<"Node["<<myID<<"]: new direction = "<< (directionVector[myID]/3.14159)*180<<endl;
        //        EV <<"Node["<<myID<<"]: directionVariation = "<< (directionVariation/3.14159)*180<<endl;
        //        EV <<"Node["<<myID<<"]: maxRotation = "<< (gmax*time/3.14159)*180<<endl;
        EV <<"Node["<<myID<<"]: old direction = "<< alpha<<endl;
        EV <<"Node["<<myID<<"]: new direction = "<< directionVector[myID]<<endl;
        EV <<"Node["<<myID<<"]: directionVariation = "<< directionVariation<<endl;
        EV <<"Node["<<myID<<"]: maxRotation = "<< gmax*time<<endl;
        if(!correctedDirection)
        {
            EV <<"Node["<<myID<<"]: rotation violation!"<<endl;
            rotationViolation[myID]++;
        }
        else
        {
            EV <<"Node["<<myID<<"]: rotation violation due to rebound, ignoring it"<<endl;
        }
    }
    double epsilon=1e-8;
    double maxSpeedValue=(((double) mobilitySelectPeriod->doubleValue() )*amax <= (double) maxSpeed->doubleValue())? ((double) mobilitySelectPeriod->doubleValue() )*amax: (double) maxSpeed->doubleValue() ;
    if(requiredSpeed>maxSpeedValue+epsilon)
    {
        EV <<"Node["<<myID<<"]: required speed = "<< requiredSpeed<<endl;
        EV <<"Node["<<myID<<"]: maxSpeed = "<< maxSpeed->doubleValue()<<endl;
        EV <<"Node["<<myID<<"]: speed violation!"<<endl;
        speedViolation[myID]++;
    }
}



void MoMo::handleMessage(cMessage *msg)
{
    int kind=msg->getKind();
    switch(kind)
    {
    case MOVE:
    {
        double x,y,v,alpha;

        d("MoMo Mobility");

        //printf("%d\n",getParentModule()->getIndex());
        //get the current position from the physic module
        //physic->getPos(x, y);

        //evaluate the new position
        x=database[myID].x;
        y=database[myID].y;
        alpha=directionVector[myID];
        v=speedVector[myID];
        double time = momoMobility(x,y);
        recordMobilityEffect(x,y,alpha,time);
        correctedDirection=false;
        database[myID].x=x;
        database[myID].y=y;
        //int xDisp= (int) floor(displayScaleFactor*database[myID].x);
        //(int yDisp = (int) floor(displayScaleFactor*database[myID].y);
        //char str[50];
        EV <<"Node["<<myID<<"]: my position: <"<<database[myID].x<<","<<database[myID].y<<">"<<endl;
        if(graphicsFlag)
        {
            cDisplayString &mhDS=mh->getDisplayString();

            if(group)
            {
                switch(groupIDVector[myID])
                {
                case 0:
                {
                    mhDS.setTagArg("b",3,"white");
                    //sprintf(str,displayS, xDisp,yDisp, "blue");
                }
                break;
                case 1:
                {
                    mhDS.setTagArg("b",3,"red");
                }
                break;
                case 2:
                {
                    mhDS.setTagArg("b",3,"yellow");
                }
                break;
                case 3:
                {
                    mhDS.setTagArg("b",3,"green");
                }
                break;

                default:
                    mhDS.setTagArg("b",3,"blue");
                }
            }
            else
                mhDS.setTagArg("b",3,"blue");
        }
        physic->setPos(x,y);//This avoids inconsistencies between the position database and the variables in the physic module of each node
        cMessage *moveMsg = new cMessage("Move",MOVE);

        moveMsg->addPar("x") = x;
        moveMsg->addPar("y") = y;

        //inform to the physic module about
        //the new position so it can be displayed
        send(moveMsg,"out");

        //tell to the physic module to move
        scheduleAt(simTime()+time, msg);
    }
    break;
    case UPDATE:
    {
        checkUpdateFlag=true;
        //tell to the physic module to move
        scheduleAt(simTime()+groupBoundUpdatePeriod, msg);
    }
    break;
    case SELECT:
    {
        selectNewMobilityParameters[myID]=true;
        scheduleAt(simTime()+mobilitySelectPeriod->doubleValue(), msg);
    }
    break;
    case SWITCH_BEHAVIOR:
    {

        //int xDisp= (int) floor(displayScaleFactor*database[myID].x);
        //int yDisp = (int) floor(displayScaleFactor*database[myID].y);
        cDisplayString &mhDS=mh->getDisplayString();
        if(group)
        {
            EV <<"Node["<<myID<<"]: group off"<<endl;
            rho=0;
            group=false;
            if(graphicsFlag)
                mhDS.setTagArg("b",3,"blue");
        }
        else
        {
            EV << "Node["<<myID<<"]: group on"<<endl;
            rho=par("rho");
            group=true;
            if(graphicsFlag)
            {
                switch(groupIDVector[myID])
                {
                case 0:
                {
                    mhDS.setTagArg("b",3,"white");
                    //sprintf(str,displayS, xDisp,yDisp, "blue");
                }
                break;
                case 1:
                {
                    mhDS.setTagArg("b",3,"red");
                }
                break;
                case 2:
                {
                    mhDS.setTagArg("b",3,"yellow");
                }
                break;
                case 3:
                {
                    mhDS.setTagArg("b",3,"green");
                }
                break;

                default:
                    mhDS.setTagArg("b",3,"blue");
                }

            }
        }
        //mh->setDisplayString(0,str,true);
        delete msg;
    }
    break;
    case SWITCH_MODALITY:
    {
        groupModality=(int) msg->par("modality");
    }
    }
}



void MoMo::finish()
{
    if(mobilityFlag)
    {
        d("MoMo random says bye");
        FILE* fout = fopen("collectedData.dat","a");
        fprintf(fout,"\nSpeed average............... %.2f\n",partial/stepsNum);
        fclose(fout);
    }
}


