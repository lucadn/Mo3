%Main script setting up and running the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

clear
close all
BMfilename='Mo3_BindingMatrix.txt';
OLfilename='Mo3_ObstacleList.txt';
%System parameters
M=10;
x_min=0;
x_max=50;
y_min=0;
y_max=50;
tmax=1000;
dt=0.01; %position update period
%xtot=x_min:0.1:x_max;


g_max=pi/4;
a_max=2;
v_min=0.001;
v_max=0.5;

%Individual Mobility module input parameters
T_IM=10*dt;% Individual Mobility update period

%Correlated Mobility module input parameters
T_CM=dt;
Dc=5;
rho_min=0.5;
groupingStrategy=2; %1: centroid of group mates; 2: closest mate not in the connected set

%Collision Avoidance module input parameters
T_CA=dt;% Collision Avoidance update period
d_CA_trigger=20;
d_CA_min=20;
theta_CA=pi/100;

%Obstacle Avoidance module input parameters
T_OA=dt;% Obstacle Avoidance update period
d_OA_trigger=1;
thetaEpsilon=pi/100;
ObsList=[];

%Upper Bound Enforcement module input parameters
T_UB=dt;


t=0:dt:tmax-dt;

%System initialization
x=x_min+(x_max-x_min)*rand(1,M);
y=y_min+(y_max-y_min)*rand(1,M);
v=v_min+(v_max-v_min)*rand(1,M);
theta=-pi+2*pi*rand(1,M);

correlatedMobilityFlag=true;
collisionAvoidanceFlag=true;
obstacleAvoidanceFlag=true;
upperBoundsEnforcementFlag=true;

if (obstacleAvoidanceFlag)
    ObsList=Mo3_LoadObstacleList(OLfilename);
    numObstacles=size(ObsList,1);
    [x,y]=Mo3_ForceOutsideObstacles(x,y,x_min,x_max,y_min,y_max, ObsList);
    %thetamin=zeros(numObstacles,M);
    %thetamax=zeros(numObstacles,M);
end

if(correlatedMobilityFlag)
    [BM, nextBMSwitchTime, BMfPos]=Mo3_LoadBindingMatrix(BMfilename,0,M);
end

xStart=x;
yStart=y;
xPath(1,:)=xStart;
yPath(1,:)=yStart;
vHistory(1,:)=v;
thetaHistory(1,:)=theta;
groupingConditionVector=false(1,M);

for i=1:length(t)
    if(mod(t(i),1000)==0)
        fprintf('t=%f\n',t(i));
    end
    v0=v;
    theta0=theta;
    %Step 0: calculate distances once and for all to save time later.
    dMatrix=Inf*ones(M,M);
    for w=1:M-1
        for r=w+1:M
            dMatrix(w,r)=Mo3_EuclideanDistance(w,r,x,y);
            dMatrix(r,w)=dMatrix(w,r);
        end
    end
    
    %Step 1 - Individual Mobility
    if(mod(t(i),T_IM)==0)
        [v,theta]=Mo3_Boundless(v,theta,v_min,v_max, a_max, g_max,T_IM);
        %fprintf('Speed vector update at t=%f \n',t(i));
    end
    
    %Step 2 - Correlated Mobility
    if(correlatedMobilityFlag)
        if (nextBMSwitchTime>-1) &&(mod(t(i),nextBMSwitchTime)<t(i))
            [BM, nextBMSwitchTime, BMfPos]=Mo3_LoadBindingMatrix(BMfilename,BMfPos,M);
        end
        if(mod(t(i),T_CM)==0)
            [v,theta,groupingConditionVector]=Mo3_CorrelatedMobility(M, x,y,dMatrix, v_max, v, theta, BM, Dc,rho_min, groupingStrategy);
        end
    end
    
    %Step 3 - Collision Avoidance
    if(collisionAvoidanceFlag && (mod(t(i),T_CA)==0))
        [v_out,frontalCollisionFlag,theta_out]= Mo3_CollisionAvoidance(M,x_min,x_max,y_min,y_max, v_min,v_max,x,y,dMatrix,v,theta,d_CA_trigger,d_CA_min,theta_CA);
        while frontalCollisionFlag
            theta=theta_out;
            [v_out,frontalCollisionFlag,theta_out]= Mo3_CollisionAvoidance(M,x_min,x_max,y_min,y_max, v_min,v_max,x,y,dMatrix,v,theta,d_CA_trigger,d_CA_min,theta_CA);
        end
        v=v_out;
        
    end
    
    %Step 4 - Obstacle Avoidance
    if(obstacleAvoidanceFlag && (mod(t(i),T_OA)==0))
        [theta]=Mo3_ObstacleAvoidance(M,x,y,theta,ObsList,numObstacles,d_OA_trigger,thetaEpsilon); 
    end
    
    %Step 5 - Upper Bounds Enforcement
    if(upperBoundsEnforcementFlag)
        [v,theta]=Mo3_UpperBoundsEnforcement(v,theta,v0,theta0,v_max,v_min, a_max, g_max,dt);
    end
    
    %Determine the displacement according to the updated speed vectors
    dX=v.*cos(theta)*dt;
    dY=v.*sin(theta)*dt;
    x=x+dX;
    y=y+dY;
    
    %Apply the perfect reflection law to compensate for movement area
    %boundary violations
    [x, y, theta, xViolations, yViolations]=Mo3_Rebound(x, y, theta, x_min,y_min,x_max,y_max);

    dX(xViolations)=-dX(xViolations);
    dY(yViolations)=-dY(yViolations);
    x(xViolations)=x(xViolations)+dX(xViolations);
    y(yViolations)=y(yViolations)+dY(yViolations);

    xPath(i,:)=x;
    yPath(i,:)=y;
    vHistory(i,:)=v;
    thetaHistory(i,:)=theta;
%     if all(groupingConditionVector)
%         plotEverything(xStart,yStart,xPath,yPath,x_min, x_max, y_min, y_max,ObsList);
%     end
end
Mo3_PlotEverything(xStart,yStart,xPath,yPath,x_min, x_max, y_min, y_max,ObsList);
stepSize=500;
%scatterEverything(xPath,yPath,x_min, x_max, y_min, y_max,ObsList,stepSize);