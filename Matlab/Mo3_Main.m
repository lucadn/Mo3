%Main script setting up and running the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

clear
close all
BMfilename='Mo3_BindingMatrixExample1.txt';
OLfilename='Mo3_ObstacleList.txt';
%System parameters 
tridimMobility=false;%Tridimensional mobility flag. If set to true, disables the Collision Avoidance and Obstacle Avoidance modules automatically
M=4;%Total number of nodes. IMPORTANT: this value must match the size of the matrixes provided in the Binding Matrix file. 
x_min=0;
x_max=50;
y_min=0;
y_max=50;
z_min=0;
z_max=50;
tmax=1000; %simulation time
dt=0.01; %position update period

%Input parameters shared by multiple modules
g_max=(pi/2)/dt;
d_max=(pi/2)/dt;
a_max=5/dt;
v_min=0.001;
v_max=2;

%Individual Mobility module input parameters
T_IM=5*dt;

%Correlated Mobility module input parameters
T_CM=dt;
Dc=10;
rho_min=1;
groupingStrategy=2; %1: centroid of group mates; 2: closest mate not in the connected set

%Collision Avoidance module input parameters
T_CA=dt;
d_CA_trigger=20;
d_CA_min=20;
theta_CA=pi/100;

%Obstacle Avoidance module input parameters
T_OA=dt;
d_OA_trigger=10;
thetaOA=pi/100;
ObsList=[];

%Upper Bound Enforcement module input parameters
T_UB=dt;


t=0:dt:tmax-dt;

%System initialization
x=x_min+(x_max-x_min)*rand(1,M);
y=y_min+(y_max-y_min)*rand(1,M);
if (tridimMobility)
    z=z_min+(z_max-z_min)*rand(1,M);
else
    z=zeros(1,M);
end
v=v_min+(v_max-v_min)*rand(1,M);
theta=-pi+2*pi*rand(1,M);
if (tridimMobility)
    phi=-pi/2+pi*rand(1,M);
else
    phi=zeros(1,M);
end


correlatedMobilityFlag=true;
collisionAvoidanceFlag=true;
obstacleAvoidanceFlag=true;
upperBoundsEnforcementFlag=false;

if (tridimMobility)
    collisionAvoidanceFlag=false;
    obstacleAvoidanceFlag=false;
end

if (obstacleAvoidanceFlag)
    ObsList=Mo3_LoadObstacleList(OLfilename);
    numObstacles=size(ObsList,1);
    [x,y]=Mo3_ForceOutsideObstacles(x,y,x_min,x_max,y_min,y_max, ObsList);
end

if(correlatedMobilityFlag)
    [BM, nextBMSwitchTime, BMfPos]=Mo3_LoadBindingMatrix(BMfilename,0,M);
end

xStart=x;
yStart=y;
zStart=z;
xPath(1,:)=xStart;
yPath(1,:)=yStart;
zPath(1,:)=zStart;
vHistory(1,:)=v;
thetaHistory(1,:)=theta;
phiHistory(1,:)=phi;
groupingConditionVector=false(1,M);

for i=1:length(t)
    if(mod(t(i),1000)==0)
        fprintf('t=%f\n',t(i));
    end
    v0=v;
    theta0=theta;
    phi0=phi;
    %Step 0: calculate distances once and for all to save time later.
    dMatrix=Inf*ones(M,M);
    for w=1:M-1
        for r=w+1:M
            dMatrix(w,r)=Mo3_EuclideanDistance(w,r,x,y,z);
            dMatrix(r,w)=dMatrix(w,r);
        end
    end
    
    %Step 1 - Individual Mobility
    if(mod(t(i),T_IM)==0)
        [v,theta,phi]=Mo3_Boundless(v,theta,phi,v_min,v_max, a_max, g_max,d_max,tridimMobility, T_IM);
    end
    
    %Step 2 - Correlated Mobility
    if(correlatedMobilityFlag)
        if (nextBMSwitchTime>-1) &&(mod(t(i),nextBMSwitchTime)<t(i))
            [BM, nextBMSwitchTime, BMfPos]=Mo3_LoadBindingMatrix(BMfilename,BMfPos,M);
        end
        if(mod(t(i),T_CM)==0)
            [v,theta,phi,groupingConditionVector]=Mo3_CorrelatedMobility(M, x,y,z,dMatrix, v_max, v, theta,phi, BM, Dc,rho_min, groupingStrategy);
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
        [theta]=Mo3_ObstacleAvoidance(M,x,y,theta,ObsList,numObstacles,d_OA_trigger,thetaOA);

    end
    
    %Step 5 - Upper Bounds Enforcement
    if(upperBoundsEnforcementFlag)
        [v,theta,phi]=Mo3_UpperBoundsEnforcement(v,theta,phi, v0,theta0,phi0,v_max,v_min, a_max, g_max,d_max, dt);
    end
    
    %Determine the displacement according to the updated speed vectors
    dX=v.*cos(theta).*cos(phi)*dt;
    dY=v.*sin(theta).*cos(phi)*dt;
    dZ=v.*sin(phi)*dt;
    x=x+dX;
    y=y+dY;
    z=z+dZ;
    %Apply the perfect reflection law to compensate for movement area
    %boundary violations
    [x, y, z, theta, phi, xViolations, yViolations,zViolations]=Mo3_Rebound(x, y, z, theta, phi, x_min,y_min,z_min, x_max,y_max,z_max);

    dX(xViolations)=-dX(xViolations);
    dY(yViolations)=-dY(yViolations);
    dZ(zViolations)=-dZ(zViolations);
    x(xViolations)=x(xViolations)+dX(xViolations);
    y(yViolations)=y(yViolations)+dY(yViolations);
    z(zViolations)=z(zViolations)+dZ(zViolations);

    xPath(i,:)=x;
    yPath(i,:)=y;
    zPath(i,:)=z;
    vHistory(i,:)=v;
    thetaHistory(i,:)=theta;
    phiHistory(i,:)=phi;

end
if(~tridimMobility)
    Mo3_PlotEverything(xStart,yStart,xPath,yPath,x_min, x_max, y_min, y_max,ObsList);
else
    Mo3_PlotEverything3D(xStart,yStart,zStart,xPath,yPath,zPath,vHistory, thetaHistory,phiHistory,x_min, x_max, y_min, y_max,z_min,z_max);
end