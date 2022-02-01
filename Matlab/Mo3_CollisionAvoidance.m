function [vOut, frontalCollisionFlag, thetaOut] = Mo3_CollisionAvoidance(M,x_min,x_max,y_min,y_max, v_min,v_max, x,y,dMatrix,vIn,theta,d_CA_trigger,d_CA_min,theta_CA)
%Function implementing the Collision Avoidance module of the Mo3 mobility model, as defined in 
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility Model for
%future generation mobile wireless networks", submitted to IEEE Access

%   The Collision Avoidance module determines whether the trajectories of
%   nodes cross, and if the distance between nodes is < d_CA_min when
%   either of them is at the crossing point. If this is the case the speed
%   of nodes is increased or decreased as needed to ensure that the
%   distance above is at least d_CA_min.

vOut=vIn;
thetaOut=theta;
frontalCollisionFlag=0;

for i=1:M
    close all
    %PATH CROSSING IDENTIFICATION
    %Step 1: determine the set of nodes that may potentially collide with node i. 
    detectedNodes=dMatrix(i,:)<d_CA_trigger;
    detectedNodesIDs=find(detectedNodes);
    thetaDetected=theta(detectedNodes);
%     if nnz(detectedNodes)>0
%         fprintf('Collision risk\n')
%     end
    dX=x(detectedNodes)-x(i);
    dY=y(detectedNodes)-y(i);
    D=cos(thetaDetected)*sin(theta(i))-sin(thetaDetected)*cos(theta(i));
    if ~all(D)
        potentialFrontalCollisions=(D==0);
        thetaCheck=thetaDetected(potentialFrontalCollisions)==theta(i);
        if ~all(thetaCheck)
            %At least in one case the angle of a node on a potential
            %frontal collision course is not equal to the one of the node
            %under consideration. Alter the direction of the node under
            %consideration and exit
            frontalCollisionFlag=1;
            thetaOut(i)=thetaOut(i)+theta_CA;
            return
        end
    end
    r=(dY.*cos(thetaDetected)-dX.*sin(thetaDetected))./D;
    s=(dY*cos(theta(i))-dX*sin(theta(i)))./D;
    collidingNodes=r>0 & s>0 & r<Inf & s<Inf;
    %Step 2: determine the crossing points and check if they fall in the
    %movement area.
    xC=x(i)+cos(theta(i)).*r(collidingNodes);
    yC=y(i)+sin(theta(i)).*r(collidingNodes);
    relevantCollidingNodes=(xC>=x_min) & (xC<=x_max) & (yC>=y_min) & (yC<=y_max);
    collidingNodesIDs=detectedNodesIDs(collidingNodes);
    collidingNodesIDs=collidingNodesIDs(relevantCollidingNodes);
%     plotCollisionCases(x,y,theta,i, collidingNodesIDs)
%     scatter(xC,yC,30,'k','filled');
%     plotCollisionCases(x,y,theta,i, find((1:M)~=i))
    if(isempty(collidingNodesIDs))%No possible collisions: the PCN set is empty, so let's skip to the next node to save time.
        continue
    end
    %SPEED BOUNDS IDENTIFICATION
    vLowerBounds=v_min*ones(1,length(collidingNodesIDs));
    vUpperBounds=v_max*ones(1,length(collidingNodesIDs));
    %Step 1: determine the time of arrival at the crossing points
    diC=sqrt((x(i)-xC(relevantCollidingNodes)).^2+(y(i)-yC(relevantCollidingNodes)).^2);
    TiC=diC/vIn(i);
    dothersC=sqrt((x(collidingNodesIDs)-xC(relevantCollidingNodes)).^2+(y(collidingNodesIDs)-yC(relevantCollidingNodes)).^2);
    TothersC=dothersC ./ vIn(collidingNodesIDs);
    earlyNodes=(TothersC<TiC);
    lateNodes=(TothersC>TiC);
    simNodes=(TothersC==TiC);
    if any(simNodes)
        lowerIDSimNodes=simNodes.*(collidingNodesIDs<i);
        higherIDSimNodes=simNodes.*(collidingNodesIDs>i);
        earlyNodes=earlyNodes.*lowerIDSimNodes;
        lateNodes=lateNodes.*higherIDSimNodes;
    end
    %Step 2: determine the bounds to ensure that the distance when one of
    %the nodes is at the crossing point is at least d_CA_min
    vLowerBounds(lateNodes)=(vIn(collidingNodesIDs(lateNodes)).*(d_CA_min+diC(lateNodes)))./dothersC(lateNodes);
    vUpperBounds(earlyNodes)=(vIn(collidingNodesIDs(earlyNodes)).*(diC(earlyNodes)))./(dothersC(earlyNodes)+d_CA_min);
    
    %Step 3: find the vi' that satisfies all the bounds and is closest to
    %vi.
    lowerBound=max(vLowerBounds);
    upperBound=min(vUpperBounds);
    if (lowerBound>upperBound)
        %We are in trouble, there is no solution. Let's set the speed to
        %the bound that solves most collision risks, and let's check again
        %at the next update
        if nnz(lateNodes)>nnz(earlyNodes)
            vOut(i)=min(lowerBound,v_max);
        else
            vOut(i)=max(upperBound,v_min);
        end
        
    else
        %A range of allowed speeds exists. Let's check whether the current
        %speed is already within it.
        if vIn(i)>=lowerBound
            if vIn(i)<=upperBound
                vOut(i)=vIn(i);%no collision risk is identified, vi is already in the allowed range
            else
                vOut(i)=upperBound; %going too fast, let's slow down to avoid collisions
            end
        else
            vOut(i)=lowerBound; %going too slow, let's speed up to avoid collisions
        end
    end
end