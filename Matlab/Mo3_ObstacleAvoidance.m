function [theta]=Mo3_ObstacleAvoidance(M,x,y,theta,ObsList,numObstacles,d_OA_trigger,thetaEpsilon)
%Function implementing the Obstacle Avoidance module of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

%IMPORTANT CAVEAT. As pointed out in the research paper, the enforcement of
%upper bounds by the UpperBoundsEnforcement module may lead to a failure in avoiding a collision with another
%node or with an obstacle. UpperBoundsEnforcement will cause Obstacle Avoidance failures in
%particular under the following conditions:
%1) d_OA_trigger set to small values (e.g. < 5 m): in this case a node only
%reacts to an obstacle when it is very close to it, and the Obstacle
%Avoidance rule may introduce a variation to theta too large compared to the
%current bound on rotation speed. 
%2) Activation of the Correlated Mobility rule: when a node needs to
%reconnect to its group on the other side of an obstacle, the combination
%of the Correlate Mobility and Obstacle Avoidance will lead to a pattern
%that follows the perimeter of the obstacle. At times the
%UpperBoundsEnforcement rule may enforce a correction on theta that
%causes the node to collide with the obstacle.
%
%In the current implementation of the model, when a collision occurs, the
%node will pass through the obstacle and eventually exit it. More sophisticated approaches to
%address this event might include applying a modified version of the
%rebound rule to immediately place the node outside the obstacle; their
%implementation is left for future work.


forbiddenRange=cell(M,2*numObstacles);
for j=1:numObstacles
    thetarange=zeros(2,M);
    %For each obstacle check whether its distance from the node is
    %less than d_OA_trigger, and determine the corrsponding forbidden range
    switch(ObsList(j,1))
        case 1
            detected=Mo3_RectangleMinDistance(x,y,ObsList(j,2), ObsList(j,3),ObsList(j,4),ObsList(j,5),d_OA_trigger);
            [thetarange(1,:), thetarange(2,:)]=Mo3_RectangleForbiddenRange(x,y,ObsList(j,2), ObsList(j,3),ObsList(j,4),ObsList(j,5));
        case 2
            detected=Mo3_EllipseMinDistance(x,y,ObsList(j,2), ObsList(j,3),ObsList(j,4),ObsList(j,5),d_OA_trigger);
            [thetarange(1,:), thetarange(2,:)]=Mo3_EllipseForbiddenRange(x,y,ObsList(j,2), ObsList(j,3),ObsList(j,4),ObsList(j,5));
    end
    thetamin=min(thetarange,[],1);
    thetamax=max(thetarange,[],1);
    splitForbiddenRangeNodes=((thetarange(1,:).*thetarange(2,:)<0) & (x>=ObsList(j,2)));
    singleForbiddenRangeNodes=((thetarange(1,:).*thetarange(2,:)>=0) | (x<ObsList(j,2)));
    for k=1:M
        %If the obstacle was detected,add the corresponding forbidden range
        %to the list of forbidden ranges,otherwise ignore it
        if detected(k)
            if(singleForbiddenRangeNodes(k))
                forbiddenRange(k,2*(j-1)+1)={[thetamin(k) thetamax(k)]};
                forbiddenRange(k,2*(j-1)+2)={[0 0]};
                
            end
            if(splitForbiddenRangeNodes(k))
                forbiddenRange(k,2*(j-1)+1)={[-pi, thetamin(k)]};
                forbiddenRange(k,2*(j-1)+2)={[thetamax(k),pi]};
            end
        else
            forbiddenRange(k,2*(j-1)+1)={[0 0]};
            forbiddenRange(k,2*(j-1)+2)={[0 0]};
        end
    end
end
%Merge the forbidden ranges
for k=1:M
    for l=1:numObstacles-1
        thetarange1=cell2mat(forbiddenRange(k,2*(l-1)+1));
        for m=l+1:numObstacles
            thetarange2=cell2mat(forbiddenRange(k,2*(m-1)+1));
            if (thetarange1(1)<=thetarange2(1) && thetarange1(2)>=thetarange2(1))||(thetarange2(1)<=thetarange1(1) && thetarange2(2)>=thetarange1(1))
                thetarange1(1)=min(thetarange1(1),thetarange2(1));
                thetarange1(2)=max(thetarange1(2),thetarange2(2));
                thetarange2(1)=thetarange1(1);
                thetarange2(2)=thetarange1(2);
                forbiddenRange(k,2*(l-1)+1)={thetarange1};
                forbiddenRange(k,2*(m-1)+1)={thetarange2};
            end
        end
    end
end
%Check if the current direction will lead to a collision, and if this is
%the case change it.
collisionRisks=false(M,2*numObstacles);
for k=1:M
    for l=1:numObstacles
        for m=1:2
            thetarange=(cell2mat(forbiddenRange(k,2*(l-1)+m)))';
            collisionRisks(k,2*(l-1)+m) = (theta(k)>=thetarange(1)) & (theta(k)<=thetarange(2));
            if collisionRisks(k,2*(l-1)+m)
                if(thetarange(2)<pi)
                    thetaUp=abs(thetarange(2)-theta(k));
                else
                    thetaUp=Inf;
                end
                if(thetarange(1)>-pi)
                    thetaDown=abs(theta(k)-thetarange(1));
                else
                    thetaDown=Inf;
                end
                if (thetaUp<thetaDown)
                    theta(k)=theta(k)+thetaUp+thetaEpsilon;
                    if theta(k)>pi
                        theta(k)=theta(k)-2*pi;
                    end
                else
                    theta(k)=theta(k)-(thetaDown+thetaEpsilon);
                    if theta(k)<-pi
                        theta(k)=theta(k)+2*pi;
                    end
                end
            end
        end
    end
end