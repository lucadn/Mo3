function [theta]=Mo3_ObstacleAvoidance(M,x,y,theta,ObsList,numObstacles,d_OA_trigger,thetaEpsilon)
%Function implementing the Obstacle Avoidance module of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

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
    %             if (nnz(splitForbiddenRangeNodes)>0)
    %                 fprintf('Split\n');
    %             end
    %             if any(detected)
    %                 fprintf('Obstacle detected\n');
    %                 plotEverything(xStart,yStart,xPath,yPath,x_min, x_max, y_min, y_max,ObsList);
    %                 y11=tan(thetamin)*(xtot-x)+y;
    %                 plot(xtot,y11,'-');
    %                 y12=tan(thetamax)*(xtot-x)+y;
    %                 plot(xtot,y12,'-');
    %                 if(abs(theta(k))<pi/2)
    %                     xNode=x:0.1:x_max;
    %                 else
    %                     xNode=x_min:0.1:x;
    %                 end
    %                 dirNode=tan(theta(k))*(xNode-x)+y;
    %                 plot(xNode,dirNode,'--')
    %                 close
    %             end
%     if nnz(sqrt((x-ObsList(1,2)).^2+(y- ObsList(1,3)).^2)<1.2*ObsList(1,4)/2)>0
%         plotEverything(xStart,yStart,xPath,yPath,x_min, x_max, y_min, y_max,ObsList);
%         for dbInd=1:length(x)
%             y11=tan(thetamin(dbInd)).*(xtot-x(dbInd))+y(dbInd);
%             plot(xtot,y11,'-');
%             y12=tan(thetamax(dbInd)).*(xtot-x(dbInd))+y(dbInd);
%             plot(xtot,y12,'-');
%             if(abs(theta(dbInd))<pi/2)
%                 xNode=x:0.1:x_max;
%             else
%                 xNode=x_min:0.1:x;
%             end
%             dirNode=tan(theta(dbInd))*(xNode-x(dbInd))+y(dbInd);
%             plot(xNode,dirNode,'--')
%         end
%         close
%     end
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
                %                     plotEverything(xStart,yStart,xPath,yPath,x_min, x_max, y_min, y_max,ObsList);
                %                     y11=tan(thetamin)*(xtot-x)+y;
                %                     plot(xtot,y11,'-');
                %                     y12=tan(thetamax)*(xtot-x)+y;
                %                     plot(xtot,y12,'-');
                %                     if(abs(theta(k))<pi/2)
                %                         xNode=x:0.1:x_max;
                %                     else
                %                         xNode=x_min:0.1:x;
                %                     end
                %                     dirNode=tan(theta(k))*(xNode-x)+y;
                %                     plot(xNode,dirNode,'--')
                %                     close
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
                %close
            end
        end
    end
end