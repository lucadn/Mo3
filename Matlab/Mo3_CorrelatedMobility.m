function [vOut, thetaOut,phiOut,groupingConditionVector] = Mo3_CorrelatedMobility(M, x,y,z,dMatrix, v_max, vIn, thetaIn, phiIn, BM, Dc,rho_min,groupingStrategy)
%Function implementing the Correlated Mobility module of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

vOut=vIn;
thetaOut=thetaIn;
phiOut=phiIn;
groupingConditionVector=false(1,M);

for i=1:M
    dMatrix(i,i)=0;% We set the distance of a node to itself to 0, in order to count it in its own connected set.
    %Step 1 - Determine bindings according to the current binding matrix
    groupMates=logical(BM(i,:));
    groupMatesIDs=find(groupMates);
    groupSize=length(groupMatesIDs);
    %Step 2 - Check binding conditions and determine the connected set
    connectedSet=(dMatrix(i,groupMates)<=Dc);
    connectedSetSize=nnz(connectedSet);
    %Step 3 - Check grouping condition
    if(groupSize==1)%If a node is alone in its group rho is automatically set to 1
        rho=1;
    else
        rho=(connectedSetSize-1)/(groupSize-1);
    end
    groupingCondition=(rho>=rho_min);
    groupingConditionVector(i)=groupingCondition;
    %If the grouping condition is not satisfied enter in Forced mode
    if(~groupingCondition)    
        switch (groupingStrategy)
            case 1
                %Move toward the centroid - not described in the research paper. This approach determines the centroid of the positions of 
                %the mates and sets the centroid as target destination
                otherGroupMateIDs=groupMatesIDs(find(groupMatesIDs~=i));
                xMates=x(otherGroupMateIDs);
                yMates=y(otherGroupMateIDs);
                zMates=z(otherGroupMateIDs);
                xTarget=mean(xMates);
                yTarget=mean(yMates);
                zTarget=mean(zMates);
            case 2 
                %Move toward the closest mate not part of the connected
                %set, defined as target destination
                matesNotInTheSet=(dMatrix(i,groupMates)>Dc);
                candidateTargets=groupMatesIDs(matesNotInTheSet);
                [dTarget, closestMate]=min(dMatrix(i,candidateTargets));
                xTarget=x(candidateTargets(closestMate));
                yTarget=y(candidateTargets(closestMate));
                zTarget=z(candidateTargets(closestMate));
        end
        %Set speed and direction based on the target destination
        vOut(i)=v_max;
        thetaOut(i)=atan2(yTarget-y(i),xTarget-x(i));
        phiOut(i)=asin((zTarget-z(i))/dTarget);
        
        %         if(abs(thetaOut(i))<pi/2)
        %             xNode=x(i):0.1:50;
        %         else
        %             xNode=0:0.1:x(i);
        %         end
        %         %xNode=x_min:0.1:x_max;
        %         yi=tan(thetaOut(i)).*(xNode-x(i))+y(i);
        %         figure()
        %         plot(xNode,yi,'--r');
        %         hold on
        %         scatter(x(i),y(i),30,'r','filled')
        %         scatter(x(closestMate),y(closestMate),30,'b','filled')
    end
end
