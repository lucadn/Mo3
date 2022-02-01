function [vOut, thetaOut,groupingConditionVector] = Mo3_CorrelatedMobility(M, x,y,dMatrix, v_max, vIn, thetaIn, BM, Dc,rho_min,groupingStrategy)
%Function implementing the Correlated Mobility module of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

vOut=vIn;
thetaOut=thetaIn;
groupingConditionVector=false(1,M);

for i=1:M
    %Step 1 - Determine bindings according to the current binding matrix
    groupMates=logical(BM(i,:));
    groupMatesIDs=find(groupMates);
    groupSize=length(groupMatesIDs);
    %Step 2 - Check binding conditions and determine the connected set
    connectedSet=(dMatrix(i,groupMates)<=Dc);
    connectedSetSize=nnz(connectedSet);
    %Step 3 - Check grouping condition
    rho=connectedSetSize/(groupSize-1);
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
                xTarget=mean(xMates);
                yTarget=mean(yMates);
            case 2 
                %Move toward the closest mate not part of the connected
                %set, defined as target destination
                matesNotInTheSet=(dMatrix(i,groupMates)>Dc);
                candidateTargets=groupMatesIDs(matesNotInTheSet);
                [~, closestMate]=min(dMatrix(i,candidateTargets));
                xTarget=x(candidateTargets(closestMate));
                yTarget=y(candidateTargets(closestMate));
        end
        %Set speed and direction based on the target destination
        vOut(i)=v_max;
        thetaOut(i)=atan2(yTarget-y(i),xTarget-x(i));
        
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
