function [x,y]=Mo3_ForceOutsideObstacles(x,y,x_min,x_max,y_min,y_max, ObsList)
%Function checking if node positions fall within obstacles defined
%according to the Obstacle Avoidance module of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access
numObstacles=size(ObsList,1);
numNodes=size(x,2);
nodesInside=true(numObstacles,numNodes);
%Keep randomly generating the positions of nodes within an obstacle until all of them fall outside. Note
%that out of simplicity each ellipse is approximated by the smallest rectangle
%containing it.
while (any(nodesInside,'all'))
    for j=1:numObstacles
        switch ObsList(j,1)
            case 1
                [xminObs, xmaxObs, yminObs, ymaxObs]=Mo3_FindRectangleCorners(ObsList(j,2), ObsList(j,3),ObsList(j,4),ObsList(j,5));
            case 2
                [xminObs, xmaxObs, yminObs, ymaxObs]=Mo3_FindRectangleCorners(ObsList(j,2), ObsList(j,3),2*ObsList(j,4),2*ObsList(j,5));
        end
        nodesInside(j,:) = (x>=xminObs) & (x<=xmaxObs) & (y>=yminObs) & (y<=ymaxObs);
        fprintf('%d nodes inside obstacle %d\n',nnz(nodesInside(j,:)),j)
        x(nodesInside(j,:)) = x_min+(x_max-x_min)*rand(1,nnz(nodesInside(j,:)));
        y(nodesInside(j,:)) = y_min+(y_max-y_min)*rand(1,nnz(nodesInside(j,:)));
    end
    
end

