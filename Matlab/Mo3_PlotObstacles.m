function [] = Mo3_PlotObstacles(ObsList)
%Function plotting the obstacles as defined for use by the Obstacle Avoidance module of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

numObstacles=size(ObsList,1);
for j=1:numObstacles
    xobs0=ObsList(j,2);
    yobs0=ObsList(j,3);
    a=ObsList(j,4);
    b=ObsList(j,5);
    switch(ObsList(j,1))
        case 1
             %rectangle('Position',[xobs0-a/2,yobs0-b/2, a,b],'EdgeColor',[0.6350 0.0780 0.1840],'FaceColor',[0.6350 0.0780 0.1840]);
             rectangle('Position',[xobs0-a/2,yobs0-b/2, a,b],'EdgeColor',[0.6350 0.0780 0.1840]);
        case 2
            t=-pi:0.01:pi;
            x=xobs0+a*cos(t);
            y=yobs0+b*sin(t);
            plot(x,y,'k-')
    end
    
end

