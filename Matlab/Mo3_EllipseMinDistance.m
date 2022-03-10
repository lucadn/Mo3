function [detected]=Mo3_EllipseMinDistance(x0,y0,xobs0, yobs0,a,b,d_OA_trigger)
%Function determining the minimum distance from a ellipsoidal obstacle as part of the Obstacle Avoidance module
%of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

%[xmin, xmax, ymin, ymax]=Mo3_FindRectangleCorners(xobs0, yobs0,a,b);
% d1=sqrt((x0-xmin)^2+(y0-ymin)^2);
% d2=sqrt((x0-xmin)^2+(y0-ymax)^2);
% d3=sqrt((x0-xmin)^2+(y0-ymax)^2);
% d4=sqrt((x0-xmin)^2+(y0-ymax)^2);
% d=[d1 d2 d3 d4];
%
%
minDistance = Inf*ones(1,length(x0));
thetaCenter=atan2(yobs0-y0,xobs0-x0);

%Intersection between ellipse and ray starting in the node position and passing through to the center
%of the ellipse.
Delta1=(b^2*2*(x0-xobs0).*cos(thetaCenter)+a^2*2*(y0-yobs0).*sin(thetaCenter)).^2-4*(b^2*(cos(thetaCenter).^2)+a^2*(sin(thetaCenter).^2)).*(b^2*(x0-xobs0).^2+a^2*(y0-yobs0).^2-a^2*b^2);
u11=(-(b^2*2*(x0-xobs0).*cos(thetaCenter)+a^2*2*(y0-yobs0).*sin(thetaCenter))+sqrt(Delta1))./(2*(b^2*(cos(thetaCenter).^2)+a^2*(sin(thetaCenter).^2)));
u12=(-(b^2*2*(x0-xobs0).*cos(thetaCenter)+a^2*2*(y0-yobs0).*sin(thetaCenter))-sqrt(Delta1))./(2*(b^2*(cos(thetaCenter).^2)+a^2*(sin(thetaCenter).^2)));

%Coordinates of first intersection point
xC11=x0+u11.*cos(thetaCenter);
yC11=y0+u11.*sin(thetaCenter);
%Coordinates of second intersection point
xC12=x0+u12.*cos(thetaCenter);
yC12=y0+u12.*sin(thetaCenter);

dist1=sqrt((x0-xC11).^2+(y0-yC11).^2);
dist2=sqrt((x0-xC12).^2+(y0-yC12).^2);
dist1MinNodes=dist1<dist2;
dist2MinNodes=dist1>=dist2;
minDistance(dist1MinNodes)=dist1(dist1MinNodes);
minDistance(dist2MinNodes)=dist2(dist2MinNodes);
detected = minDistance<=d_OA_trigger;

end

