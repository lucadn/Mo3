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
%Ray ellipse intersection
%(x-xobs0)^2/a^2+(y-obs0)^2/b^2=1
%x=x0+u*cos(theta)
%y=y0+u*sin(theta)
%(x0+u*cos(theta)-xobs0)^2/a^2+(y0+u*sin(theta)-yobs0)^2/b^2=1
%b^2*(x0+u*cos(theta)-xobs0)^2+a^2*(y0+u*sin(theta)-yobs0)^2=a^2*b^2
%b^2*((x0-xobs0)^2+u^2*(cos(theta)^2)+2*(x0-xobs0)*u*cos(theta))+a^2*((y0-yobs0)^2+u^2*(sin(theta)^2)+2*(y0-yobs0)*u*sin(theta))=a^2*b^2
%u^2*(b^2*(cos(theta)^2)+a^2*(sin(theta)^2))+u*(b^2*2*(x0-xobs0)*cos(theta)+a^2*2*(y0-yobs0)*sin(theta))+b^2*(x0-xobs0)^2+a^2*(y0-yobs0)-a^2*b^2=0
Delta1=(b^2*2*(x0-xobs0).*cos(thetaCenter)+a^2*2*(y0-yobs0).*sin(thetaCenter)).^2-4*(b^2*(cos(thetaCenter).^2)+a^2*(sin(thetaCenter).^2)).*(b^2*(x0-xobs0).^2+a^2*(y0-yobs0).^2-a^2*b^2);
u11=(-(b^2*2*(x0-xobs0).*cos(thetaCenter)+a^2*2*(y0-yobs0).*sin(thetaCenter))+sqrt(Delta1))./(2*(b^2*(cos(thetaCenter).^2)+a^2*(sin(thetaCenter).^2)));
u12=(-(b^2*2*(x0-xobs0).*cos(thetaCenter)+a^2*2*(y0-yobs0).*sin(thetaCenter))-sqrt(Delta1))./(2*(b^2*(cos(thetaCenter).^2)+a^2*(sin(thetaCenter).^2)));

%fprintf('Collision on node 1\n')
xC11=x0+u11.*cos(thetaCenter);
yC11=y0+u11.*sin(thetaCenter);
%fprintf('Collision on node 1\n')
xC12=x0+u12.*cos(thetaCenter);
yC12=y0+u12.*sin(thetaCenter);
%fprintf('No collision on node: IMPOSSIBLE\n')

dist1=sqrt((x0-xC11).^2+(y0-yC11).^2);
dist2=sqrt((x0-xC12).^2+(y0-yC12).^2);
dist1MinNodes=dist1<dist2;
dist2MinNodes=dist1>=dist2;
minDistance(dist1MinNodes)=dist1(dist1MinNodes);
minDistance(dist2MinNodes)=dist2(dist2MinNodes);
detected = minDistance<=d_OA_trigger;

end

