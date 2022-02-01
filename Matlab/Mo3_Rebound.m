function [x, y, theta, xViolations, yViolations]=Mo3_Rebound(x, y, theta, minX,minY,maxX,maxY)
%Function implementing the perfect reflection rebound as part of the Individual Mobility module
%of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access


xViolationsMin=x < minX;
x(xViolationsMin)=minX;
theta(xViolationsMin)= pi-theta(xViolationsMin);

xViolationsMax=x > maxX;
x(xViolationsMax)=maxX;
theta(xViolationsMax)= pi-theta(xViolationsMax);


yViolationsMin=y < minY;
y(yViolationsMin)=minY;
theta(yViolationsMin)= -theta(yViolationsMin);

yViolationsMax=y > maxY;
y(yViolationsMax)=maxY;
theta(yViolationsMax)= -theta(yViolationsMax);

xViolations=xViolationsMin|xViolationsMax;
yViolations=yViolationsMin|yViolationsMax;

thetalow=find(theta<-pi);
while ~isempty(thetalow)
    theta(thetalow)=theta(thetalow)+2*pi;
    thetalow=find(theta<-pi);
end

thetahigh=find(theta>pi);
while ~isempty(thetahigh)
    theta(thetahigh)=theta(thetahigh)-2*pi;
    thetahigh=find(theta>pi);
end
% if( y < minY)
%     violation(2)=true;
%     y = minY;
%     theta = 6.28 - theta ;
% end
% if( y > maxY)
%     violation(2)=true;
%     y = maxY;
%     theta = 6.28 - theta;
% end