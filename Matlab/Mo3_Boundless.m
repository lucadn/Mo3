function [v,theta,phi]=Mo3_Boundless(v,theta,phi,v_min,v_max, a_max, g_max,d_max,tridimMobility,T)
%Function implementing the Individual Mobility module of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access




%Determine the new speed and check if it is within the allowed range [v_min,
%v_max]
DeltaV=-a_max*T+2*a_max*T*rand(1,length(v));
v=v+DeltaV;
v(find(v<v_min))=v_min;
v(find(v>v_max))=v_max;

%Determine the new direction and check if it is within the allowed range [-pi,
%pi]
DeltaTheta=-g_max*T+2*g_max*T*rand(1,length(theta));
theta=theta+DeltaTheta;
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
%If 3D mobility is on, determine the new elevation and check if it is within the allowed range [-pi/2,
%pi/2]
if(tridimMobility)
    DeltaPhi=-d_max*T+2*d_max*T*rand(1,length(phi));
    phi=phi+DeltaPhi;
    philow=find(phi<-pi/2);
    phi(philow)=-pi/2;
    phihigh=find(phi>pi/2);
    phi(phihigh)=pi/2;
end


