function [v,theta]=Mo3_Boundless(v,theta,v_min,v_max, a_max, g_max, T)
%Function implementing the Individual Mobility module of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

DeltaV=-a_max*T+2*a_max*T*rand(1,length(v));
DeltaTheta=-g_max*T+2*g_max*T*rand(1,length(theta));

v=v+DeltaV;
v(find(v<v_min))=v_min;
v(find(v>v_max))=v_max;
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

% while(theta<-pi)
%     theta=theta+2*pi;
% end
% while(theta>pi)
%     theta=theta-2*pi;
% end
