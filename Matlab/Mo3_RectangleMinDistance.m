function [detected]=Mo3_RectangleMinDistance(x0,y0,xobs0, yobs0,a,b,d_OA_trigger)
%Function determining the minimum distance from a rectangular obstacle as part of the Obstacle Avoidance module
%of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

[xmin, xmax, ymin, ymax]=Mo3_FindRectangleCorners(xobs0, yobs0,a,b);
% d1=sqrt((x0-xmin)^2+(y0-ymin)^2);
% d2=sqrt((x0-xmin)^2+(y0-ymax)^2);
% d3=sqrt((x0-xmin)^2+(y0-ymax)^2);
% d4=sqrt((x0-xmin)^2+(y0-ymax)^2);
% d=[d1 d2 d3 d4];
%
%
minDistance = Inf*ones(1,length(x0));

BottomLeftNodes=(x0<=xmin) & (y0<=ymin);
minDistance(BottomLeftNodes)=sqrt((x0(BottomLeftNodes)-xmin).^2+(y0(BottomLeftNodes)-ymin).^2);

LeftNodes=(x0<=xmin) &  (y0>ymin) &  (y0<=ymax);
minDistance(LeftNodes)=sqrt((x0(LeftNodes)-xmin).^2);

TopLeftNodes=(x0<=xmin) &  (y0>ymax);
minDistance(TopLeftNodes)=sqrt((x0(TopLeftNodes)-xmin).^2+(y0(TopLeftNodes)-ymax).^2);

BottomNodes=(x0>xmin) & (x0<=xmax) &  (y0<=ymin);
minDistance(BottomNodes)=sqrt((y0(BottomNodes)-ymin).^2);

TopNodes=(x0>xmin) &  (x0<=xmax) &  (y0>ymax);
minDistance(TopNodes)=sqrt((y0(TopNodes)-ymax).^2);

BottomRightNodes=(x0>xmax) &  (y0<=ymin);
minDistance(BottomRightNodes)=sqrt((x0(BottomRightNodes)-xmax).^2+(y0(BottomRightNodes)-ymin).^2);

RightNodes=(x0>xmax) &  (y0>ymin) & (y0<=ymax);
minDistance(RightNodes)=sqrt((x0(RightNodes)-xmax).^2);

TopRightNodes=(x0>xmax) &  (y0>ymax);
minDistance(TopRightNodes)=sqrt((x0(TopRightNodes)-xmax).^2+(y0(TopRightNodes)-ymax).^2);

detected = minDistance<=d_OA_trigger;

end

