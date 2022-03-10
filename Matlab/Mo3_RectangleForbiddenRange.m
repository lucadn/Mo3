function [thetamin,thetamax]=Mo3_RectangleForbiddenRange(x0,y0,xobs0, yobs0,a,b)
%Function determining the forbidden range for a rectangular obstacle as part of the Obstacle Avoidance module
%of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

[xmin, xmax, ymin, ymax]=Mo3_FindRectangleCorners(xobs0, yobs0,a,b);

thetamin=zeros(1,length(x0));
thetamax=zeros(1,length(x0));

BottomLeftNodes=(x0<=xmin) & (y0<=ymin);
thetamin(BottomLeftNodes)=atan2(ymin-y0(BottomLeftNodes),xmax-x0(BottomLeftNodes));
thetamax(BottomLeftNodes)=atan2(ymax-y0(BottomLeftNodes),xmin-x0(BottomLeftNodes));

LeftNodes=(x0<=xmin) &  (y0>ymin) &  (y0<=ymax);
thetamin(LeftNodes)=atan2(ymin-y0(LeftNodes),xmin-x0(LeftNodes));
thetamax(LeftNodes)=atan2(ymax-y0(LeftNodes),xmin-x0(LeftNodes));

TopLeftNodes=(x0<=xmin) &  (y0>ymax);
thetamin(TopLeftNodes)=atan2(ymin-y0(TopLeftNodes),xmin-x0(TopLeftNodes));
thetamax(TopLeftNodes)=atan2(ymax-y0(TopLeftNodes),xmax-x0(TopLeftNodes));

BottomNodes=(x0>xmin) & (x0<=xmax) &  (y0<=ymin);
thetamin(BottomNodes)=atan2(ymin-y0(BottomNodes),xmax-x0(BottomNodes));
thetamax(BottomNodes)=atan2(ymin-y0(BottomNodes),xmin-x0(BottomNodes));

TopNodes=(x0>xmin) &  (x0<=xmax) &  (y0>ymax);
thetamin(TopNodes)=atan2(ymax-y0(TopNodes),xmin-x0(TopNodes));
thetamax(TopNodes)=atan2(ymax-y0(TopNodes),xmax-x0(TopNodes));

BottomRightNodes=(x0>xmax) &  (y0<=ymin);
thetamin(BottomRightNodes)=atan2(ymax-y0(BottomRightNodes),xmax-x0(BottomRightNodes));
thetamax(BottomRightNodes)=atan2(ymin-y0(BottomRightNodes),xmin-x0(BottomRightNodes));

RightNodes=(x0>xmax) &  (y0>ymin) & (y0<=ymax);
thetamin(RightNodes)=atan2(ymin-y0(RightNodes),xmax-x0(RightNodes));
thetamax(RightNodes)=atan2(ymax-y0(RightNodes),xmax-x0(RightNodes));

TopRightNodes=(x0>xmax) &  (y0>ymax);
thetamin(TopRightNodes)=atan2(ymax-y0(TopRightNodes),xmin-x0(TopRightNodes));
thetamax(TopRightNodes)=atan2(ymin-y0(TopRightNodes),xmax-x0(TopRightNodes));

end

