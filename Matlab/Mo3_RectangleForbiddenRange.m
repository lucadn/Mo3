function [thetamin,thetamax]=Mo3_RectangleForbiddenRange(x0,y0,xobs0, yobs0,a,b)
%Function determining the forbidden range for a rectangular obstacle as part of the Obstacle Avoidance module
%of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

[xmin, xmax, ymin, ymax]=Mo3_FindRectangleCorners(xobs0, yobs0,a,b);
% d1=sqrt((x0-xmin)^2+(y0-ymin)^2);
% d2=sqrt((x0-xmin)^2+(y0-ymax)^2);
% d3=sqrt((x0-xmin)^2+(y0-ymax)^2);
% d4=sqrt((x0-xmin)^2+(y0-ymax)^2);
% d=[d1 d2 d3 d4];
thetamin=zeros(1,length(x0));
thetamax=zeros(1,length(x0));
%
%
BottomLeftNodes=(x0<=xmin) & (y0<=ymin);
LeftNodes=(x0<=xmin) &  (y0>ymin) &  (y0<=ymax);
TopLeftNodes=(x0<=xmin) &  (y0>ymax);
%if (x0<=xmin)
%   if (y0<=ymin)%Bottomleft
%fprintf('Bottomleft nodes: %d\n',BottomLeftNodes);
thetamin(BottomLeftNodes)=atan2(ymin-y0(BottomLeftNodes),xmax-x0(BottomLeftNodes));
thetamax(BottomLeftNodes)=atan2(ymax-y0(BottomLeftNodes),xmin-x0(BottomLeftNodes));
%end
%if (y0>ymin)&&(y0<=ymax)%left
%fprintf('Left nodes: %d\n',LeftNodes);
thetamin(LeftNodes)=atan2(ymin-y0(LeftNodes),xmin-x0(LeftNodes));
thetamax(LeftNodes)=atan2(ymax-y0(LeftNodes),xmin-x0(LeftNodes));
%end
%if (y0>ymax)%topleft
%fprintf('Topleft nodes: %d\n',TopLeftNodes);
thetamin(TopLeftNodes)=atan2(ymin-y0(TopLeftNodes),xmin-x0(TopLeftNodes));
thetamax(TopLeftNodes)=atan2(ymax-y0(TopLeftNodes),xmax-x0(TopLeftNodes));
%end
%end
BottomNodes=(x0>xmin) & (x0<=xmax) &  (y0<=ymin);
%if (x0>xmin)&&(x0<=xmax)
%if (y0<=ymin)%Bottom
%fprintf('Bottom nodes: %d\n',BottomNodes);
thetamin(BottomNodes)=atan2(ymin-y0(BottomNodes),xmax-x0(BottomNodes));
thetamax(BottomNodes)=atan2(ymin-y0(BottomNodes),xmin-x0(BottomNodes));
%end
TopNodes=(x0>xmin) &  (x0<=xmax) &  (y0>ymax);
%if (y0>ymax)%top
%fprintf('Top nodes: %d\n',TopNodes);
thetamin(TopNodes)=atan2(ymax-y0(TopNodes),xmin-x0(TopNodes));
thetamax(TopNodes)=atan2(ymax-y0(TopNodes),xmax-x0(TopNodes));
%end
%end
BottomRightNodes=(x0>xmax) &  (y0<=ymin);
%if (x0>xmax)
%   if (y0<=ymin)%Bottomright
%fprintf('Bottomright nodes: %d\n',BottomRightNodes);
thetamin(BottomRightNodes)=atan2(ymax-y0(BottomRightNodes),xmax-x0(BottomRightNodes));
thetamax(BottomRightNodes)=atan2(ymin-y0(BottomRightNodes),xmin-x0(BottomRightNodes));
%   end
RightNodes=(x0>xmax) &  (y0>ymin) & (y0<=ymax);
%  if (y0>ymin)&&(y0<=ymax)%right
%fprintf('Right nodes: %d\n',RightNodes);
thetamin(RightNodes)=atan2(ymin-y0(RightNodes),xmax-x0(RightNodes));
thetamax(RightNodes)=atan2(ymax-y0(RightNodes),xmax-x0(RightNodes));
% end
TopRightNodes=(x0>xmax) &  (y0>ymax);
%if (y0>ymax)%topright
%fprintf('Topright nodes: %d\n',TopRightNodes);
thetamin(TopRightNodes)=atan2(ymax-y0(TopRightNodes),xmin-x0(TopRightNodes));
thetamax(TopRightNodes)=atan2(ymin-y0(TopRightNodes),xmax-x0(TopRightNodes));
%end

% if(size(thetamin,2)<10)
%     fprintf('Error')
%     AllNodesCheck=BottomLeftNodes|LeftNodes|TopLeftNodes|BottomNodes|TopNodes|BottomRightNodes|RightNodes|TopRightNodes;
%     pause
% end
%size(thetamax)
end

