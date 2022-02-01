function [xmin, xmax, ymin, ymax]=Mo3_FindRectangleCorners(xobs0, yobs0,a,b)
%Function determining the corners of a rectangular obstacle as part of the Obstacle Avoidance module
%of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

xmin=xobs0-a/2;
xmax=xobs0+a/2;
ymin=yobs0-b/2;
ymax=yobs0+b/2;