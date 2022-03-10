function d = Mo3_EuclideanDistance(i,j,x,y,z)
%Function evaluating the euclidean distance between two nodes, part of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

d=sqrt((x(i)-x(j))^2+(y(i)-y(j))^2+(z(i)-z(j))^2);
end

