function [theta_t1,theta_t2,x_sol_1,y_sol_1,x_sol_2,y_sol_2] = Mo3_FindTangentDirections(x0,y0,xobs0,yobs0,a,b)
%Function determining the directions of the tangents to an ellipsoidal obstacle as part of the Obstacle Avoidance module
%of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

c0=((a^2)*(y0*yobs0)-(a^2)*(yobs0^2)+b^2*xobs0*x0-b^2*xobs0^2+a^2*b^2)/((a^2)*(y0-yobs0));
c1=-(b^2*(x0-xobs0))/(a^2*(y0-yobs0));
c2=a^2;
c3=-2*a^2*yobs0;
c4=b^2;
c5=-2*b^2*(xobs0);
c6=a^2*yobs0^2+b^2*xobs0^2-(a^2)*(b^2);

aCoeff=c2*c1^2+c4;
bCoeff=2*c2*c1*c0+c3*c1+c5;
cCoeff=c2*c0^2+c3*c0+c6;
Delta=bCoeff^2-4*aCoeff*cCoeff;
x_sol_1=(-bCoeff+sqrt(Delta))/(2*aCoeff);
x_sol_2=(-bCoeff-sqrt(Delta))/(2*aCoeff);
y_sol_1=c1*x_sol_1+c0;
y_sol_2=c1*x_sol_2+c0;

theta_t1=atan2(y_sol_1-y0,x_sol_1-x0);
theta_t2=atan2(y_sol_2-y0,x_sol_2-x0);
end

