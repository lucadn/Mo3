function [thetamin,thetamax]=Mo3_EllipseForbiddenRange(x0,y0,xobs0, yobs0,a,b)
%Function determining the forbidden range for a ellipsoidal obstacle as part of the Obstacle Avoidance module
%of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

thetamin=zeros(1,length(x0));
thetamax=zeros(1,length(x0));
c0=((a^2)*(y0*yobs0)-(a^2)*(yobs0^2)+b^2*xobs0*x0-b^2*xobs0^2+a^2*b^2)./((a^2)*(y0-yobs0));
c1=-(b^2*(x0-xobs0))./(a^2*(y0-yobs0));
c2=a^2;
c3=-2*a^2*yobs0;
c4=b^2;
c5=-2*b^2*(xobs0);
c6=a^2*yobs0^2+b^2*xobs0^2-(a^2)*(b^2);

aCoeff=c2.*(c1.^2)+c4;
bCoeff=2.*c2.*c1.*c0+c3.*c1+c5;
cCoeff=c2.*(c0.^2)+c3.*c0+c6;
Delta=bCoeff.^2-4.*aCoeff.*cCoeff;
realSolutionIndexes=Delta>0;
x_sol_1=(-bCoeff+sqrt(Delta))./(2*aCoeff);
x_sol_2=(-bCoeff-sqrt(Delta))./(2*aCoeff);
y_sol_1=c1.*x_sol_1+c0;
y_sol_2=c1.*x_sol_2+c0;

thetamax(realSolutionIndexes)=atan2(y_sol_1(realSolutionIndexes)-y0(realSolutionIndexes),x_sol_1(realSolutionIndexes)-x0(realSolutionIndexes));
thetamin(realSolutionIndexes)=atan2(y_sol_2(realSolutionIndexes)-y0(realSolutionIndexes),x_sol_2(realSolutionIndexes)-x0(realSolutionIndexes));
end
