function [vOut, thetaOut] = Mo3_UpperBoundsEnforcement(vIn,thetaIn,v0,theta0,v_max, v_min,a_max, g_max,dt)
%Function implementing the Upper Bounds Enforcement module part of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility Model for
%future generation mobile wireless networks", submitted to IEEE Access

% The Upper Bounds Enforcement module TBD.

maxDeltav=a_max*dt;
if maxDeltav>=(v_max-v_min)
    vOut=vIn;
else
    Deltav=vIn-v0;
    DeltavSign=sign(Deltav);
    DeltavViolations=find(abs(Deltav)>a_max*dt);
    Deltav(DeltavViolations)=DeltavSign(DeltavViolations)*a_max*dt;
    vOut=v0+Deltav;
end

maxDeltatheta=g_max*dt*0.999;
maxDeltathetaCheck=g_max*dt;
if (maxDeltatheta>=pi)
    thetaOut=thetaIn;
else
    thetaOut=thetaIn;
    Deltatheta=abs(thetaIn-theta0);
    beyondPiDeltas=find(Deltatheta>pi);
    Deltatheta(beyondPiDeltas)=2*pi-Deltatheta(beyondPiDeltas);
    DeltathetaViolations=find(Deltatheta>maxDeltatheta);
    
    for i=1:length(DeltathetaViolations)
        if (thetaIn(DeltathetaViolations(i))>theta0(DeltathetaViolations(i)))
            if ((thetaIn(DeltathetaViolations(i))>0) && (theta0(DeltathetaViolations(i))>=thetaIn(DeltathetaViolations(i))-pi))||(thetaIn(DeltathetaViolations(i))<=0)
                thetaOut(DeltathetaViolations(i))=theta0(DeltathetaViolations(i))+maxDeltatheta;
            else
                thetaOut(DeltathetaViolations(i))=theta0(DeltathetaViolations(i))-maxDeltatheta;
                while thetaOut(DeltathetaViolations(i))<-pi
                    thetaOut(DeltathetaViolations(i))=thetaOut(DeltathetaViolations(i))+2*pi;
                end
            end
            
        else
            if ((theta0(DeltathetaViolations(i))>0) && (thetaIn(DeltathetaViolations(i))>=theta0(DeltathetaViolations(i))-pi))||(theta0(DeltathetaViolations(i))<=0)
                thetaOut(DeltathetaViolations(i))=theta0(DeltathetaViolations(i))-maxDeltatheta;
            else
                thetaOut(DeltathetaViolations(i))=theta0(DeltathetaViolations(i))+maxDeltatheta;
                while thetaOut(DeltathetaViolations(i))>pi
                    thetaOut(DeltathetaViolations(i))=thetaOut(DeltathetaViolations(i))-2*pi;
                end
            end
        end
    end
    DeltathetaCheck=abs(thetaOut-theta0);
    beyondPiDeltasCheck=find(DeltathetaCheck>pi);
    DeltathetaCheck(beyondPiDeltasCheck)=2*pi-DeltathetaCheck(beyondPiDeltasCheck);
    DeltathetaViolationsCheck=find(DeltathetaCheck>maxDeltathetaCheck, 1);
    if(~isempty(DeltathetaViolationsCheck))
        fprintf('UB error\n');
    end
end

