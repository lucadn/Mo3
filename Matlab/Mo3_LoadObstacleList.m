function ObsList=Mo3_LoadObstacleList(OLfilename)
%Function loading the obstacle list used by the Obstacle Avoidance module of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

fileID=fopen(OLfilename,'r');
fLine=fgets(fileID);
numObjects=0;
while (fLine~=-1)
    if(fLine(1)~='%')%Skip comments
        [type, ~, ~, nIndex]=sscanf(fLine,'%s');
        if strcmp(type(1:8),'Obstacle')
            numObjects=numObjects+1;
            ObsList(numObjects,:)=sscanf(fLine(9:end),'%d %f %f %f %f');
        end
    end
    fLine=fgets(fileID);
end
fclose(fileID);
end

