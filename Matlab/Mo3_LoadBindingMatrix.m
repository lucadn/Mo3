function [BM,nextSwitchTime,BMFilePos]=Mo3_LoadBindingMatrix(BMfilename,fPos,M)
%Function loading the Binding Matrix used by the Correlated Mobility module of the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access

fileID=fopen(BMfilename,'r');
fseek(fileID,fPos,'bof');
BM=eye(M);
nextSwitchTime=0;
fLine=fgets(fileID);
while (fLine(1)=='%')%Skip comments
    fLine=fgets(fileID);
end
[BM_line]=textscan(fLine,'%d',M);
BM(1,:)=cell2mat(BM_line(1,1));
for i=2:M
    [BM_line]=textscan(fileID,'%d',M);
    BM(i,:)=cell2mat(BM_line(1,1));
end
ftell(fileID)
fLine=fgets(fileID);%get rid of end of line character, because textscan doesn't...
fLine=fgets(fileID);
if (fLine==-1)
    nextSwitchTime=-1; %File over, this was the last BM to be loaded
    fprintf('File over, this was the last BM to be loaded\n');
else
    while (fLine~=-1)
        if(fLine(1)=='%')%Skip comments
            fLine=fgets(fileID);
            continue
        end
        [type, ~, ~, nIndex]=sscanf(fLine,'%s');
        if strcmp(type(1:10),'nextSwitch')
            nextSwitchTime=sscanf(fLine(11:end),'%f');
            break
        else
            fprintf('BM file formatting error\n');
        end
    end
end
BMFilePos=ftell(fileID);
fclose(fileID);
end

