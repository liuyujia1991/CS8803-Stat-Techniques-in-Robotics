clear all
close all
clc

%Parameters
zhit = 0.75;
zrandom = 0.20;
zmax = 0.05;
SigmaHit = 2;
NofParticles = 10000;

VideoCapture = false;

%Read map data
Mapdata = importdata('data/map/wean.dat');

%Calculate the distance matrix
%If cannot find DistanceMatrix file, Remove the comment below
%CalcDistanceMatrix
matrixFile = load('DistanceMatrix');
DistanceMatrix = matrixFile.DistanceMatrix;

%Start video Capture
if VideoCapture
    writerObj = VideoWriter('VideoCapture.avi'); 
    open(writerObj); 
end

%Read data log and convert to meters
Logfile = importdata('data/log/robotdata1_HJ.log');
LogData= Logfile.data;
LogData(:,1) = LogData(:,1)/100;
LogData(:,2) = LogData(:,2)/100;
LogLength = size(LogData,1);

indexLargest = 1;
MapLength = size(Mapdata,1);
MapWidth = size(Mapdata,2);
angles = [-pi/2:pi/179:pi/2];

%Random initialize particles.
PossibleLoc = Mapdata > 0.95;
PossibleLoc = PossibleLoc(:);
PossibleIndex = randsample(size(PossibleLoc,1),NofParticles,true,PossibleLoc);
ParticleX = mod(PossibleIndex,MapWidth)/10;
ParticleY = ceil(PossibleIndex/MapWidth)/10;
ParticleTheta = 2*pi*rand(NofParticles,1)-pi;

%Plot everything for the first time
imshow(Mapdata)
hold on 
mypoints = plot(ParticleY*10,ParticleX*10,'.');
bestpoint = plot(ParticleY(indexLargest)*10,ParticleX(indexLargest)*10,'r*');
LargestYloc = 1;
LargestXloc = 1;
Laserdata = fill(LargestYloc*10,LargestXloc*10,'r');
TextNumberParticles = text(10,20,strcat('Number of Particles: ',num2str(NofParticles)),'color','white');

%Loop through the log file
for lineindex = 2:LogLength;
    
    likelihood = ones(NofParticles,1);
    %Obtain the relative odometry
    PrevReading = LogData(lineindex-1,:);
    CurrentReading = LogData(lineindex,:);
    dX = CurrentReading(1) - PrevReading(1);
    dY = CurrentReading(2) - PrevReading(2);
    dTheta = CurrentReading(3) - PrevReading(3);
    
    %Calculate the relative motion
    MovDistance = (dX^2+dY^2)^0.5;
    MovDirection = ParticleTheta - PrevReading(3) + atan2(dY,dX);
    Xchange = MovDistance*cos(MovDirection);
    Ychange = MovDistance*sin(MovDirection);
    
    %Motion Model
    ParticleX = ParticleX+Xchange.*(1+randn(NofParticles,1)/20)+randn(NofParticles,1)/50;
    ParticleY = ParticleY+Ychange.*(1+randn(NofParticles,1)/20)+randn(NofParticles,1)/50;
    ParticleTheta = ParticleTheta + dTheta.*(1+randn(NofParticles,1)/20)+randn(NofParticles,1)/50;

    delete(mypoints);
    mypoints = plot(ParticleY*10,ParticleX*10,'.');
    
    
    %If current reading is a laser data%
    if ~isnan(CurrentReading(5))
        
        %Calculate the sensor positions for each particle
        SensorPosX = ParticleX + 0.25*cos(ParticleTheta);
        SensorPosY = ParticleY + 0.25*sin(ParticleTheta);
        
        LaserReading = CurrentReading(7:end-1)/100;
        ReasonReading = LaserReading<80;

        for i = 1:NofParticles
            
            %Obtain the reading locations
            Xloc = SensorPosX(i) + LaserReading.*cos(ParticleTheta(i)+angles);
            Yloc = SensorPosY(i) + LaserReading.*sin(ParticleTheta(i)+angles);
            
            
            tempY = round(ParticleY(i)*10);
            tempX = round(ParticleX(i)*10);
            %Remove particle if it goes out of bound or hit wall
            if tempY <= 0 | tempX <= 0 | tempY >800 | tempX > 800 | Mapdata(tempX,tempY) <0.95
                likelihood(i) = 0;
            else
                
                %Clip the indexes to the matrix size
                Xmapindex = round(Xloc*10);
                Xmapindex(Xmapindex<1) = 1;
                Xmapindex(Xmapindex>MapWidth) = MapWidth;
                Ymapindex = round(Yloc*10);
                Ymapindex(Ymapindex<1) = 1;
                Ymapindex(Ymapindex>MapLength) = MapLength;
                
                %Obtain the distance reading from the distance matrix
                distanceReadings = DistanceMatrix(sub2ind(size(DistanceMatrix),Xmapindex,Ymapindex));
                
                %Calculate the likelihood
                Prob = normpdf(distanceReadings,0,SigmaHit);
                IndividLikelihood = (zhit*Prob+zrandom/zmax).*ReasonReading;
                likelihood(i) = prod(nonzeros(IndividLikelihood));
            end
        end
        
        %Normalize the weight
        Pweight = likelihood/sum(likelihood);
        [~,indexLargest] = max(Pweight);
        
        %Reduce the number of particles
        Oldcount = NofParticles;
        NofParticles = round(NofParticles*0.99);
        NofParticles = max(NofParticles,1000);
        
        %Plot the particle with the largest weight and its laser readings
        LargestSensorPosX = ParticleX(indexLargest) + 0.25*cos(ParticleTheta(indexLargest));
        LargestSensorPosY = ParticleY(indexLargest) + 0.25*sin(ParticleTheta(indexLargest));
        LargestXloc = LargestSensorPosX + LaserReading.*cos(ParticleTheta(indexLargest)+angles);
        LargestYloc = LargestSensorPosY + LaserReading.*sin(ParticleTheta(indexLargest)+angles);
        delete(Laserdata);
        Laserdata = fill(LargestYloc*10,LargestXloc*10,'r');
        
        delete(bestpoint);
        bestpoint = plot(ParticleY(indexLargest)*10,ParticleX(indexLargest)*10,'g*');
    
        %Resample based on the weight    
        newParticleIndex = randsample(Oldcount,NofParticles,true,Pweight);
        ParticleX= ParticleX(newParticleIndex);
        ParticleY= ParticleY(newParticleIndex);
        ParticleTheta= ParticleTheta(newParticleIndex);
        
        %Plot the current number of particles
        delete(TextNumberParticles)
        TextNumberParticles = text(10,20,strcat('Number of Particles: ',num2str(NofParticles)),'color','white');
        
    end
 
    if VideoCapture
        %Save the frame
        frame = getframe(gcf); 
        writeVideo(writerObj, frame);
    end      
        
    fprintf(strcat('Current log file index: ',num2str(lineindex),'\n')) 
    pause(.002)
end

if VideoCapture
    close(writerObj); 
end

