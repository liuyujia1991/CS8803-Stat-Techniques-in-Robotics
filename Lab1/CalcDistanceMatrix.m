Mapdata = importdata('data\map\wean.dat');

allLoc = Mapdata>=0;
allLoc = allLoc(:);
[allX, allY] = ind2sub(size(Mapdata),find(allLoc));

PossibleWall = Mapdata<0.05 & Mapdata>-0.1;
PossibleWall = PossibleWall(:);
[wallX, wallY] = ind2sub(size(Mapdata),find(PossibleWall));

DistanceMatrix = zeros(size(Mapdata));
tic
for i = 1:size(Mapdata,1)
    for j = 1:size(Mapdata,2)
        dist = ((wallX - i).^2+(wallY-j).^2).^0.5;
        DistanceMatrix(i,j) = min(dist)/10;
    end
end
toc
save('DistanceMatrix','DistanceMatrix')