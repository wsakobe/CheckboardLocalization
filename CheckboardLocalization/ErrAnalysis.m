WorldPoints = load('EndEffectorWorldPoints.txt');
for i=1:size(WorldPoints,1)/100
    averageWorldPoints(i,:)=mean(WorldPoints(1+(i-1)*100:i*100,:),1);
end
A=repmat(0:32:160,1,3);
B=[0*ones(1,6),32*ones(1,6),64*ones(1,6)];
C=ones(1,18);
realBoard = [A;B;C]';
[~, ~, R, T] = ransac_icp(averageWorldPoints', realBoard', 10, 8, 50, 0.6);
err = (R * realBoard' + repmat(T, [1 size(averageWorldPoints, 1)]) - averageWorldPoints').^2/size(averageWorldPoints, 1);
for i =1:size(averageWorldPoints,1)
    err1(i)=sqrt(err(1,i)^2+err(2,i)^2+err(3,i)^2);
end
mean(err1)