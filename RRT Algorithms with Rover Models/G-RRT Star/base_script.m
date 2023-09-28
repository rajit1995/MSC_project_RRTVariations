clc;
clearvars;
rrtvariables;
load("FieldConst.mat");
Dimensions.Length = Length;
Dimensions.Width = Width;
Obstacles.Number = numObstacles;
Obstacles.X = obstacleX;
Obstacles.Y = obstacleY;
Obstacles.Centers = obstacleCenters;
Dist = zeros(1,iternum);
Iterations = zeros(1,iternum);
time1 = zeros(1,iternum);
time2 = zeros(1,iternum);

for i=1:iternum 
RRTState = rrtstateinit(pointA,pointB,Dimensions,Obstacles);
tic;
RRTState = GRRTSTAR1(RRTState);
% RRTState = getpath(RRTState);
elapsedTime = toc;
filename = ['G_RRT_STAR' num2str(i) '.jpg'];
saveas(1,filename)
Dist(i) = RRTState.Final.dist_total;
Iterations(i) = RRTState.Final.Iterations;
time1(i) = elapsedTime;
disp(['Iteration Completed:',num2str(i)])
disp(['Iterations :',num2str(RRTState.Final.Iterations)])
disp(['Distance Travelled :',num2str(RRTState.Final.dist_total)])
disp(['Time Taken :',num2str(elapsedTime)])
disp(' ') 
elapsedTime = toc;
Rover = roverinit(RRTState);
Rover = rovermodel(Rover);
time2(i) = elapsedTime;
end
Avg_distance = mean(Dist);
Avg_iterations = mean(Iterations);
Avg_time1 = mean(time1);
Avg_time2 = mean(time2);

%%
disp(['Average Iterations per run:',num2str(Avg_iterations)])
disp(['Average Distance of the Path:',num2str(Avg_distance)])
disp(['Average time taken :',num2str(Avg_time1)])
disp(['Time taken by the Rover:',num2str(Avg_time2)])










