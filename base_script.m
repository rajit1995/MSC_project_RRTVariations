clc;
rrtvariables;
load("FieldConst.mat");
Dimensions.Length = Length;
Dimensions.Width = Width;
Obstacles.Number = numObstacles;
Obstacles.X = obstacleX;
Obstacles.Y = obstacleY;
Obstacles.Centers = obstacleCenters;

for i=1:10 
RRTState = rrtstateinit(pointA,pointB,Dimensions,Obstacles);
tic;
RRTState = APFRRT(RRTState);

elapsedTime = toc;
Dist(i) = RRTState.Final.dist_total;
Iterations(i) = RRTState.Final.Iterations;
time(i) = elapsedTime;
disp(['Iteration Completed:',num2str(i)])
disp(['Iterations :',num2str(RRTState.Final.Iterations)])
disp(['Distance Travelled :',num2str(RRTState.Final.dist_total)])
disp(['Time Taken :',num2str(elapsedTime)])
disp(' ') 

end
Avg_distance = mean(Dist);
Avg_iterations = mean(Iterations);
Avg_time = mean(time);
%%
disp(['Average Iterations over 10 runs:',num2str(Avg_iterations)])
disp(['Average Distance Travelled over 10 runs:',num2str(Avg_distance)])
disp(['Average time taken over 10 runs:',num2str(Avg_time)])










