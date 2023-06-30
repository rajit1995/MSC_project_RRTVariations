load("FieldConst.mat")
Dimensions.Length = Length;
Dimensions.Width = Width;
Obstacles.Number = numObstacles;
Obstacles.X = obstacleX;
Obstacles.Y = obstacleY;
Obstacles.Centers = obstacleCenters;
RRTState = rrtstateinit(pointA,pointB,Dimensions,Obstacles);
RRTState = APFRRT(RRTState);






