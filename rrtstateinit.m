function RRTState = rrtstateinit(pointA,pointB,Dimensions,Obstacles)
    RRTState.PointA = pointA;
    RRTState.PointB = pointB;
    
    RRTState.Dimensions = Dimensions;
    RRTState.Obstacles = Obstacles;
    
    RRTState.q_near = [-1,-1];
    RRTState.q_new = [-1,-1];
    RRTState.iteration.count = 1;
    RRTState.iteration.max = 1000;
    RRTState.Terraincolour = [0.800000000000000,0.600000000000000,0.400000000000000];
    RRTState.pathvertices(1,:) = pointA;
    RRTState.Threshold = 1;
    RRTState.StepSize =1;
    RRTState.GoalReachInd =0;
    for i = 1:RRTState.Obstacles.Number
       for j= 1:size(RRTState.Obstacles.X(i,:),2)
           radius(i,j) = norm([RRTState.Obstacles.X(i,j),RRTState.Obstacles.Y(i,j)] - RRTState.Obstacles.Centers(i));
       end
       RRTState.Obstacles.radius_max(i) = max(radius(i,:))
    end
end
