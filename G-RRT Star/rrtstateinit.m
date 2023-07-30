function RRTState = rrtstateinit(pointA,pointB,Dimensions,Obstacles)
    RRTState.PointA = pointA;
    RRTState.PointB = pointB;
    
    RRTState.Dimensions = Dimensions;
    RRTState.Obstacles = Obstacles;
    RRTState.q_near_count = 0;
    RRTState.int_ind = 0;
    RRTState.q_near = RRTState.PointA;
    RRTState.q_new = RRTState.PointA;
    RRTState.iteration.count = 1;
    RRTState.iteration.max = 1000;
    RRTState.R_new2goal = Inf;
    RRTState.rwradius=2;
    RRTState.plotfinalpath = 0;
    RRTState.Final.dist = [];
    RRTState.nearidx=1;
    RRTState.Terraincolour = [0.8,0.6,0.4];    
        RRTState.Branches1(1,:) = [1,1,0];
    RRTState.pathvertices(1,:) = [pointA,0];
    RRTState.StepSize =1;
    RRTState.Threshold = 1 ;
    %RRTState.neighborhoodRadius = 25;
   
    RRTState.GoalReachInd =0;
    for i = 1:RRTState.Obstacles.Number
       for j= 1:size(RRTState.Obstacles.X(i,:),2)
           RRTState.Obstacles.radius(i,j) = norm([RRTState.Obstacles.X(i,j),RRTState.Obstacles.Y(i,j)] - [RRTState.Obstacles.Centers(i,1) ,RRTState.Obstacles.Centers(i,2)]);
       end
       RRTState.Obstacles.radius_max(i) = max(RRTState.Obstacles.radius(i,:)) ;
    end
end
