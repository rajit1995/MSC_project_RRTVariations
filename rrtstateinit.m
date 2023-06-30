function RRTState = rrtstateinit(pointA,pointB,Dimensions,Obstacles)
RRTState.PointA = pointA;
RRTState.PointB = pointB;

RRTState.Dimensions = Dimensions;
RRTState.Obstacles = Obstacles;

RRTState.q_near = [-1,-1];
RRTState.q_new = [-1,-1];
RRTState.iteration.count = 1;
RRTState.iteration.max = 100;
RRTState.Terraincolour = [0.800000000000000,0.600000000000000,0.400000000000000];
RRTState.pathvertices(1,:) = pointA;
RRTSate.Threshold = 0.1;
RRTState.StepSize = 5;
end
