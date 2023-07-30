function RRTState = polyintersect(RRTState)
B = [RRTState.pathvertices(RRTState.nearidx,1),RRTState.q_new(1)]
C = [RRTState.pathvertices(RRTState.nearidx,2),RRTState.q_new(2)]
for i=1:RRTState.Obstacles.Number
[xi,yi] = polyxpoly(B,C,RRTState.Obstacles.X(i,:), RRTState.Obstacles.Y(i,:))
end
end