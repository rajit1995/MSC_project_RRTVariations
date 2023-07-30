function RRTState = polyintersect(RRTState)
B = [RRTState.pathvertices(RRTState.nearidx,1),RRTState.q_new(1)];
C = [RRTState.pathvertices(RRTState.nearidx,2),RRTState.q_new(2)];
int_ind =[];
for i=1:RRTState.Obstacles.Number
[xi,yi] = polyxpoly(B,C,RRTState.Obstacles.X(i,:), RRTState.Obstacles.Y(i,:));
int_ind = [int_ind,xi,yi];
end
RRTState.int_ind = sum(int_ind);
end