function RRTState = getqnear(RRTState)
   min = 1000000; 
    for i=1:size(RRTState.pathvertices,1)
            dist = norm(RRTState.pathvertices(i,1:2) - RRTState.PointB);
            if dist<min
                min = dist;
                RRTState.q_near = RRTState.pathvertices(i,1:2);
                RRTState.nearidx = i;
            else 
                continue;
            end
     end
end
