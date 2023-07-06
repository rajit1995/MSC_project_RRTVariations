function RRTState = getqnear(RRTState)
   min = 1000000; 
    for i=1:size(RRTState.pathvertices,1)
            dist = norm(RRTState.pathvertices(i,:) - RRTState.PointB);
            if all(size(norm(min))==0) 
                min = dist;
                RRTState.q_near = RRTState.pathvertices(i,:);
            elseif dist<min
                min = dist;
                RRTState.q_near = RRTState.pathvertices(i,:);
            end
     end
end
