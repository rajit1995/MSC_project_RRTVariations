function RRTState = getfinalpathvertices(RRTState)
RRTState.finalpathvertices = [];
for i = 1:size(RRTState.path,2)
RRTState.finalpathvertices(i,:) = RRTState.pathvertices(RRTState.path(1,i),:);
end

end