function RRTState = rewireRRT(RRTState)
 
        % Find nearby nodes within the neighborhood radius
        neighborDistances = pdist2(RRTState.q_new, RRTState.Branches(:, 1:2));
        nearNodesIndices = find(neighborDistances <= RRTState.neighborhoodRadius)
        
        % Calculate costs to reach the new point
        costs = RRTState.Branches(nearNodesIndices, 5) + norm(repmat(RRTState.q_new, size(nearNodesIndices, 1), 1) - RRTState.Branches(nearNodesIndices, 1:2), 2);
        minCostIdx = nearNodesIndices(find(costs == min(costs), 1));
        
        % Rewire edges if a better cost is found
        for j = 1:length(nearNodesIndices)
            
            
            % Calculate the new cost
            edgeLength = norm(RRTState.q_new - RRTState.Branches(nearNodesIndices(j), 1:2));
            newCost = RRTState.Branches(minCostIdx, 5) + edgeLength;
            
            % Rewire if the new cost is lower
            if newCost < RRTState.Branches(nearNodesIndices(j), 5) && edgeLength <= RRTState.StepSize
                RRTState.Branches(nearNodesIndices(j), 3:5) = [RRTState.q_new, newCost];
            end
        end
        
        % Add the new point to the tree
        RRTState.Branches = [RRTState.Branches; RRTState.q_near, RRTState.q_new, RRTState.Branches(minCostIdx, 5) + norm(RRTState.q_new - RRTState.Branches(minCostIdx, 1:2))];
   
end