function RRTState = rewireRRT1(RRTState,~)
        RRTState.new_branch = zeros(1,3);
        RRTState.distances = sqrt(sum((RRTState.pathvertices(:,1:2) - RRTState.q_new).^2, 2));
        RRTState.neighboringIndices = find(RRTState.distances <= RRTState.rwradius);
        
        vert_sz = size(RRTState.pathvertices,1);
        RRTState.new_branch = [RRTState.nearidx,vert_sz,RRTState.cost_ref];
     % Branch_ref = norm(RRTState.pathvertices(RRTState.neighboringIndices(i),1:2)-RRTState.q_new)
         Branch_ref = norm(RRTState.pathvertices(RRTState.nearidx,1:2)-RRTState.q_new);
            for i=1:size(RRTState.neighboringIndices)
            cost_new = RRTState.pathvertices(RRTState.neighboringIndices(i),3) +...
                norm(RRTState.pathvertices(RRTState.neighboringIndices(i),1:2)-RRTState.q_new);
            Branch_ref = norm(RRTState.pathvertices(RRTState.neighboringIndices(i),1:2)-RRTState.q_new);
            if cost_new <= RRTState.new_node_cost
                %disp('Rewire')
                RRTState.new_node_cost = cost_new;
                %
                % nearidx = RRTState.neighboringIndices(i,:);
                 RRTState.nearidx = RRTState.neighboringIndices(i,:);
            else
                continue;
            end 
            end
        
       
     RRTState.pathvertices(size(RRTState.pathvertices,1)+1,:) =[RRTState.q_new,RRTState.new_node_cost];
    
     RRTState.Branches1(size(RRTState.Branches1,1)+1,:) = [RRTState.nearidx,size(RRTState.pathvertices,1),Branch_ref];




end