function RRTState = GRRTSTAR1(RRTState)
  
    RRTState.q_new = RRTState.PointA;
    RRTState.iteration.count = 1;
    RRTState.q_near = RRTState.PointA;
    a=0.2;
    
    %rng(2);
    while RRTState.iteration.count < RRTState.iteration.max
        visualrrt(RRTState);
            %sampling Region
%         RRTState.q_new = [rand()*RRTState.Dimensions.Length,rand()*RRTState.Dimensions.Width];
            RRTState.R_new2goal = norm(RRTState.q_new-RRTState.PointB);
            if size(RRTState.pathvertices,1) < RRTState.iteration.max*a
                RRTState.R_new2goal = RRTState.R_new2goal*rand();
            end
              
        Ball_rand = [RRTState.R_new2goal*cos(2*pi*rand()),RRTState.R_new2goal*sin(2*pi*rand())];
        RRTState.q_new = RRTState.PointB + Ball_rand;


        %Find nearest node
        RRTState.distances = sqrt(sum((RRTState.pathvertices(:,1:2) - RRTState.q_new).^2, 2));
        [~,RRTState.nearidx] = min(RRTState.distances);
        
        RRTState.q_near = RRTState.pathvertices(RRTState.nearidx,1:2);
       
            RRTState = getqnew(RRTState);
        in = zeros(1,RRTState.Obstacles.Number);
        out = zeros(1,RRTState.Obstacles.Number);
        for i = 1:RRTState.Obstacles.Number
            [in(i),out(i)] = inpolygon(RRTState.q_new(1), RRTState.q_new(2), RRTState.Obstacles.X1(i,:), RRTState.Obstacles.Y1(i,:));
            indicator = [in,out];
            sum_ind = sum(indicator);
        end

        if sum_ind == 0  
        
                if RRTState.GoalReachInd ~=1
  
                    sz = size(RRTState.Branches1,1);
                   
                    RRTState.cost_ref = norm(RRTState.pathvertices(RRTState.nearidx,1:2)-RRTState.q_new);
                    RRTState.new_node_cost = RRTState.pathvertices(RRTState.nearidx,3) + RRTState.cost_ref;
                    if size(RRTState.pathvertices,1)>=1
                        
                        RRTState = rewireRRT1(RRTState,sz);
                    end
                else
                    RRTState.cost_ref = norm(RRTState.pathvertices(RRTState.nearidx,1:2)-RRTState.PointB);
                    RRTState.new_node_cost = RRTState.pathvertices(RRTState.nearidx,3) + RRTState.cost_ref;
                    RRTState.q_new = RRTState.PointB;
                    RRTState.q_near = RRTState.q_new;
                    sz = size(RRTState.Branches1,1);
                    RRTState = rewireRRT1(RRTState,sz);
                    RRTState = getpath(RRTState);
                    RRTState = getfinalpathvertices(RRTState);
                    RRTState.Final.dist = hypot(diff(RRTState.finalpathvertices(:,1)), diff(RRTState.finalpathvertices(:,2)))  ;  
                    RRTState.Final.dist_total = sum(RRTState.Final.dist);
                    RRTState.plotfinalpath = 1;
                      visualrrt(RRTState);
                     
                        break;
                     
                end
        else
            
           continue;
            
        end

      

        
   

       
                if norm(RRTState.q_new-RRTState.PointB) <= RRTState.Threshold
                    RRTState.GoalReachInd =1;
                   RRTState.Final.Iterations = RRTState.iteration.count;
                   
                    
                end

        RRTState.iteration.count = RRTState.iteration.count+1;
        

%disp( RRTState.iteration.count);
    end

end