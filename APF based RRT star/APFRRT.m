function RRTState = APFRRT(RRTState)
    r_near =5;
    K_att = 3;
    K_epd = 2;
    K_rep = 2;
    RRTState.q_new = RRTState.PointA;
    RRTState.iteration.count = 1;
    RRTState.q_near = RRTState.PointA;
    %rng(2);
    while RRTState.iteration.count < RRTState.iteration.max
   % while RRTState.iteration.count < 10
        visualrrt(RRTState);
      if RRTState.GoalReachInd ==1 
          % %RRTState = getpath(RRTState);
          % %RRTState = getfinalpathvertices(RRTState);
          % % RRTState.Final.dist = hypot(diff(RRTState.finalpathvertices(:,1)), diff(RRTState.finalpathvertices(:,2)))  ;  
          % % RRTState.Final.dist_total = sum(RRTState.Final.dist);
          % % RRTState.plotfinalpath = 1;
          % visualrrt(RRTState);
          % 
          %   break;
       end
       


        %Generating the random point for the next node of RRT
        r_rand = rand()*r_near;
        %e_ng = (RRTState.PointB - RRTState.q_near)/norm((RRTState.PointB - RRTState.q_near));

        theta = rand;
        Ball_rand = [r_rand*cos(2*pi*theta),r_rand*sin(2*pi*theta)];
        RRTState.q_new = RRTState.q_near + Ball_rand;

        %Updating the new node as per gravitational potential of the target
        e_nr = (RRTState.q_new - RRTState.q_near)/norm((RRTState.q_new - RRTState.q_near));
        e_ng = (RRTState.PointB - RRTState.q_near)/norm((RRTState.PointB - RRTState.q_near));
        F_gra = K_att*e_ng;
        Expand_fun = K_epd*e_nr;
        Growth = F_gra + Expand_fun;
        RRTState.q_new = RRTState.q_near + Growth;

        %Updating the new node as per repulsive potential of the obstacles
        F_rep = 0;
        for i = 1:RRTState.Obstacles.Number
            e_vn = (RRTState.q_new -[RRTState.Obstacles.Centers(i)]) /norm (RRTState.q_new - [RRTState.Obstacles.Centers(i)] );
            if norm(RRTState.Obstacles.Centers(i,:) - RRTState.q_new) <= RRTState.Obstacles.radius_max(i)+0.5
                F_rep = F_rep + K_rep*e_vn;
            else 
                F_rep = F_rep + 0 ;
            end
        end
        
        RRTState.q_new = RRTState.q_new + F_rep;
        
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

        q_near_bkp=RRTState.q_near;
        if RRTState.StepSize ~= 5
            RRTState = getqnear(RRTState);
        else 
            RRTState.q_near = RRTState.pathvertices(length(RRTState.pathvertices),1:2);
        end
        RRTState = getqnear(RRTState);

        
    %    error = norm(RRTState.q_near - q_near_bkp)
        if RRTState.q_near == q_near_bkp
            RRTState.q_near_count = RRTState.q_near_count +1;
        else 
            RRTState.q_near_count = 0;
        end

         if RRTState.q_near_count > 5 
                RRTState.StepSize = 5;
               
        else
            RRTState.StepSize = 1;
           

        end

                if norm(RRTState.q_new-RRTState.PointB) <= RRTState.Threshold
                    RRTState.GoalReachInd =1;
                   RRTState.Final.Iterations = RRTState.iteration.count;
                   
                    
                end

        RRTState.iteration.count = RRTState.iteration.count+1;
        

disp( RRTState.iteration.count);
    end

end