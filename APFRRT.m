function RRTState = APFRRT(RRTState)
    r_near = 10;
    K_att = 3;
    K_epd = 2;
    K_rep = 2;
    RRTState.q_new = RRTState.PointA;
    RRTState.iteration.count = 1;
    RRTState.q_near = RRTState.PointA;

    while RRTState.iteration.count < RRTState.iteration.max
        visualrrt(RRTState);
        if RRTState.GoalReachInd ==1
            dist = hypot(diff(RRTState.pathvertices(:,1)), diff(RRTState.pathvertices(:,2)))  ;   
            RRTState.Final.dist_total = sum(dist);                
        
            
            break;
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
            
            if norm(RRTState.Obstacles.Centers(i,:) - RRTState.q_new) <= RRTState.Obstacles.radius_max(i)  +1.5
                F_rep = F_rep + K_rep*e_vn;
            else 
                F_rep = F_rep + 0 ;
            end
        end
        
        RRTState.q_new = RRTState.q_new + F_rep;
        
            %q_near_bkp = RRTState.q_near;
            RRTState = getqnew(RRTState);
       
%        RRTState.Branches(RRTState.iteration.count,:) = [RRTState.q_near(1),RRTState.q_near(2),RRTState.q_new(1),RRTState.q_new(2) 0];

        %RRTState.q_near=RRTState.q_new;
        for i = 1:RRTState.Obstacles.Number
            [in(i),out(i)] = inpolygon(RRTState.q_new(1), RRTState.q_new(2), RRTState.Obstacles.X(i,:), RRTState.Obstacles.Y(i,:));
            indicator = [in,out];
            sum_ind = sum(indicator);
        end

        if sum_ind == 0  && norm(RRTState.q_new-RRTState.pathvertices(RRTState.iteration.count,:))> 0.5
              
                if norm(RRTState.q_new-RRTState.PointB) <= RRTState.Threshold
                    RRTState.GoalReachInd =1;
                    RRTState.pathvertices(RRTState.iteration.count+1,:) = RRTState.q_new;
                    RRTState.Final.Iterations = RRTState.iteration.count;
                   
                    
                end
        
                if RRTState.GoalReachInd ~=1
                    RRTState.pathvertices(RRTState.iteration.count+1,:) = RRTState.q_new;
                  %  RRTState.Branches(RRTState.iteration.count,:) = [RRTState.q_near(1),RRTState.q_near(2),RRTState.q_new(1),RRTState.q_new(2) ];

                    RRTState = rewireRRT(RRTState);
                else
                    RRTState.pathvertices(RRTState.iteration.count+1,:) = RRTState.q_new;
                    RRTState.pathvertices(RRTState.iteration.count+2,:) = RRTState.PointB;
                 %   RRTState.Branches(RRTState.iteration.count,:) = [RRTState.q_near(1),RRTState.q_near(2),RRTState.q_new(1),RRTState.q_new(2) ];
                   
                    RRTState.q_near  = RRTState.PointB;
                    RRTState.q_new  = RRTState.PointB;
                    
                end
        else
            
           continue;
            
        end
        q_near_bkp=RRTState.q_near;
        RRTState = getqnear(RRTState);

        
    %    error = norm(RRTState.q_near - q_near_bkp)
        if RRTState.q_near == q_near_bkp
            q_near_count = q_near_count +1;
        else 
            q_near_count = 0;
        end

        if q_near_count > 5 
                RRTState.StepSize = 5;
                K_att = 0;
                K_epd = 2;
                K_rep = -2;
        else
            RRTState.StepSize = 1;
            K_att = 3;
            K_epd = 2;
            K_rep = 1.5;
           
        end




        RRTState.iteration.count = RRTState.iteration.count+1;
        


    end

end