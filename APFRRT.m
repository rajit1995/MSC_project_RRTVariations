function RRTState = APFRRT(RRTState)
    r_near = 10;
    r = 4;
    K_att = 3;
    K_epd = 2;
    K_rep = 1.5;
    RRTState.q_new = [-1,-1];
    RRTState.iteration.count = 1;
    RRTState.q_near = RRTState.PointA;

    while RRTState.iteration.count < RRTState.iteration.max
        visualrrt(RRTState);
        if RRTState.GoalReachInd ==1
            break;
        end

        %Generating the random point for the next node of RRT
        r_rand = rand()*r_near;

        %cosine_tetha = dot(RRTState.PointB , RRTState.q_near)/(norm(RRTState.PointB)*norm(RRTState.q_near));
        e_ng = (RRTState.PointB - RRTState.q_near)/norm((RRTState.PointB - RRTState.q_near));
        theta_d =  atand(e_ng(2)/e_ng(1));        
        theta = deg2rad(theta_d) ;
        
        Ball_rand = [r_rand*cos(2*pi*theta),r_rand*sin(2*pi*theta)];
        RRTState.q_new = RRTState.q_near + Ball_rand;

        %Updating the new node as per gravitational potential of the target
        e_nr = (RRTState.q_new - RRTState.q_near)/norm((RRTState.q_new - RRTState.q_near));
        
        F_gra = K_att*e_nr;
        Expand_fun = K_epd*e_ng;
        Growth = F_gra + Expand_fun;
        RRTState.q_new = RRTState.q_near + Growth;

        %Updating the new node as per repulsive potential of the obstacles
        F_rep = 0;
        for i = 1:RRTState.Obstacles.Number
            e_vn(i,:) = (RRTState.q_new - RRTState.Obstacles.Centers(i,:)  ) /norm (RRTState.q_new - RRTState.Obstacles.Centers(i,:) );
            
            if norm(RRTState.Obstacles.Centers(i,:) - RRTState.q_new) < RRTState.Obstacles.radius_max(i)
                F_rep = F_rep + K_rep*e_vn(i,:);
            else 
                F_rep = F_rep + 0;
            end
        end
        
        RRTState.q_new = RRTState.q_new + F_rep;
        RRTState = getqnear(RRTState);
        %RRTState.q_near=RRTState.q_new;
        if norm(RRTState.q_near-RRTState.PointB) <= RRTState.Threshold
            RRTState.GoalReachInd =1;
            RRTState.pathvertices(RRTState.iteration.count+1,:) = RRTState.q_near;
            disp(RRTState.iteration.count)
            
        end
        if RRTState.GoalReachInd ~=1
            RRTState.pathvertices(RRTState.iteration.count+1,:) = RRTState.q_near;
        else
            RRTState.pathvertices(RRTState.iteration.count+1,:) = RRTState.PointB;
            RRTState.q_near  = RRTState.PointB;
            RRTState.q_new  = RRTState.PointB;
            
        end
        RRTState.iteration.count = RRTState.iteration.count+1;
    end

end