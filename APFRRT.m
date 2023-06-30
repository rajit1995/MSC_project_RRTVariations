function RRTState = APFRRT(RRTState)
    r_near = 15
    r = 5;
    K_att = 3;
    K_epd = 2;
    K_rep = 1.5;
    RRTState.q_new = [-1,-1];
    RRTState.iteration.count = 1;
    RRTState.q_near = RRTState.PointA;

    while RRTState.iteration.count < RRTState.iteration.max
        visualrrt(RRTState);

        %Generating the random point for the next node of RRT
        r_rand = rand()*r_near
        theta = rand()*(2*pi)
        Ball_rand = [r_rand*cos(theta),r_rand*sin(theta)]
        RRTState.q_new = RRTState.q_near + Ball_rand

        %Updating the new node as per gravitational potential of the target
        e_nr = (RRTState.q_new - RRTState.q_near)/norm((RRTState.q_new - RRTState.q_near));
        e_ng = (RRTState.PointB - RRTState.q_near)/norm((RRTState.PointB - RRTState.q_near));
        F_gra = K_att*e_nr;
        Expand_fun = K_epd*e_ng;
        Growth = F_gra + Expand_fun;
        RRTState.q_new = RRTState.q_near + Growth;

        %Updating the new node as per repulsive potential of the obstacles
        F_rep = 0;
        for i = 1:RRTState.Obstacles.Number
            e_vn(i,:) = (RRTState.q_new - RRTState.Obstacles.Centers(i,:) ) /norm (RRTState.q_new - RRTState.Obstacles.Centers(i,:) );
            dist_qo(i) = norm(RRTState.Obstacles.Centers(i,:) - RRTState.q_new);
            if norm(dist_qo(i)) < r
                F_rep = F_rep + K_rep*e_vn(i,:);
            else 
                F_rep = F_rep + 0;
            end
        end
        
        RRTState.q_new = RRTState.q_new + F_rep;
        if norm(RRTState.q_near - RRTState.q_new) < RRTState.StepSize
            RRTState.q_near = RRTState.q_new;
        else 
            RRTState.q_near = RRTState.q_near + RRTState.StepSize*(RRTState.q_new - RRTState.q_near)/norm((RRTState.q_new - RRTState.q_near))
        end


        
        RRTState.pathvertices(RRTState.iteration.count+1,:) = RRTState.q_near;

                        

        RRTState.iteration.count = RRTState.iteration.count+1;
    end

end