function Rover = losapnav(Rover)
 for i = 2:size(Rover.waypoints,1)
      Rover.next_wayPoint = Rover.waypoints(i,1:2); 
      if i==size(Rover.waypoints,1)
          Rover.RadiusAcc = 0.01;
          Rover.LoSRadius = norm(Rover.pos_curr - Rover.next_wayPoint);
      else 
         
          Rover.RadiusAcc = norm(Rover.prev_wayPoint-Rover.next_wayPoint)*0.3;
          
      end

    while Rover.wpacc_ind ~=1
        Rover.counter = Rover.counter+1;
     for j=1:36
       Rover.LosCircle(j,:) = [(Rover.pos_curr(1) + Rover.LoSRadius*cos(j*Rover.theta_rad)) (Rover.pos_curr(2) + Rover.LoSRadius*sin(j*Rover.theta_rad))];
       
    end

    B = [Rover.prev_wayPoint(:,1),Rover.next_wayPoint(:,1)];
    C = [Rover.prev_wayPoint(:,2),Rover.next_wayPoint(:,2)];
    [xi,yi] = polyxpoly(B,C,Rover.LosCircle(:,1),Rover.LosCircle(:,2));
        if size(xi,1) > 1
            if norm([xi(1) yi(1)] - Rover.next_wayPoint) < norm([xi(2) yi(2)] - Rover.next_wayPoint)
                Rover.pos_des = [xi(1) yi(1)];
            else
                    Rover.pos_des = [xi(2) yi(2)];
            end
        elseif size(xi,1) == 1
            Rover.pos_des = [xi yi];
           
        else 
            Rover.pos_des = Rover.next_wayPoint;
        end
       
        % norma = norm(Rover.pos_curr - Rover.next_wayPoint);
        if norm(Rover.pos_curr - Rover.next_wayPoint) <= Rover.RadiusAcc 
             Rover.wpacc_ind = 1;
        elseif norm(Rover.pos_des - Rover.prev_wayPoint) < norm(Rover.pos_des - Rover.next_wayPoint) && i~= size(Rover.waypoints,1)
            Rover.wpacc_ind = 1;
        else
             Rover.wpacc_ind = 0;
        
         Rover.e_u = Rover.pos_des - Rover.pos_curr;
        
         Rover.u_sur = Rover.Kpu*Rover.e_u + Rover.Kiu*(Rover.e_u+Rover.e_u_1)*Rover.dt/2 + Rover.kdu*(Rover.e_u - Rover.e_u_1)/Rover.dt;
        Rover.a = (Rover.u_sur - Rover.u_sur_1)/Rover.dt;
         Rover.disp = Rover.u_sur_1*Rover.dt + 0.5*Rover.a*Rover.dt*Rover.dt;
         Rover = obstacledetect(Rover) ;
         Rover.obst =  [Rover.obst;Rover.poly_ind];
        Rover.pos_curr = Rover.pos_curr + Rover.disp;
        % pos_curr = Rover.pos_curr;
        Rover.Travel = [Rover.Travel;Rover.pos_curr];
         Rover.e_u_1 = Rover.e_u;
         Rover.u_sur_1 = Rover.u_sur;
         visualrover(Rover);
        end
    end
       Rover.prev_wayPoint = Rover.next_wayPoint;
       Rover.wpacc_ind = 0;
    
  end
  
    
