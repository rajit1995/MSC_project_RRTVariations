function Rover = obstacledetect(Rover)
  Rover.pos_curr1 = Rover.pos_curr + Rover.disp;
 for j=1:36
       Rover.RovCircle(j,:) = [(Rover.pos_curr1(1) + (Rover.Radius+0.01)*cos(j*Rover.theta_rad)) (Rover.pos_curr1(2) + (Rover.Radius+0.01)*sin(j*Rover.theta_rad))];      
  end
    poly1 = polyshape([Rover.RovCircle(:,1)],[Rover.RovCircle(:,2)]);
    
    for j= 1:Rover.Obstacles.Number
        poly2 = polyshape([Rover.Obstacles.X(j,:)],[Rover.Obstacles.Y(j,:)]);
        polyout = intersect(poly1,poly2);
        Rover.polyind(j,:) =  polyout.NumRegions;
    end
    Rover.poly_ind = sum(Rover.polyind);
    if Rover.polyind == 0
        Rover.pos_curr = Rover.pos_curr1;
    else
       new_head= acos(sum(Rover.pos_curr.*Rover.next_wayPoint)/(norm(Rover.pos_curr)*norm(Rover.next_wayPoint)));
       if new_head > 0
           new_head_angle = pi/2;
       elseif new_head < 0
           new_head_angle = -1* pi/2;
       else
           new_head_angle = pi;
       end
        Dcos = [cos(new_head_angle) -sin(new_head_angle);sin(new_head_angle) cos(new_head_angle)] ;
        Rover.u_sur = transpose(Dcos*transpose(Rover.u_sur));
        Rover.disp = Rover.u_sur*Rover.dt;
        Rover = obstacledetect(Rover);
        % Rover.pos_curr1 = Rover.pos_curr + Rover.disp;


    end

end