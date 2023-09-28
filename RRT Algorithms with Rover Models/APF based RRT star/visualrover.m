function visualrover(Rover)
idx=2;
    if Rover.counter~=1
        clf(idx)
    else
        figure(idx)
    end
     rover_plot(Rover);
    pause(0)
   
end