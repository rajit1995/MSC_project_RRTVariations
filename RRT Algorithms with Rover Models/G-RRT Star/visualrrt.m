function visualrrt(RRTState)
idx=1;
    if RRTState.iteration.count~=1
        clf(idx)
    else
        figure(idx)
    end
    rrt_plot(RRTState);
    pause(0)
   
end