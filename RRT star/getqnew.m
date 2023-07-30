function RRTState = getqnew(RRTState)
    if norm(RRTState.q_near - RRTState.q_new) <= RRTState.StepSize
        RRTState.q_new = RRTState.q_new;
    else
        RRTState.q_new = RRTState.q_near + RRTState.StepSize*(RRTState.q_new - RRTState.q_near)/norm((RRTState.q_new - RRTState.q_near));
        
    end   
end
