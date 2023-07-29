function RRTState = rewireRRT1(RRTState,sz)
        RRTState.new_branch = zeros(1,3);
        % RRTState.cost_ref = norm(RRTState.pathvertices(RRTState.nearidx,:)-RRTState.q_new);
        RRTState.distances = sqrt(sum((RRTState.pathvertices(:,1:2) - RRTState.q_new).^2, 2));
        RRTState.neighboringIndices = find(RRTState.distances <= RRTState.rwradius);
        
        vert_sz = size(RRTState.pathvertices,1);
        RRTState.new_branch = [RRTState.nearidx,vert_sz,RRTState.cost_ref];
       % RRTState.Branches1(sz+1,:) = [RRTState.nearidx,size(RRTState.pathvertices,1),RRTState.cost_ref];
        %RRTState.Branches1(sz+1,:) = RRTState.new_branch
      %  cost_ref = norm(RRTState.pathvertices(RRTState.nearidx,:)-RRTState.q_new);
      RRTState.cost_new_ref = 0;
        for i=1:size(RRTState.neighboringIndices)
            RRTState.cost_new_ref = 0;
            RRTState.cost_new = norm(RRTState.pathvertices(RRTState.neighboringIndices(i,:),:)-RRTState.q_new);
            
           % test_branch = [int64(RRTState.neighboringIndices(i,:)), int64(RRTState.nearidx)];         
            %search for any existing path/branches to the new node
            for j=1:size(RRTState.Branches1,1)
               %disp('hello for') 
               if (RRTState.Branches1(j,1) == RRTState.neighboringIndices(i,:)) && (RRTState.Branches1(j,2) == RRTState.nearidx) 
                   total_cost = RRTState.Branches1(j,3) + RRTState.cost_ref;
                   disp('hello')
                  % branch_idx = j;
                   if (RRTState.cost_new  <= total_cost  )
                   %if ( RRTState.cost_ref >= RRTState.cost_new || RRTState.cost_new_ref >= RRTState.cost_new)  
                     %  if (RRTState.cost_new  < total_cost  )
                        
                       disp('hello if');
                       RRTState.cost_new_ref = RRTState.cost_new;
                   
                       new_branch1 = RRTState.neighboringIndices(i,:);
                       new_branch2 =size(RRTState.pathvertices,1);
                       RRTState.new_branch
                       RRTState.new_branch = [new_branch1,new_branch2, RRTState.cost_new ];
                    
                       disp('hello2');
                       RRTState.new_branch;
                  
                     elseif RRTState.cost_ref >= RRTState.cost_new
                     RRTState.cost_new_ref = RRTState.cost_new;
                    disp('hello if');
                       RRTState.cost_new_ref = RRTState.cost_new;
                   
                       new_branch1 = RRTState.neighboringIndices(i,:);
                       new_branch2 =size(RRTState.pathvertices,1);
                       RRTState.new_branch
                       RRTState.new_branch = [new_branch1,new_branch2, RRTState.cost_new ];
                    
                       disp('hello2');
                       RRTState.new_branch;
                   else 
                       disp('hello else');
                       RRTState.new_branch = [RRTState.nearidx,vert_sz,RRTState.cost_ref];
                        %RRTState.Branches1(sz+1,:) = RRTState.new_branch;
                       %new_branch = [RRTState.nearidx,vert_sz,RRTState.cost_ref];
                       %continue;
                      
                      
                   end
               else
                   RRTState.new_branch = [RRTState.nearidx,vert_sz,RRTState.cost_ref];
                    % RRTState.Branches1(sz+1,:) = RRTState.new_branch;
                    
               end
               %RRTState.Branches1(sz+1,:)
               RRTState.Branches1(sz+1,:) = RRTState.new_branch;
            end

            
        end



end