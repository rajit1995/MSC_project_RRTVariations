clc;
clearvars;

vertices = [1,1;2,3;4,5;5,6;5,1;6,4;8,9;6,5.5];
branches = [1,2,2.361;2,3,2.8284;3,4,1.4142;2,5,3.6056;5,6,3.1623;4,7,4.2426;3,8,2.0615];
v_new = [6,4.8] ;

 min = 1000000; 
    for i=1:size(vertices,1)
            dist = norm(vertices(i,:) - v_new);
            if all(size(norm(min))==0) 
                min = dist;
                v_near = vertices(i,:);
            elseif dist<min
                min = dist;
                 v_near= vertices(i,:);
                 idx=i;
            end
    end
 %rewirring

 cost_ref = norm(vertices(idx,:)-v_new); 
R=2;
distances = sqrt(sum((vertices - v_new).^2, 2));


% Find the neighboring indices within the given radius
neighboringIndices = find(distances <= R);
vertices(size(vertices,1)+1,:) =v_new;
branches(size(branches,1)+1,:) = [idx,size(vertices,1),cost_ref];
for i=1:size(neighboringIndices)
    cost_new = norm(vertices(neighboringIndices(i,:),:)-v_new);
    
    
    idx;
    size(branches,1)
%search for any existing path/branches to the new node
    for j=1:size(branches,1)
       if branches(j,1:2) == [neighboringIndices(i,:), idx]
           total_cost = branches(j,3) + cost_ref;
           if cost_new < total_cost 
               new_branch = [neighboringIndices(i,:),size(vertices,1)+1, cost_new];
               branches(size(branches,1)+1,:) = new_branch;
               
           end
       end
    end
end


   
    
    

































%disp(branches(2,1))
%disp(branches(2,2))
%disp(vertices(branches(2,1),1));
%disp(vertices(branches(2,1),2));
%disp(vertices(branches(2,2),1));
%disp(vertices(branches(2,2),2));
scatter(vertices(:,1),vertices(:,2), 'filled', 'MarkerFaceColor', 'm');
hold on
scatter(v_new(1),v_new(2), 'filled', 'MarkerFaceColor', 'r');
hold on
for i=1:size(branches,1)
    plot([vertices(branches(i,1),1),vertices(branches(i,2),1)],[vertices(branches(i,1),2),vertices(branches(i,2),2)], 'b');
    hold on
end
xlim([0, 10]);
ylim([0, 10]);
hold off
