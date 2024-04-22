function [edges] = GenPRM(milestones, map, edges)

K = 10;

for i=1:length(milestones)  
    point = milestones(i,:);
    neighbors = KNN(milestones, point, K);

    for j=1:length(neighbors)            
        [Collision, ~] = CheckCollision(point, neighbors(j,:), map); 

        if (Collision == 0) 
            edges = [edges; [milestones(i,:), neighbors(j,:)]];
            
        end
    end
end

