function [neighbors] = KNN(milestones, point, K)

    distances = sqrt(sum((milestones - point).^2, 2)); 
    [~, idx] = sort(distances);
    
    neighbors = milestones(idx(1:K), :);
end
