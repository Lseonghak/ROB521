function [milestones] = GenSamples(row, col, nS, wall_dist, milestones ,map)

sample = [rand(nS,1)*col + 0.5, rand(nS,1)*row + 0.5];
sample_filter = MinDist2Edges(sample, map) >= wall_dist;

milestones = [milestones(1,:); sample(sample_filter,:) ; milestones(2,:)];

end

