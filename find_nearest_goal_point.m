function [goal_point] = find_nearest_goal_point(traj)
global DATA

traj_size = size(traj);
length = traj_size(1);
goal_point = traj(end, :);
min_dist = 100;
nearest_point_idx = [];
for i = 1:length
    query_point = traj(i, :);
%     disp(DATA.X_curr(7:8));
%     disp(query_point);
    dist = norm(DATA.X_curr(7:8)-query_point);
    if dist < min_dist
        min_dist = dist;
        nearest_point_idx = i;
    end
end

for i = nearest_point_idx:length-2
    query_point = traj(i, :);
    dist = norm(DATA.X_curr(7:8)-query_point);
    disp("dist")
    disp(dist)
    dist_to_end = norm(DATA.X_curr(7:8)-traj(end, :));
    if dist <= 0.5
        continue
    end
    
    if dist > 0.5
        goal_point = traj(i+2, :);
        disp("dist > 0.5")
        break
    elseif dist_to_end < 1
        goal_point = traj(end, :);
        break
    end 
end

end

