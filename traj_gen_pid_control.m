function [] = traj_gen_pid_control()
global TRAJECTORY

RIGHT_TURN = true;
STRAIGHT_LINE = true;
STRAIGHT_LINE_DISTURBED = true;
CHANGE_LINE = true;

if RIGHT_TURN
    disp("gen right turn trajectory");
    x = 0:0.1:10;
    y = zeros(1, length(x));
    traj_1 = [x', y'];
    theta = linspace(pi/2, 0, 20);
    x = 15*cos(theta);
    y = 15*sin(theta);
    x = x-x(1);
    y = y-y(1);
    traj_2 = [x', y']+traj_1(end, :);
    y = linspace(0,-10, 101);
    x = zeros(1, length(y));
    traj_3 = [x', y']+traj_2(end, :);
    TRAJECTORY.right_turn = [traj_1; traj_2; traj_3];
end 

if STRAIGHT_LINE
    disp("gen straight line trajectory");
    x = 1:1000;
    y = zeros(1, length(x));
    TRAJECTORY.straight_line = [x', y'];
    clear x y
end
    
if STRAIGHT_LINE_DISTURBED
    disp("gen disturbed straight line trajectory");
    x = 0:0.1:1000;
    y = zeros(1, length(x));
%     disp(size(y(5001:end)))
%     disp(size(x(1:5001)));
    y(5001:end) = y(5001:end) + x(1:5001)/5;
    TRAJECTORY.straight_line_disturbed = [x', y'];
end

if CHANGE_LINE
    disp("gen change lane trajectory");
    x = -50:0.2:50;
    y = atan(x);
    x = x * 5;
    TRAJECTORY.change_lane = [x', y'];
    TRAJECTORY.change_lane = TRAJECTORY.change_lane - TRAJECTORY.change_lane(1, :);
end

end

