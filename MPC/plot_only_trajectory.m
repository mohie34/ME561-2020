function plot_only_trajectory()
%Function to have the trajectory printing in a figure of its own

    clf(figure(2))
    global DATA
    global TRAJECTORY
    global TP
    
    
    plot(TRAJECTORY.center_line(:,1),TRAJECTORY.center_line(:,2),'b')
    hold on
    plot(TRAJECTORY.bounds_left_lower(:,1),TRAJECTORY.bounds_left_lower(:,2), 'r')
    hold on
    plot(TRAJECTORY.bounds_left_upper(:,1),TRAJECTORY.bounds_left_upper(:,2), 'r')
    hold on
    plot(TRAJECTORY.bounds_right_lower(:,1),TRAJECTORY.bounds_right_lower(:,2), 'r')
    hold on
    plot(TRAJECTORY.bounds_right_upper(:,1),TRAJECTORY.bounds_right_upper(:,2), 'r')
    title("Trajectory X(t) vs Y(t)")
    % DRAW TARGET
    if(size(TP.waypoints_completed,1)>1)
        for i = 2:size(TP.waypoints_completed,1)
            hold on
            wpt_done = TP.waypoints_completed(i,:);
            plot(wpt_done(1),wpt_done(2),'gX')
            circle_green(wpt_done(1),wpt_done(2), TP.waypoint_thresh);
            axis equal
        end
    end
    hold on
    plot(TP.waypoint(1),TP.waypoint(2),'gX')
    circle_blue(TP.waypoint(1),TP.waypoint(2), TP.waypoint_thresh);
    hold on
    plot(DATA.X_sim(:,7),DATA.X_sim(:,8),'k','linewidth',3)   
    axis equal
    
end

function h = circle_green(x,y,r)
    hold on
    th = 0:pi/2:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit,'g');
end


function h = circle_blue(x,y,r)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit,'b');
end