function UPDATE_WAYPOINT()
%UPDATE_WAYPOINT update next waypoint target for the mpc controller
%based on the L2 norm

% UPDATES OUR REFERENCE WAYPOINT BASED ON OUR CURRENT CAR STATES.
    
    global TP
    global TRAJECTORY
    
    if (norm(TP.trike_pos(1:2) - TP.waypoint(1:2)) < TP.waypoint_thresh...
                                                        && ~TP.last_pos)
        disp("GOT TO WAYPOINT !!");
        TP.waypoints_completed = [TP.waypoints_completed; TP.waypoint(1:2)];
        TP.prev_indx = TP.pos_indx;
        
        while (norm(TP.trike_pos(1:2) - TP.waypoint(1:2)) < TP.update_thresh...
                                                         && ~TP.last_pos)
            TP.pos_indx = TP.pos_indx + 1;
            TP.waypoint = TRAJECTORY.center_line(TP.pos_indx,:);
            if (TP.pos_indx == size(TRAJECTORY.center_line,1)) 
                TP.last_pos = true;
                disp("LAST WAYPOINT !!");
            end
        end
        TP.waypoint = TRAJECTORY.center_line(TP.pos_indx,:);
        
    end
    
    wp_dists = vecnorm((TRAJECTORY.center_line...
                                  -(TP.trike_pos(1:2)))');
    [val,TP.closest_indx] =  min(wp_dists);
    if (TP.closest_indx == size(TRAJECTORY.center_line,1)) 
                disp(val)
                TP.reach_last_pos = true;
                disp("REACHED LAST WAYPOINT !!");
    end
        
end

