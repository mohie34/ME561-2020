function  GEN_TRAJECTORY()
    global TRAJECTORY
    % generates a trajectory for MPC
    scale = 2;
    interpolate_resolution = 0.001;
    x_s_curve = scale*(-pi/2+0.07:0.01:pi/2-0.07);
    y_s_curve = tan(1/scale*x_s_curve);
    shift = -x_s_curve(1);

    x_center = shift+(x_s_curve(1):interpolate_resolution:x_s_curve(end));
    y_curve_interp = interp1(x_s_curve+shift,y_s_curve,x_center);

    xleft_lower = -0.75+shift+(x_s_curve(1):interpolate_resolution:x_s_curve(15));
    xleft_upper = -0.75+shift+(x_s_curve(280):interpolate_resolution:x_s_curve(end));
    xright_lower = 0.75+shift+(x_s_curve(1):interpolate_resolution:x_s_curve(15));
    xright_upper = 0.75+shift+(x_s_curve(280):interpolate_resolution:x_s_curve(end));

    yleft_lower = interp1((shift+x_s_curve(1:15)-0.75),y_s_curve(1:15),xleft_lower);
    yleft_upper = interp1((shift+x_s_curve(280:end)-0.75),y_s_curve(280:end),xleft_upper);
    yright_lower = interp1((shift+x_s_curve(1:15)+0.75),y_s_curve(1:15),xright_lower);
    yright_upper = interp1((shift+x_s_curve(280:end)+0.75),y_s_curve(280:end),xright_upper);

    % center_line = [(x_s_curve+shift)',y_s_curve'];
    % bounds_left_lower = [(shift+x_s_curve(1:15)-0.75)',y_s_curve(1:15)'];
    % bounds_left_upper = [(shift+x_s_curve(280:end)-0.75)',y_s_curve(280:end)'];
    % bounds_right_lower = [(shift+x_s_curve(1:15)+0.75)',y_s_curve(1:15)'];
    % bounds_right_upper = [(shift+x_s_curve(280:end)+0.75)',y_s_curve(280:end)'];

    TRAJECTORY.center_line = [x_center',y_curve_interp'];
    TRAJECTORY.bounds_left_lower = [xleft_lower',yleft_lower'];
    TRAJECTORY.bounds_left_upper = [xleft_upper',yleft_upper'];
    TRAJECTORY.bounds_right_lower = [xright_lower',yright_lower'];
    TRAJECTORY.bounds_right_upper = [xright_upper',yright_upper'];

    TRAJECTORY.start = TRAJECTORY.center_line(1,:);
    TRAJECTORY.init_heading = atan2(TRAJECTORY.center_line(10,2)-TRAJECTORY.center_line(1,2),...
                                       TRAJECTORY.center_line(10,1)-TRAJECTORY.center_line(1,1));

    % vq1 = interp1(bounds_left_lower(:,1),bounds_left_lower(:,2),-0.75:0.005:-0.48);
    % plot(-0.75:0.005:-0.48,vq1,'kO')

    axis equal

end

