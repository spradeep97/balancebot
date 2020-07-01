function plot_bot(y_vec,u_vec,ref_vec,tvec, draw_robot, fig_num)

    % Draws the robot data for more concrete visualization of what is
    % going on with the states and control input.
    
    global Model
    
    figure(fig_num)
    if(draw_robot)
        body_angle = y_vec(1,end);
        wheel_angle = y_vec(2,end);
        wheel_radius = Model.radius;
        bot_height = Model.bot_height; %bot height in cm (change it for more accurate graphics)

        x_pos = -1*wheel_angle*wheel_radius*100;
        draw_radius = wheel_radius*100; %use cm as units of drawing    


        h1 = subplot(2,2,1);
        cla(h1)   
        draw_bumper(x_pos, body_angle, draw_radius)
        hold on
        draw_body(x_pos, body_angle, bot_height, draw_radius)
        hold on
        draw_wheel(x_pos, wheel_angle, draw_radius)
        hold on
        draw_top(x_pos, body_angle, bot_height, draw_radius)
        axis([x_pos-bot_height x_pos+bot_height 0 bot_height+5])

        hold off
    end
    
    subplot(2,2,2);
    plot(tvec,u_vec);
    title("Duty Cycle vs Time")
    xlabel("Time [s]")
    ylabel("Duty Cycle [unitless]")
    
    subplot(2,2,3);
    plot(tvec,y_vec(1,:));
    hold on
    plot(tvec,ref_vec(1,:),'k');
    hold off
    title("Sim&Ref Body Angle vs Time")
    xlabel("Time [s]")
    ylabel("Body Angle [rad]")
%     legend('Act', 'Ref');
    
    subplot(2,2,4);
    plot(tvec,y_vec(2,:));
    hold on
    plot(tvec,ref_vec(2,:),'k');
    hold off
    title("Sim&Ref Wheel Angle vs Time")
    xlabel("Time [s]")
    ylabel("Wheel Angle [rad]")
%     legend('Act', 'Ref');
    pause(0.00001)
end

function draw_body(x_pos, body_angle, bot_height, draw_radius)

    bot_width = 3;
    rec_xpos = [x_pos-0.5*bot_width x_pos-0.5*bot_width...
           x_pos+0.5*bot_width x_pos+0.5*bot_width]; 
    rec_ypos = [draw_radius draw_radius+bot_height...
       draw_radius+bot_height draw_radius];
    body = patch('XData',rec_xpos,'YData',rec_ypos,'FaceColor','y');
    rotate(body,[0 0 1],body_angle*180/pi,[x_pos draw_radius 0])
end

function draw_wheel(x_pos, wheel_angle, draw_radius)
    
    pos = [x_pos-draw_radius 0 2*draw_radius 2*draw_radius]; 
    rectangle('Position',pos,'Curvature',[1 1],'FaceColor',...
        [.5 .5 .5],'EdgeColor','k','LineWidth',2)
    hold on
    a = 30*pi/180;
    rim_xpos = [x_pos-draw_radius*cos(a) x_pos...
           x_pos+draw_radius*cos(a) x_pos]; 
    rim_ypos = [draw_radius-draw_radius*sin(a) 2*draw_radius...
       draw_radius-draw_radius*sin(a) draw_radius]; %draw_radius
    rim = patch('XData',rim_xpos,'YData',rim_ypos,'FaceColor','k');
    rotate(rim,[0 0 1],wheel_angle*180/pi,[x_pos draw_radius 0])
end

function draw_top(x_pos, body_angle, bot_height, draw_radius)

    top_width = 12;
    top_thickness = 1.5;
    top_xpos = [x_pos-0.5*top_width x_pos-0.5*top_width...
           x_pos+0.5*top_width x_pos+0.5*top_width]; 
    top_ypos = [0 top_thickness top_thickness 0]+bot_height-top_thickness+draw_radius;
   
    top = patch('XData',top_xpos,'YData',top_ypos,'FaceColor','b');
    rotate(top,[0 0 1],body_angle*180/pi,[x_pos draw_radius 0])
    
end


function draw_bumper(x_pos, body_angle, draw_radius)

    bumper_width = 12;
    bumper_thickness = 1.5;
    bumper_xpos = [x_pos-0.5*bumper_width x_pos-0.5*bumper_width...
           x_pos+0.5*bumper_width x_pos+0.5*bumper_width]; 
    bumper_ypos = [0 bumper_thickness bumper_thickness 0]+2*draw_radius;
   
    bumper = patch('XData',bumper_xpos,'YData',bumper_ypos,'FaceColor',[0.3 0.3 0.3]);
    rotate(bumper,[0 0 1],body_angle*180/pi,[x_pos draw_radius 0])
    
end