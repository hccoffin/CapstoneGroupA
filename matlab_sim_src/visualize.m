function [] = visualize(t,y,r,l)
    figure(1)
    h = animatedline;
    axis([-1 1 0 1])
    
    ang=0:0.01:2*pi; 
    for i = 1:length(t)
        clearpoints(h)
        %% Access important State Variables
        theta = y(i,1);
        phi = y(i,3);
        
        %% Compute new body positions 
        x_wheel = theta*r;
        y_wheel = r;
        x_body = x_wheel + l*sin(phi);
        y_body = y_wheel + l*cos(phi);
        
        %% Draw circle
        addpoints(h, x_wheel + r*cos(ang), y_wheel + r*sin(ang))
        
        %% Draw body
        addpoints(h, [x_wheel x_body], [y_wheel y_body])
        drawnow
        pause(0.05)
    end
end

