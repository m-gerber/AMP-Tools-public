%% ODE Function to Extract Path

function path = dynamics(t,x,u1,u2,carHeight)

    theta = x(5);
    v     = x(3);
    phi   = x(4);
    
    path = [v*cos(theta); v*sin(theta); u1; u2; v/carHeight*tan(phi)];

end