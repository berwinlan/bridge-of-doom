function starterCodeForBridgeOfDoomQEA2021()

% visualize = false;

% Explicityly defining u
u = [];
% u will be our parameter
syms u;

% Assumuptions
assume(u,'real');
assume(u,'positive');

% Dilation - conversion from the raw parameterization of the curve to 
% time.
% As the dilation increases, the time that the robot will take to travel
% along the curve increases.
dilation = 10;

% Equation of the bridge (scaled by dilation)
R = 4*[0.396*cos(2.65*((u/dilation)+1.4));...
       -0.99*sin((u/dilation)+1.4);...
       0];
   
% Bounds of u (with respect to dilation)
timeBounds = [0 (3.2*dilation)];

% Tangent vector
T = diff(R);

% Normalized tangent vector
That = T/norm(T);

% Linear speed vector
linearSpeed = simplify(norm(T));

% Angular velocity vector
omega = cross(That, diff(That));

% Left and Right Wheel Velocities
d = 0.235;
velocity_left = linearSpeed - ((d/2)*omega(3));
velocity_right = linearSpeed + ((d/2)*omega(3));

% visualize the path
% if visualize
%     figure;
%     fplot(R(1),R(2), timeBounds);
%     axis equal;
%     
%     figure;
%     
%     fplot(linearSpeed, timeBounds);
%     hold on;
%     fplot(omega(3), timeBounds);
%     legend({Linear velocity','Angular velocity'});
% end

pub = rospublisher('raw_vel');

% Pause 1 second
pause(1);

% Stop the robot if it's going right now
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);

% Pause 1 second
pause(1);

bridgeStart = double(subs(R,u,timeBounds(1)));
startingThat = double(subs(That,u,timeBounds(1)));
placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), ...
startingThat(2));

% Wait a bit for robot to fall onto the bridge
pause(1);

% Driving
rostic;
u_val = 0;
while u_val < timeBounds(2)
    t = rostoc;
    u_val = t;
    msg = rosmessage(pub);
    % Substitute current time (u value) into expressions above
    velocity_L = double(subs(velocity_left, u, u_val));
    velocity_R = double(subs(velocity_right, u, u_val));
    msg.Data = [velocity_L,velocity_R];
    % send messages to the robot
    send(pub,msg);
end

% Stop the robot
msg = rosmessage(pub);
msg.Data = [0 0];
send(pub,msg);
clear pub;

end
