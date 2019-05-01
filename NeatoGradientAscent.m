% x = linspace(-3,1,100);
% y = linspace(-3,1,100);
% 
% [X,Y] = meshgrid(x,y);
% 
% Z = X.*Y-X.^2-Y.^2-2*X-2*Y+4;
% 
% clf
% contour(X,Y,Z,50)

% a = findGradient(result,0,0)

clf

h = fcontour(result);

h.LevelStep=0.2;

axis([-.25 1.25 -.25 2.25])
hold on

title('Contour Plot of Function')
xlabel('x')
ylabel('y')

position = [0; 0];
L0 = 0.05;

current_angle = -pi/2;




%Configure ROS
% pub = rospublisher('/raw_vel');
% msg = rosmessage(pub);

%Set wheel distance
d = .2413;

% va = 0;
% wa = 1.374/4;
% 
% vl = va-wa*d/2;
% vr = va+wa*d/2;
% 
% %Send datapoints
% msg.Data = [vl,vr];
% send(pub, msg);
% 
% %Wait to change to next velocities
% pause(4)
% 
% msg.Data = [0, 0];
% send(pub, msg);
% 
% pause(.25)




while 1
    
    
    old_position = position;
    old_angle = current_angle;
    
    current_gradient = double(findGradient(result,position(1),position(2)));
    
    if norm(current_gradient) > 100
        break
    end
    
    delta = -1./norm(current_gradient);
    
    delta
    
    l = L0 .* delta;
    
    position = double(position + l .* current_gradient);
    
    plot(position(1),position(2),'m*')
    
    drawnow
    
    
    
%     distance = L0;
%     current_angle = atan((position(2)-old_position(2))/(position(1)-old_position(1)));
%     
%     angle_change = current_angle - old_angle;
%     
%     wa = .5;
%     
%     vl = -wa*d/2;
%     vr = wa*d/2;
%     msg.Data = [vl,vr];
%     send(pub, msg);
%     pause(angle_change/wa)
%     
%     msg.Data = [0, 0];
%     send(pub, msg);
%     
%     
%     va = .1;
%     
%     vl = va;
%     vr = va;
%     msg.Data = [vl,vr];
%     send(pub, msg);
%     pause(.1524)
%     
%     msg.Data = [0, 0];
%     send(pub, msg);
    
    
    
    
    
    
%     va = .1 ;
%     wa = angle_change/(.07622*2);
%     
%     
%     
%     
%     
%     vl = va-wa*d/2;
%     vr = va+wa*d/2;
% 
% %Send datapoints
%     msg.Data = [vl,vr];
%     send(pub, msg);
% 
% %Wait to change to next velocities
%     pause(.0762*2)
end

% msg.Data = [0, 0];
% send(pub, msg);











function grad = findGradient(f,c,d)

syms a b X Y
symgrad = gradient(f);

grad = double(subs(symgrad,[X,Y],[c d]));





end