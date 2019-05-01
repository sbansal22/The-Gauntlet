data = load('r_and_theta.mat'); %loading the static scan data of the gauntlet using the LIDAR

[M,B,DOMAIN, circle] = RANSAC(data); %runs the RANSAC algorithm to obtain the required line features, line domain, and the bucket shape

result = createGradient(M,B,DOMAIN, circle); 

h = fcontour(result); %plots the contour lines for a surface

h.LevelStep=0.2; %Sets the steps for slope levels on the plot

axis([-.25 1.25 -.25 2.25])
hold on
title('Contour Plot of Function')
xlabel('x')
ylabel('y')

position = [0; 0]; %Initializes the position of the NEATO in the global frame of reference
L0 = 0.05;

current_angle = -pi/2; %Initializes the heading of the NEATO in the global frame of reference

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
    
    if norm(current_gradient) > 100 % Checks if the slope exceeds 100 units
        break
    end
    
    delta = -1./norm(current_gradient);
    

    
    l = L0 .* delta;
    
    position = double(position + l .* current_gradient); % computes the new position based on a version of the euler's algorithm
    
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





%{

FUNCTIONS:

Functions to do all required actions in the gradient ascent algorithm


%}





function grad = findGradient(f,c,d)

syms a b X Y
symgrad = gradient(f); %computes the gradient matrix of the function f

grad = double(subs(symgrad,[X,Y],[c d])); 





end

function [result] = createGradient (M,B,DOMAIN, circle)


m = M(1); % Assigning the slope of the line for the first object
b = B(1); % Assigning the intercept of the line for the first object

domain = DOMAIN(:,1); % Assings the range in x for the first object

scale_factor1 = 100; % Assigns a scaling factor for the first object
result = -createLine(m,b,domain ) / scale_factor1; % computes the line and scales it by the factor 

m = M(2); % Assigning the slope of the line for the second object
b = B(2); % Assigning the intercept of the line for the second object

domain = DOMAIN(:,2); % Assings the range in x for the second object 

scale_factor2 = 100; % Assigns a scaling factor for the second object
result = result + -createLine(m,b,domain ) / scale_factor2; % computes the line and scales it by the factor 

% result = result + createLine(3,0,domain);

result = result + 5*createBucket(circle(1),circle(2));

fsurf(result) % creates a surface plot of the objects and the bucket

axis([-.25 1.25 -.25 2.25 -10 10])

figure
end

function [result] = createBucket (x_position,y_position)
    
    syms X Y

    result = log(sqrt((X - x_position)^2+(Y - y_position)^2));
end



function [result] = createLine (m,b,domain)

    x = linspace(domain(1),domain(2));
    syms X Y
    result = 0;

    for i = 1:100
        result = result + log((sqrt((X-x(i))^2+(Y-(m*x(i)+b))^2)));
    end
end


function [M,B,DOMAIN, circle] = RANSAC(data)


% load('transposedDatasets.mat')

c = 1;
r = data.r_all(:,1);
theta = data.theta_all(:,1);
for i=1:(length(r))
    if r(i,:) ~= 0
    data.r_clean(c,:) = r(i,:);
    data.theta_clean(c,:) = theta(i,:);
    c = c+1;
    end
end

% Computing the cartesian coordinates 
coordinates(1,:) = data.r_clean .* cos(deg2rad(data.theta_clean));
coordinates(2,:) = data.r_clean .* sin(deg2rad(data.theta_clean));

coordinates(:,coordinates(1,:) > 1.25) = [];
coordinates(:,coordinates(1,:) < 0) = [];
coordinates(:,coordinates(2,:) < 0) = [];
coordinates(:,coordinates(2,:)  > 2) = [];

leftoverCoordinates = coordinates;
hold on
plot(coordinates(1,:),coordinates(2,:),'*b')
% while  length(leftoverCoordinates)>6
for i = 1:2
    
    [M(i),B(i),DOMAIN(:,i),leftoverCoordinates] = ransac(leftoverCoordinates);
    
    x_1 = linspace(DOMAIN(1),DOMAIN(2));
    y_1 = M(i)*x_1 + B(i);
    plot(x_1,y_1,'lineWidth',5)
end

legend('1','2','3','4','5','6','7','8')

% [a,b,c,d] = ransac(coordinates);
% [e,f,g,h] = ransac(d);
% [i,j,k,l] = ransac(h);
% 
% x_1 = linspace(c(1),c(2));
% x_2 = linspace(g(1),g(2));
% x_3 = linspace(k(1),k(2));
% 
% y_1 = a*x_1 + b;
% y_2 = e*x_2 + f;
% y_3 = i*x_3 + j;
% 
% plot(x_1,y_1)
% plot(x_2,y_2)
% plot(x_3,y_3)

plot(leftoverCoordinates(1,:),leftoverCoordinates(2,:),'*r')

axis([-.5 2 -.5 2.5])

walls_x = [-.25 1.6 1.6 -.25 -.25];
walls_y = [-.25 -.25 2.25 2.25 -.25];

plot(walls_x,walls_y,'m','lineWidth',5)

circle = [median(leftoverCoordinates(1,:)) median(leftoverCoordinates(2,:))];

plot(median(leftoverCoordinates(1,:)),median(leftoverCoordinates(2,:)),'*','lineWidth',20)


hold off

figure

end



function [ M, B, domain, remainingCoordinates] = ransac(coordinates)

% defining the line using the two point form of a line

% initialize khat

counter = 0;
coordinates = [coordinates ;ones(length(coordinates))];
coordinates = coordinates(1:3,:);
goodPoints = []; % initializes a matrix to add the 'inliers' from RANSAC
goodMatrix = zeros(length(coordinates(1,:)),100); % initializes the matrix to add the compelete line for an object
coefficientsMatrix = zeros(3,100);

for i=1:100
    
    rand1 = randi(length(coordinates(1,:)),1); % picks a random integer for the first RANSAC argument
    rand2 = randi(length(coordinates(1,:)),1); % picks a random integer for the second RANSAC argument
 
    if rand1 == rand2
        rand2 = randi(length(coordinates(1,:)),1); % Tries to make sure that the first and the second arguments are not the same
    end

    x = coordinates(1,:); 
    y = coordinates(2,:);

    x1 = x(rand1); % assigns the first x argument using the random index
    x2 = x(rand2); % assigns the second x argument using the random index
    y1 = y(rand1); % assigns the first y argument using the random index
    y2 = y(rand2); % assigns the second y argument using the random index
    
    Khat = [0 0 1]; % initializes a vector 
    Tvector = [x2-x1 ; y2-y1 ; 1]; % defines the line as a vector
    That = Tvector./vecnorm(Tvector); % defines a unit vector tangent to the line
    Nhat = cross(Khat, That); % defines the normal vector as the cross product of K direction and T direction
    
    coefficientsMatrix(:,i) = [x2-x1; y2-y1; 1];
    
    clear goodPoints 
    goodPoints = [];
    counter = 0;
    
    for k=1:length(coordinates(1,:))
        point = coordinates(:,k); % picks a point on the line
        pvector = [x1 - point(1) ; y1 - point(2); 1]; % computes the position vector
        perp_dist = dot(Nhat,pvector); % dot product to compute the magnitude of the perpendicular component
        parr_dist = dot(That,pvector); % dot product to compute the magnitude of the parallel component
        if abs(perp_dist) < 0.001 && abs(parr_dist) < 1 % RANSAC thresholds
            counter = counter + 1;
            goodPoints(length(goodPoints) + 1 )= k;
        end
    end
    goodMatrix (:,i) = [goodPoints zeros(1,length(coordinates(1,:))-length(goodPoints))]; % records the inliers
end

for k = length(coordinates(1,:)):-1:1 % indexes in from end to start
    if any(goodMatrix(k,:)) % Checks for nonzero elements
        [~,index] = max(goodMatrix(k,:)); % iterates through good matrix to find the column with maximum inliers

        bestCoefficients = coefficientsMatrix(:,index); % stores the points for line of best fit
        break
    end 
end

bestLine = goodMatrix(:,index);
bestLine = bestLine(bestLine ~= 0); % removes any remaining null points

includedCoordinates = zeros(3,length(bestLine));
for i = 1:length(bestLine)
    includedCoordinates(:,length(includedCoordinates)+1) = coordinates(:,bestLine(i));
    coordinates(:,bestLine(i)) = 0;
    
end
coordinates( :, ~any(coordinates,1) ) = [];  %columns
includedCoordinates( :, ~any(includedCoordinates,1) ) = [];  %columns
hold on

[~,M,B] = regression(includedCoordinates(1,:),includedCoordinates(2,:)); % performes regression on RANSAC's output (points) to fit a usable line to it.

remainingCoordinates = coordinates;

%domain = [min(includedCoordinates(1,:)) max(includedCoordinates(1,:))];

standardDev = std(includedCoordinates(1,:));

domainCenter = mean(includedCoordinates(1,:));

domain = [domainCenter - standardDev domainCenter + standardDev]; % defines a domain for the function based on the mean and standard deviation



% domain = [prctile(domain,20) prctile(domain,80)];

% x_fit = linspace(domain(1),domain(2));
% y_fit = M * x_fit + B;

%plot(x_fit,y_fit,'c')
end