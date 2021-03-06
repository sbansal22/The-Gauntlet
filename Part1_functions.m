%% RANSAC 
% Cleaning the data

data = load('r_and_theta.mat');

[M,B,DOMAIN] = RANSAC(data);

function [M,B,DOMAIN] = RANSAC(data)
clf

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
    
    
    
    x_1 = linspace(DOMAIN(1,i),DOMAIN(2,i));
    y_1 = M(i)*x_1 + B(i);
    plot(x_1,y_1,'lineWidth',5)
end



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





plot(median(leftoverCoordinates(1,:)),median(leftoverCoordinates(2,:)),'*','lineWidth',20)


title('Map of Gauntlet with RANSAC walls')
xlabel('X (Meters)')
ylabel('Y (Meters)')
legend('Included Points','Wall 1','Wall 2','Remaining Points','Gauntlet Perimeter','Bucket')
hold off

end

function [ M, B, domain, remainingCoordinates] = ransac(coordinates)

% defining the line using the two point form of a line

% initialize khat


counter = 0;
coordinates = [coordinates ;ones(length(coordinates))];
coordinates = coordinates(1:3,:);
goodPoints = [];
goodMatrix = [zeros(length(coordinates(1,:)),100)];
coefficientsMatrix = zeros(3,100);


for i=1:100
    
    rand1 = randi(length(coordinates(1,:)),1);
    rand2 = randi(length(coordinates(1,:)),1);
    

    if rand1 == rand2
        rand2 = randi(length(coordinates(1,:)),1);
    end

    x = coordinates(1,:);
    y = coordinates(2,:);

    x1 = x(rand1);
    x2 = x(rand2);
    y1 = y(rand1);
    y2 = y(rand2); 
    
    Khat = [0 0 1];
    Tvector = [x2-x1 ; y2-y1 ; 1];
    That = Tvector./vecnorm(Tvector);
    Nhat = cross(Khat, That);
    
    coefficientsMatrix(:,i) = [x2-x1; y2-y1; 1];
    
    clear goodPoints 
    goodPoints = [];
    counter = 0;
    
    for m=1:length(coordinates(1,:))
        point = coordinates(:,m);
        pvector = [x1 - point(1) ; y1 - point(2); 1];
        perp_dist = dot(Nhat,pvector);
        parr_dist = dot(That,pvector);
        if abs(perp_dist) < 0.001 && abs(parr_dist) < 1
            counter = counter + 1;
            goodPoints(length(goodPoints) + 1 )= m;

            
            
        end
    end
    goodMatrix (:,i) = [goodPoints zeros(1,length(coordinates(1,:))-length(goodPoints))];
end

for k = length(coordinates(1,:)):-1:1
    if any(goodMatrix(k,:))
        [~,index] = max(goodMatrix(k,:));

        bestCoefficients = coefficientsMatrix(:,index);
        break
    end 
end

bestLine = goodMatrix(:,index);
bestLine = bestLine(bestLine ~= 0);

includedCoordinates = zeros(3,length(bestLine));
for i = 1:length(bestLine)
    includedCoordinates(:,length(includedCoordinates)+1) = coordinates(:,bestLine(i));
    coordinates(:,bestLine(i)) = 0;
    
end
coordinates( :, ~any(coordinates,1) ) = [];  %columns
includedCoordinates( :, ~any(includedCoordinates,1) ) = [];  %columns
hold on

[~,M,B] = regression(includedCoordinates(1,:),includedCoordinates(2,:));

remainingCoordinates = coordinates;

%domain = [min(includedCoordinates(1,:)) max(includedCoordinates(1,:))];

standardDev = std(includedCoordinates(1,:));

domainCenter = mean(includedCoordinates(1,:));

domain = [domainCenter - standardDev domainCenter + standardDev];



% domain = [prctile(domain,20) prctile(domain,80)];

% x_fit = linspace(domain(1),domain(2));
% y_fit = M * x_fit + B;

%plot(x_fit,y_fit,'c')
end
    
         