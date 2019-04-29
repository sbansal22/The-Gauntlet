%% RANSAC 
% Cleaning the data
clear
load('scan4.mat')
c = 1;
for i=1:(length(r))
    if r(i,:) ~= 0
    r_clean(c,:) = r(i,:);
    theta_clean(c,:) = theta(i,:);
    c = c+1;
    end
end

% Computing the cartesian coordinates 
coordinates(1,:) = r_clean .* cos(deg2rad(theta_clean));
coordinates(2,:) = r_clean .* sin(deg2rad(theta_clean));

% test plot
% plot(x,y,'o')

% thresholds for x and y
threshold = 0.25*(max(coordinates(1,:)));

upper_x_threshold = 1.25*(max(coordinates(1,:)));
upper_y_threshold = 1.25*(max(coordinates(2,:)));
lower_x_threshold = 0.75*(min(coordinates(1,:)));
lower_y_threshold = 0.75*(min(coordinates(2,:)));

% defining the random points
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

% defining the line using the two point form of a line
x_plot = 0:2;
y_plot = ((y2-y1)/(x2-x1))*(x_plot-x1) + y1;

figure(1)
plot(x_plot,y_plot,'--'), hold on

plot(x,y,'*r')
xval = [x1, x2];
yval = [y1, y2];
plot(xval, yval,'ob'), hold on

m = polyfit(xval,yval, 1);
x_fit = linspace(x1, x2, 50);
y_fit = m(1) * x_fit + m(2);

plot(x_fit, y_fit,'-.')

% initialize khat
Khat = [0 0 1];
Tvector = [x2-x1 ; y2-y1 ; 1];
That = Tvector./vecnorm(Tvector);
Nhat = cross(Khat, That);

counter = 0;
coordinates = [coordinates ;ones(length(coordinates))];
coordinates = coordinates(1:3,:);
goodPoints = [];
goodMatrix = [zeros(length(coordinates(1,:)),10)];


for i=1:10
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
    
    clear goodPoints 
    goodPoints = [];
    counter = 0;
    
    for m=1:length(coordinates(1,:))
        point = coordinates(:,m);
        pvector = [x1 - point(1) ; y1 - point(2); 1];
        perp_dist = dot(Nhat,pvector);
        if abs(perp_dist) < 0.01
            counter = counter + 1;
            goodPoints(length(goodPoints) + 1 )= m;
            
        end
        
    end
    goodMatrix (:,i) = [goodPoints zeros(1,length(coordinates(1,:))-length(goodPoints))];
end


for k = length(coordinates(1,:)):-1:1
    if any(goodMatrix(k,:))
        [~,index] = max(goodMatrix(k,:));
        break
        
    end
    
end

bestLine = goodMatrix(:,index);

bestLine = bestLine(bestLine ~= 0);

for i = 1:length(bestLine)
    plot(coordinates(1,bestLine(i)),coordinates(2,bestLine(i)),'*')
    
    
end


hold off
% 
% max_counter = 0;
% for k=1:10
%     max_counter = zeros(1,10);
%     for j=1:97
%         if goodMatrix(j,k) ~= 0
%             max_counter(1,k) = max_counter(1,k) + 1;
%         end
%     end
% end
            
         