%% Experimental computations
load('gaultlet1.mat') % load the dataset created by the wheel encoders
Pos_step_L = zeros(370,1); % Initializing a matrix of zeroes to receive the encoder differentiated PosX values
Pos_step_R = zeros(370,1); % Initializing a matrix of zeroes to receive the encoder differentiated PosY values
Time_step = zeros(370,1); % Initializing a matrix of zeroes to receive the encoder differentiated time values
Loc = zeros(370,1); % Initializing a matrix of zeroes to receive the encoder location values
time = dataset(:,1); % Assigning the time encoded data from the recorded dataset
Pos_L = dataset(:,2); % Assigning the PosX encoded data from the recorded dataset
Pos_R = dataset(:,3); % Assigning the PosY encoded data from the recorded dataset

for w = 2:370 % iterate through each row of the matrices defined above
    
    Time_step(w,:) = time(w,:) - time(w-1,:); % Differentiating time to obtain the time steps
    Pos_step_L(w,:) = Pos_L(w,:) - Pos_L(w-1,:); % Differentiating posX to obtain the time steps
    Pos_step_R(w,:) = Pos_R(w,:) - Pos_R(w-1,:); % Differentiating posY to obtain the time steps
    
end

Vle = Pos_step_L./Time_step; % Computing the experimental left wheel velocity
Vre = Pos_step_R./Time_step; % Computing the experimental right wheel velocity
linspeede = (Vre+Vle)/(0.254); % Computing the experimental linear speed using the left and right wheel velocities
AngVe = (Vre-Vle)/(0.0254); % Computing the experimental angular velocity using the left and right wheel velocities

for h = 1:370 % iterate through each row of the matrices defined above
    
    Loc(h,:) = Time_step(h,:)*linspeede(h,:); % Computes location of the NEATO based on the experimental linear speed
    
end


%% Robot’s actual left and right wheel velocities

figure(1)
% for n = 1:length(u)% loop through each of the points
    plot(time-33,Vle,'b') % plot the left wheel experimental velocity
    title('Left wheel velocity (m/s)')
    xlabel('time(s)') 
    ylabel('velocity(m/s)')
    legend('Left Experimental')
    
figure(2)
    
    plot(time-25,Vre, 'r') % plot the right wheel experimental velocity
    %quiver3(r(1,n),r(2,n),r(3,n),Bhat(1,n),Bhat(2,n),Bhat(3,n),'g'), hold off % plot the unit binormal
    title('Right wheel velocity (m/s)')
    xlabel('time(s)') 
    ylabel('velocity(m/s)')
    legend('Right Experimental')
% end

%% Robot’s actual linear speed and angular

linspeede = (Vre+Vle)/(0.254); % Computing the experimental linear speed using the left and right wheel velocities
AngVe = (Vre-Vle)/(0.0254); % Computing the experimental angular velocity using the left and right wheel velocities
figure(3)
%for n = 1:length(u)% loop through each of the points
    plot(time-31,linspeede, 'b') % plot the experimental linear speed
    title('Linear Speed (m/s)')
    xlabel('time(s)') 
    ylabel('velocity(m/s)')
    legend('Linear Speed Experimental')
    
    figure(4)
    plot(time-23,AngVe, 'r') % plot the experimental angular velocity vector
    
    title('Angular velocity vector (rad/s)')
    xlabel('time(s)') 
    ylabel('angular velocity(rad/s)')
    legend('Angular Velocity Experimental')
%end