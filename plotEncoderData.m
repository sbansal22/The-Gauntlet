load('gaultlet3.mat')

dataset = dataset(1:305,:);

d = .2413;
dl = [0];
dr = [0];

for i=1:150
dl=diff(dataset(:,2))/(32/151);
dr=diff(dataset(:,3))/(32/151);

vactual(i)= (dl(i)+dr(i))/2;
wactual(i)=(dr(i)-dl(i))/d;
    
    
end

dataset(:,2) = dataset(:,2)-dataset(1,2);
dataset(:,3) = dataset(:,3)-dataset(1,3);



x = 0
y = 0
theta = 0

for i=1:length(vactual)
    
    theta(i+1) = theta(i)+wactual(i)*32/125;
    x(i+1) = x(i) + vactual(i)*sin(theta(i+1));
    y(i+1) = y(i) + vactual(i)*cos(theta(i+1))
    
end

plot(x,y)
axis([0 1.5 0 2.5])
