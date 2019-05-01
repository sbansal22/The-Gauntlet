m = M(1);
b = B(1);

domain = DOMAIN(:,1);


result = -createLine(m,b,domain ) / 100;

m = M(2);
b = B(2);

domain = DOMAIN(:,2);


result = result + -createLine(m,b,domain ) / 100;

% result = result + createLine(3,0,domain);

result = result + 5*createBucket(1,1.75);





fsurf(result)
% 
axis([-.25 1.25 -.25 2.25 -10 10])
% axis([-10 10 -10 10 -10 10])



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



