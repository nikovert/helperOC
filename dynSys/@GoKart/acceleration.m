function [a, v] = acceleration(obj, x)
% acc = acceleration(x)
%   returns the acceleration along x, towards the origin
%   terrain function:
    v = sin(3*pi*x).*(x - x.*cos(2*pi*x) + (3*x.^2)/10 - x.^3/5);
    a = sin(3*pi*x).*((3*x)/5 - cos(2*pi*x) - (3*x.^2)/5 + 2*x.*pi.*sin(2*pi*x) + 1) + 3*pi*cos(3*pi*x).*(x - x.*cos(2*pi*x) + (3*x.^2)/10 - x.^3/5);
    a = obj.damp * a;
end

