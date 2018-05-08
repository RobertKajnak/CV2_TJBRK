function [pi_hat,T] = normalizedPi(pi)
%NORMALIZEDPI Summary of this function goes here
%   Detailed explanation goes here
    n= size(pi,1);
    xs = pi(:,1);
    ys = pi(:,2);
    mx = sum(xs)/n;
    my = sum(ys)/n;
    d = sum(sqrt((xs-mx).^2+(ys-my).^2))/n;

    s2d = sqrt(2)/d;
    T = [s2d 0 -mx*s2d;0 s2d -my*s2d;0 0 1];

    pi_hat = (T*pi')';
end

