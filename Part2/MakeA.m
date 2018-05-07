function [A] = MakeA(p,pi)
    n = size(p,1);
    if (size(p)~=size(pi))
        error('Interest points sizes do not match');
    end
    
    x1 = p(:,1);
    x2 = pi(:,1);
    y1 = p(:,2);
    y2 = pi(:,2);
    A = [x1.*x2, x1.*y2 x1 y1.*x2 y1.*y2 y1 x2 y2 ones(n,1)];
    
end
