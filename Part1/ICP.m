function [R,t] = ICP(A1,A2,psi)
%   ICP - 
%   A1, A2 - point cloud, dimension N*3
%   psi - transformation function
    if nargin==0
        A1=load('data/source.mat');
        A1=A1.source';
        A2=load('data/target.mat');
        A2=A2.target';
    elseif nargin <3 
        psi = @(x,R,t) m * x -t; 
    end
    
    [n,d] = size(A1);
    
    R=eye(3);
    t=[0 0 0]';
    RMSold = 0;
    RMS = 1;
    k=1;
    while abs(RMSold-RMS)>1e-2
        fprintf('Attempt %d\n',k);
        k=k+1;
        RMSold = RMS;
        P = A1;
        Q = zeros(size(A2));
        e = inf;
        ind = 0;
        % TODO already matched points should be removed from A2
        %find best matches from A2 for transfomation from A1
        
        fprintf('Step 2: ');
        used = zeros(1,n);
        for i=1:n
            if mod(i*10,n)==0
                fprintf('%d%% ',i/n);
            end
            for j=1:n
                diff = norm(A2(j,:)'-R*A1(i,:)' + t);
                if (diff<e) %&& (used(i)==0)
                    %used(i)=1;
                    e = diff;
                    ind = j;
                end
            end
            Q(i,:) = A2(ind,:);
        end
        fprintf('\n');

        % Mininimization target
        Mt=0;
        for i=1:n
            Mt = Mt + norm(R*P(i)+t-Q(i))^2;
        end
        pbar = sum(P)'/n;
        qbar = sum(Q)'/n;

        X=P-pbar';
        Y=Q-qbar';

        %Step 3
        S = X'*Y;

        %Step 4;
        [U,~,V]=svd(S);
        Matrix = eye(d);
        Matrix(d,d) = det(V*U');
        R = V*Matrix*U';
        t = qbar - R*pbar;

        diffSum = 0;
        fprintf('Step 4: ');
        for i=1:n
            if mod(i*10,n)==0
                fprintf('%d%% ',i/n);
            end
           diffSum =diffSum + norm(Q(i,:)'-R*P(i,:)' + t).^2;
        end
        fprintf('\n');
        
        RMS=sqrt(diffSum/n)
    end
end