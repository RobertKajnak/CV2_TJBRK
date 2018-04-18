% function [R,t] = ICP(A1,A2,psi)
% %   ICP - 
% %   A1, A2 - point cloud, dimension N*3
% %   psi - transformation function
%     if nargin==0
        A1=load('data/source.mat');
        A1=A1.source';
        A2=load('data/target.mat');
        A2=A2.target';
%     elseif nargin <3 
%         psi = @(x,R,t) m * x -t; 
%     end
    
    [n,d] = size(A1);
    %TODO add the weights W
    
    R=eye(3);
    t=[0 0 0]';
    RMSold = 0;
    RMS = 1;
    k=1;
    figure;
    %fscatter3(A1(:,1),A1(:,2),A1(:,3),1:size(A1,1));
    while abs(RMSold-RMS)>1e-2 && k<5
        fprintf('Attempt %d\n',k);
        k=k+1;
        RMSold = RMS;
        P = A1;
        Q = zeros(size(A2));
        e = inf;
        ind = 0;
        
        %find best matches from A2 for transfomation from A1
        tic
        fprintf('Step 2: ');
        used = zeros(1,n);
        for i=1:n
            e=inf;
            ind=0;
            if mod(i*10,n)==0
                fprintf('%d%% ',i*100/n);
            end
            for j=1:n
                diff = norm(A2(j,:)'-R*A1(i,:)' - t);
                if (diff<e) %&& (used(i)==0)
                    used(i)=1;
                    e = diff;
                    ind = j;
                end
            end
            Q(i,:) = A2(ind,:);
        end
        fprintf('; Finished in: %.2f seconds\n',toc);
        
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
        tic
        fprintf('Step 4: ');
        for i=1:n
            if mod(i*10,n)==0
                fprintf('%d%% ',i*100/n);
            end
            P(i,:) = R*P(i,:)' + t;
            diffSum =diffSum + norm(Q(i,:)'-P(i,:)').^2;
        end
        fprintf('; Finished in: %.2f seconds\n',toc);
        
        RMS=sqrt(diffSum/n)
    end
    hold on;
    fscatter3([A1(:,1);P(:,1);A2(:,1)],[A1(:,2);P(:,2);A2(:,2)],[A1(:,3);P(:,3);A2(:,3)],1:n*3);
%end