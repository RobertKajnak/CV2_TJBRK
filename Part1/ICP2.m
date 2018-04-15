function [R,t] = ICP2(pc1,pc2,samples,sampling,max_repeats,rms)

    %TODO different kinds of sampling, uniform(?) below?
    if (sampling == 0)
        ran = randsample(1:size(pc1),samples);
        pc1 = pc1(ran,:);
        ran = randsample(1:size(pc2),samples);
        pc2 = pc2(ran,:);
    elseif (sampling ==1)
        %.....
    end
        %.....


    [n,d] = size(pc1);
    %TODO add the weights W
    
    R=eye(3);
    t=[0 0 0];
    RMSold = 0;
    RMS = inf;
    k=0;
    

    while abs(RMSold-RMS)>rms && k<max_repeats
        
        k=k+1;
        RMSold = RMS;
        
        P = pc1;
        Q = zeros(size(pc1));

        
        %find best matches from A2 for transfomation from A1
        for i=1:n
            e = inf;
            ind = 0;
            for j=1:n
                diff = norm( (R*pc1(i,:)' + t') - pc2(j,:)');
                if (diff<e) 
                    e = diff;
                    ind = j;
                end
            end
            Q(i,:) = pc2(ind,:);
        end 
        
        p_bar = mean(P);
        q_bar = mean(Q);

        X = P - p_bar;
        Y = Q - q_bar;

        %Step 3
        %multiplication should be the correct way round
        S = X'*Y;

        %Step 4;
        [U,~,V]=svd(S);
        Matrix = eye(d);
        Matrix(d,d) = det(V*U');
        R = V*Matrix*U';

        t = q_bar' - R*p_bar';

        %calculate RMS
        diffSum = 0;
        for i=1:n
            P(i,:) = (R*P(i,:)' + t)';
            diffSum =diffSum + norm( P(i,:)' - Q(i,:)'  )^2;
        end
        RMS=sqrt(diffSum/n);
        fprintf('  %f  ',RMS);

    end
    fprintf(' \n');
    
    
end