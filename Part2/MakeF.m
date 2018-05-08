function [F] = MakeF(A)

    [~,S,V] = svd(A);

    %TODO is this math part correct?

    [~,ind]=min(S(S~=0));

    Fc = V(:,ind);

    F = reshape(Fc,[3 3]);

    [Uf,Df,Vf] = svd(F);

    %[~,ind] = min(Df(Df~=0));

    %Df(ind,ind)=0;

    %Df==diag(r,s,t), where r>=s>=t => min_ind(Df)==3 
    Df(3,3) = 0;
    
    F = Uf*Df*Vf';

end