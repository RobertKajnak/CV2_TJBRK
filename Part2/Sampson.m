function [d] = Sampson(F,p_base,p_target)

    Fpi = (F*p_base').^2;
    FTpi_prime = (F'*p_target').^2;
    
    d = ones(size(p_base,1),1);
    for i = 1:size(p_base,1)
        d(i,1) = (p_target(i,:)*F*p_base(i,:)').^2 /(Fpi(1,i) +  Fpi(2,i) +  FTpi_prime(1,i) + FTpi_prime(2,i)  );
    end
    
    
end

