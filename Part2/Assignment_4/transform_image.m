function [new_image,nc]=transform_image(image,m,t)
    % TRANSFORM_IMAGE Transforms IMAGE based on the transformation
    % paramters M and T
    %
    % m - rotation matrix
    % t - translation matrix
    % nc - Coordinate of new image's corners
    
    %convert images so floating point operations can be done
    image = double(image);
   
    sz = size(image);
    
    %% Calculate the position of the edges of the transformed image
    % the inverse transform is used to calculate the edges after
    %transformation to the new coordinate system
    mi = m^-1;
    function [xy]=cinv(x,y)
        xy = mi*([x;y]-t);
        %xy = [xy(2);xy(1)];
    end
    %corner positions
    nc = floor([cinv(1,1),cinv(1,sz(2)),cinv(sz(1),1),cinv(sz(1),sz(2))]);
    
    %new size based on corners
    minx = min(nc(1,:));
    maxx = max(nc(1,:));
    miny = min(nc(2,:));
    maxy = max(nc(2,:));
    nsz = [maxx-minx,maxy-miny, size(image,3)];
    
    %calculate offset to center image
    offx = minx;
    offy = miny;
    offset=[offx;offy];
    
    %% Transform image
    new_image=uint8(zeros(nsz));
    for i=offx:nsz(1)+offx
        for j=offy:nsz(2)+offy
            xy=m*[i;j]+t;
            xy=round(xy);
            if xy(1)>0 && xy(1)<sz(1) && xy(2)>0 && xy(2)<sz(2)
               new_image(i-offx+1, j-offy+1, :)=uint8(image(xy(1),xy(2), :));
            end
        end
    end
    nc=nc+offset;
end

    