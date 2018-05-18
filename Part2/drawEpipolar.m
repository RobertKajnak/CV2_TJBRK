function drawEpipolar(F,p_base,p_target,im1,im2,title)

    figure('name',title);
    imshow(im1);
    hold on;

    eline = F'*p_target';
    plot(p_base(:,1),p_base(:,2),'O red');
    
    %connect points that are "off screen"
    for i=1:size(p_base,1)
        
        a = -eline(1,i) / eline(2,i);
        
        x = p_base(i,1) - size(im1,2);
        xx = p_base(i,1) + size(im1,2);
        
        y = p_base(i,2) - size(im1,2)*a;
        yy = p_base(i,2) + size(im1,2)*a;

        line([x xx],[y yy]);
    end
    hold off
    
    figure('name',title);
    imshow(im1);
    hold on;

    eline = F*p_base';
    plot(p_target(:,1),p_target(:,2),'O red');
    
    for i=1:size(p_target,1)
        
        a = -eline(1,i) / eline(2,i);
        
        x = p_target(i,1) - size(im2,2);
        xx = p_target(i,1) + size(im2,2);
        
        y = p_target(i,2) - size(im2,2)*a;
        yy = p_target(i,2) + size(im2,2)*a;

        line([x xx],[y yy])
    end
    hold off


end