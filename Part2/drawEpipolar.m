function drawEpipolar(F,p,im,title)

    x0=0;
    x1=size(im,2);
    
    %ax+by+c=0 => y=-ax/b-c/b;
    y=@(x,eq)-eq(1,:).*x./eq(2,:)-eq(3,:)./eq(2,:);
    
    %eight-point
    eq = F*p';
    y0 = y(x0,eq);
    y1 = y(x1,eq);
    
    figure('name',title);
    imshow(im);
    hold on;
    for i=1:size(p,1)
        plot(p(i,1),p(i,2),'o', 'MarkerFaceColor', 'g','color','g');
        plot([x0,x1],[y0(i),y1(i)],'g');
    end
    hold off;



end