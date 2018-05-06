boat1 = imread('boat1.pgm');
boat2 = imread('boat2.pgm');
% figure;
% imshow(boat1);
% figure;
% imshow(boat2);
[scores, matches,f1,f2,d1,d2] = keypoint_matching(boat2, boat1);
[bt] = RANSAC(boat2, boat1, matches, f1, f2, 10, 50, 0);

%W,H
%the first image is assumed to be in perfect landscape
offset=floor(bt.m*[1;1]+bt.t);
base = boat2;
rotated = transform_image(base,bt.m,bt.t);

%calculate the new size for the image
nsz = round([max(offset(2)+size(rotated,2),size(base,2))-min(offset(2),0),...
       max(offset(1)+size(rotated,1),size(base,1))-min(offset(1),0)]);
nsz = [nsz(2),nsz(1)];

%overlay the two images
im = zeros(nsz);
figure;
im=overlay(im,boat1,-offset.*(offset<0));
im=overlay(im,rotated,offset.*(offset>0));
imshow(im,[]);

%TODO If you know a matlab function for this one, feel free to replace it. 
%The one I found was kinda complicated
function [new_image]=overlay(image1,image2,offset)
    new_image = image1;
    for i=1:size(image2,1)
        for j=1:size(image2,2)
            if image2(i,j)~=0
                new_image(i+offset(1),j+offset(2))=image2(i,j);
            end
        end
    end
end