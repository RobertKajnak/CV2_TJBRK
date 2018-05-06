function [im] = stitch(im1_str, im2_str)
image1 = imread(im1_str);
image2 = imread(im2_str);

%% Calculate the bets match parameters
[scores, matches,f1,f2,d1,d2] = keypoint_matching(image2, image1);
[bt] = RANSAC(image2, image1, matches, f1, f2, 20, 3);

%% Calculate necessary offsets
%get width and height of second image
w = size(image2, 2);
h = size(image2, 1);

%calculate where those corners land
%the first image is assumed to be in perfect landscape
offsets=[bt.m*[1;1]+bt.t, bt.m*[1;h]+bt.t, ...
    bt.m*[w;1]+bt.t, bt.m*[w;h]+bt.t];
offset(1) = ceil(min(offsets(1,:)));
offset(2) = ceil(min(offsets(2,:)));

offset_image1(1) = -min(offset(1), 0);
offset_image1(2) = -min(offset(2), 0);
offset_rotated(1) = max(offset(1), 0);
offset_rotated(2) = max(offset(2), 0);

%% Rotate the image
base = image2;
rotated = transform_image(base,bt.m,bt.t);

%% calculate the new size for the image
nsz = round([max(offset(2)+size(base,1),size(rotated,1))-min(offset(2),0),...
       max(offset(1)+size(base,2),size(rotated,2))-min(offset(1),0)]);
nsz = [nsz(1),nsz(2), size(image1,3)];

%% Overlay the two images
im = zeros(nsz);

im = overlay(im, image1, offset_image1);
im = overlay(im, rotated, offset_rotated);
im = uint8(im);
end

%% Helper function to overlay the two images
function [new_image]=overlay(image1,image2,offset)
    new_image = image1;
    for i=1:size(image2,1)
        for j=1:size(image2,2)
            if image2(i,j)~=0
                new_image(i+offset(2),j+offset(1), :)=image2(i,j, :);
            end
        end
    end
end