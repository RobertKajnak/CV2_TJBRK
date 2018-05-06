image1 = 'left.jpg';
image2 = 'right.jpg';

%% Display original images
figure('name','Original right and left images');
subplot(1,2,1);
imshow(image1);
title('left.jpg');
subplot(1,2,2);
imshow(image2);
title('right.jpg');

%% Display stitched images
figure('name','Aligning left.jpg with right.jpg');
stitched = stitch(image2,image1);
imshow(stitched);

figure('name','Aligning right.jpg with left.jpg');
stitched = stitch(image1,image2);
imshow(stitched);