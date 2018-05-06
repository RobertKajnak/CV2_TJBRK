function [scores, matches,f1,f2,d1,d2]= keypoint_matching(image1,image2)
    % KEYPOINT_MATCHING Calculates the matching feature points of IMAGE1
    % and IMAGE2 based on David Lovwe's SIFT. The VLFeat module is required to
    % run this function. 
    %
    % The input images can be RGB or uint8. In other cases a conversion
    % will be attemted.
    % 
    % For the structure of [f1,f2,d1,d2] see vl_sift
    % For [scores, matches] see vl_ubmatch
    % See also VL_SIFT, VL_UBCMATCH
    
    image1 = tosingle(image1);
    image2 = tosingle(image2);
    
    [f1,d1] = vl_sift(image1);
    [f2,d2] = vl_sift(image2);
    
    [matches,scores] = vl_ubcmatch(d1, d2);
    
    %vl_sift requires uint8 images, so a cast is attempted
    function [converted_image] = tosingle(image)

        if size(image,3)==3
            converted_image = single(rgb2gray(image));
        elseif isa(image(1,1),'uint8')
            converted_image = single(image);
        end
    end
end