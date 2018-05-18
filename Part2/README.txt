The entry point to the program is Main.m  
This also runs the sift install code.  
Section "Debug/partial result flags" at lines 9-16 contains three flag type variables to display plots. 
	showSift displays the matches found between the image pairs (useful to visually check for outliers)  
	showEpipolar displays the epiploar lines found by using the three different fundamental matrices
	stopAfterFirstIteration stops the algorithm after the first iteration is complete: enable this if one or both of the prefious flags is set, so that only the first pair is show, and the reader is not overwhelmed by plots.
	examplePVM when set to 'true' will use the PVM from the supplied example file instead of the one calculated in this assignment
	nr_consec_imgs specifies the number of pointclouds that should be stitched together during the (5) structure from motion section.

  
To obtain some of the plots, variables would have to be modified manually, as these are not intended to be user facing: the user should not deal with the bad configurations.
	matches = vl_ubcmatch(d1, d2,5); -- remove '5' to undo "sift built-in filtering" as mentioned in the report
	[p_base, p_target] = InterestPoints(f1,f2,matches,30,showSift,im1,im2); -- 30 can be changed to -1 to disable 2D RANSAC
	