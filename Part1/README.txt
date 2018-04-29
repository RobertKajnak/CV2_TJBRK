The entry points of the program for testing the frame merger is merge2.m. It should be called as merge2()
At the beginning of the merge2() function, all desired parameters and parameter combinations can be specified for which to run both approaches 3.1 and 3.2 with. The results will be saved in files accordingly and can be retrieved by the load(....mat) function.

The results can be visualised by running plot_res(model) of the desired model pointcloud.
To verify our results, we suggest the following settings:

%choose desired parameters to run ICP on
%bruteforce or knn
method = 'bruteforce';
verbose = 1;
%cutoff RMS delta between 2 iterations
rms = 0.0001;
%skip in-between frames
sp = [1];
%number of samples
sa = [50000];
%number of max ICP iterations unless rms is unchanged
it = [30];
%choose sampling method
sampling = 'uniform';

Note that method='knn' would be the method of choice in all cases, but only if the optional dependencies have been installed


The entry point for testing ICP algorithm can be found in ICP_test.m, which can be run as a script without parameters. The type variable on the 12. line changes the way the algorithm works between the default setups sine, sine with noise and real dataset (detailed in the lines proceeding it). The specific details of the test can be modified using the variables 'samples', 'iter' etc. found on lines 15-22. For a complete explanation about the variables use 'help ICP2' or read the documenation section within that file.
All the tables and figures can be reproduced using the values specified in their description, exluding the pixel normal value distribution and the reduced shape of the body (from improvements/ICP section), which are not user facing and are not normally visible. They can be, however reproduced using breakpoints in method getUsefulIndicesSorted, line 364 and lines 246 (using the plotting sequence found in ICP_test.m) respectively.