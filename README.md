# FPFH
This code has beeen made to compute FPFH descriptor. 
Before running this code, the user must install the PCL librairy (version 1.8.1) and its dependencies (OpenNI, Boost, Eigen, FLANN, Qhull, and VTK). 
Then, the user must link the PCL library to his Visual Studio 2015 FPFH project. To do so, the user must define a visual project with the fpfh cpp file, 
then, right click on "properties", "C/C++" , "Additional Include Directories" --> add the paths to the include directory for PCL and its dependencies. 
Then right click on "properties", "Linker", "Additional Librairies Directories" --> add the paths to PCL and its dependencies lib folder. 

This code uses PCL functions to compute the FPFH descriptors (https://pcl.readthedocs.io/projects/tutorials/en/latest/fpfh_estimation.html). <br \>
It takes at least 2 input parameter (and an optional third one). These inputs are  : <br />
- the keypoints point cloud filename <br />
- the original point cloud (keypoints + other surface points) <br />
- the voxel size for subsampling (optional)

It will produce a 2-fomat (binary and text) fpfh descriptor file. Those 2 files will be written at the same location as the keypoints file and will be named after the keypoints file with a ".bin" or a ".txt" extension. For the .txt file, the "fpfh" suffix is added just before the extenion. <br \>
For example, if our keypoints filename is "path_to_keyponints'/"filename"+".ply" , then the code will produce : <\ br>
- a fpfh descriptor binary file :  "path_to_keyponints'/"filename"+".bin" 
- the same file in txt format : "path_to_keyponints'/"filename"+ "_fpfh"+".txt" 




