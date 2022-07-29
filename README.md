# FPFH
This code has beeen made to compute FPFH descriptor. 
Before running this code, the user must install the PCL librairy (version 1.8.1) and its dependencies (OpenNI, Boost, Eigen, FLANN, Qhull, and VTK). 
Then, the user must link the PCL library to his Visual Studio 2015 FPFH project. To do so, the user must define a visual project with the fpfh cpp file, 
then, right click on "properties", "C/C++" , "Additional Include Directories" --> add the paths to the include directory for PCL and its dependencies. 
Then right click on "properties", "Linker", "Additional Librairies Directories" --> add the paths to PCL and its dependencies lib folder

