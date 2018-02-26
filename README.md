#PCL Locate bug fix
sudo updatedb
locate PCLConfig
copy the directory containing that file
run 
catkin_make -DPCL_DIR=/usr/lib/x86_64-linux-gnu/cmake/pcl/
