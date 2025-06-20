# modified_lio_sam
this project based on lio-sam, and modified several items:
1. gps input. now can easily accept raw gns LLA data, and use geography lib to trans data;
2. bag data input. now can use bag.open method to input data, avoid using bag play method;
3. change save strategy, output is more available for users;
4. change some params and method, not significant, just noticed.

notice:
if CMake Error at CMakeLists.txt:1:
  Parse error.  Expected a command name, got unquoted argument with text
  "/opt/ros/noetic/share/catkin/cmake/toplevel.cmake".
  
cd src && try 'iconv -f GBK -t UTF-8 CMakeLists.txt >CMakeLists.txt'
