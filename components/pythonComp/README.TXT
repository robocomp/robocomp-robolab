1) Install libv4l-dev

2) Checkout AprilTags
svn co https://svn.csail.mit.edu/apriltags

3) cd apriltags 

4) compile  (don't run cmake)
If it does not finish due to OpenCV error its OK as long as libapriltags.a is generated.
make

5) Install
sudo cp -r pod-build/include/* /usr/local/lib
sudo cp -r pod-build/lib/* /usr/local/include


