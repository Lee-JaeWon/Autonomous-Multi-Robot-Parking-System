# Error.md

This page is a summary of potential errors.<br>

## 1.
[OpenCV Error](https://stackoverflow.com/questions/63455427/fatal-error-opencv2-opencv-modules-hpp-no-such-file-or-directory-include-ope) with Astar node.<br>

```
fatal error: opencv.hpp: No such file or directory
   12 | #include <opencv/opencv.hpp>
```
with opencv 4.4.0, Try the command below.
```
sudo cp -r /usr/local/include/opencv4/opencv2 /usr/include/
```