# Extra Dataset
To create an additional dataset for testing the VO algorithm, and Android
phone with a wide angle camera lens was used to record a new dataset.

# Extracting Frames from Video
`ffmpeg` was used to extract image frames at a desired frame rate from the
source video. During the extraction the frames were also resized to 50% to
reduce computational effort of the VO algorithm as well as storage
requirements.

`ffmpeg -i 20190103_152351.mp4 -r 15 -vf scale=960:-1 testimg%6d.png`

# Calibration
A video sequence was collected with the Matlab calibration pattern,
and images were extracted from it using ffmpeg. Images were extracted from
the video sequence on the phone, because the phone changes the viewing angle
when videos are recorded compared to taking raw images, and the image frame 
size will be different when just taking an image. Calibration images were
then resized to 50% scale (960x540) using a script that utilzes the
ImageMagick tools to automatically process all images in the folder.
Calibration was then performed using the Camera Calibrator app in Matlab from
the Computer Vision toolbox.

Maximum error in reprojected pixels is less than 0.6, and mean reprojection
error in pixels is 0.23

# Dewarping Images
After calibration, the extracted camera paramters were used to preprocess and
dewarp the images. By dewarping the images in the dataset, just the camera
intrinsic parameters can be used, instead of also adding the complexity of
compensating for the barrel distortion in the pipeline. This simplifies the
processing and modifications, since the other 3 datasets just use an intrinsics
matrix. Video frames are also scaled to 50% before applying the dewarping so
that the correct intrinsics are applied, and the processing occurs much faster.

# Data Set
[https://polybox.ethz.ch/index.php/s/4rvexuxYj4ZIymS](https://polybox.ethz.ch/index.php/s/4rvexuxYj4ZIymS)
