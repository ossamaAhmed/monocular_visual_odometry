# Visual Odometry
Mini project for Vision Algorithms for Mobile Robots 2018.
Implementation of a working, simple, monocular visual odometry (VO) pipeline with the following implementation details:
  * Harris Corner Feature Detector
  * 5 Point Algorithm for initial bootstrapping
  * KLT Tracking of feature points across frames
  * Triangulation of new landmarks
  * Local pose refinement through optimization
  * Release of a custom grocery store dataset

## Authors:
Jordan **Burklund**,
Jose **Vasquez**,
Ossama **Ahmed**,
Yilun **Wu**

## Development Environment
MATLAB R2018b with Computer Vision Toolbox, Image Processing Toolbox, Statistics Toolbox, Optimization Toolbox

## How to run
Download [KITTI, Malaga, Parking](http://rpg.ifi.uzh.ch/teaching.html) dataset and copy them under `data` folder and run `main.m` to start the pipeline!

## Results
Results benchmarked on a Macbook Pro **3.1 GHz i5-7267U, 16GB RAM** with MATLAB R2018b

KITTI Dataset (~2 frames/sec)

<a href="https://www.youtube.com/watch?v=dyNT3g425sU
" target="_blank"><img src="http://img.youtube.com/vi/EHmbAUY3yuI/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

MALAGA Dataset (~1.6 frames/sec)

<a href="http://www.youtube.com/watch?feature=player_embedded&v=YNKbG5N_DZs
" target="_blank"><img src="http://img.youtube.com/vi/YNKbG5N_DZs/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

Parking Dataset (~1.4 frames/sec)

<a href="http://www.youtube.com/watch?feature=player_embedded&v=SgB9KkR0CwI
" target="_blank"><img src="http://img.youtube.com/vi/SgB9KkR0CwI/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

Grocery Store Dataset (~1.5 frames/sec)

<a href="https://youtu.be/Blju6blWjoA
" target="_blank"><img src="http://img.youtube.com/vi/LsCGowc6WXI/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

# Additional Data Set
Image frames for the grocery store dataset can be found at:
[https://polybox.ethz.ch/index.php/s/4rvexuxYj4ZIymS](https://polybox.ethz.ch/index.php/s/4rvexuxYj4ZIymS)

Extract the `grocery` folder from the compressed file, and put it in the `/data` folder with the other datasets.
To test with the grocery store dataset, set `ds=3`.
