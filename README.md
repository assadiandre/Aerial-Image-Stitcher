# Aerial-Image-Stitcher
An algorithm built to stitch drone aerial imagery. The program runs locally on iOS (written in Swift, Objective C++) and is built off of the OpenCV framework. It produces accurately geo-referenced maps. 

## Installation of Libraries 
It's recommended to use the latest version of OpenCV (4.1.1) with the contribution modules. You can download it [here](https://www.dropbox.com/s/m8yxpqen3m1m4m5/opencv2.framework.zip?dl=0). Alternatively, you can build it yourself, however the former is much easier. Once downloaded, add it to `Aerial-Image-Stitcher/QuickMap/OpenCV`

## Downloading Data Sets
The stitcher accepts images at any resolution (it will automatically resize), however there must be GPS encoded in the exif data. It's highly recommended that images are shot 150ft above with minimal distortion and 30-80% overlap. Some good data sets to test: 

- [Sensefly Data Sets](https://www.sensefly.com/education/datasets/?dataset=1503)
- [Dronemapper Data Sets](https://dronemapper.com/sample_data/)
- [Custom DJI Data Sets](https://google.com)

Once downloaded, add the JPG image folder to `Aerial-Image-Stitcher/Images`. In `StitcherViewController.swift` specify the dataset path, then build. 

## Sources: 
The stitcher was heavily inspired by [A Real-time Stitching Algorithm for UAV Aerial Images](https://www.atlantis-press.com/proceedings/iccsee-13/4836) by Peng Xiong, Xianpeng Liu, Chao Gao, Zan Zhou, Chunxiao Gao, and Qiongxin Liu. Multiple image referencing or "predicted region matching" is a key technique integrated in the algorithm that increases  geographical accuracy. 





