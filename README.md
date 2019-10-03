# Aerial-Image-Stitcher
An algorithm built to stitch drone aerial imagery. The program runs locally on iOS and is built off of the OpenCV framework. It produces accurately geo-referenced maps. 

## Installation of Libraries 
It's recommended to use the latest version of OpenCV (4.1.1) with the contribution modules. You can download it [here](https://pages.github.com/), then add it to `Aerial-Image-Stitcher/QuickMap/OpenCV`

## Downloading Data Sets
The stitcher accepts images at any resolution (it will automatically resize), however there must be GPS encoded in the exif data. It's highly recommended that images are shot 150ft above with minimal distortion and 30-80% overlap. Some good data sets to test: 

• [Sensefly Data Sets](https://www.sensefly.com/education/datasets/?dataset=1503)
• [Dronemapper Data Sets](https://dronemapper.com/sample_data/)



