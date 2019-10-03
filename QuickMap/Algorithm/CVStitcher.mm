//  CVStitcher.mm
//  Quick Map
//
//  Created by Andre on 6/27/19.
//  Copyright © 2019 3DRobotics. All rights reserved.
//
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/detail/util.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgcodecs/ios.h"

#import "CVStitcher.h"
#import <UIKit/UIKit.h>
#import <opencv2/xfeatures2d.hpp>
#import <opencv2/xfeatures2d/nonfree.hpp>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace cv::detail;


struct ImageReturn
{
    Mat stored_image_mask;
    Mat stored_image;
    Point2f reference_point;
};

struct DistanceAccumulator {
    vector<double> all;
    void add(double distX, double distY) {
        double totalDist = sqrt( (distX * distX) + (distY * distY) );
        all.push_back( totalDist );
    }
    bool avgExists() {
        if (all.size() == 0) {
            return false;
        }
        return true;
    }
    double getMedianDistance() {
        if (avgExists()) {
            sort(all.begin(), all.end());
            return all[ floor( (all.size() - 1)/2 )];
        }
        return 0;
    }
    double calcTotalDist(double distX, double distY) {
        double totalDist = sqrt( (distX * distX) + (distY * distY) );
        return totalDist;
    }
};

struct KeyPointReturn {
    vector<vector<Point2f>> keyPoints;
    Point2f refPoint;
    
    KeyPointReturn(vector<vector<Point2f>> keyPointsParam, Point2f refPointParam)  {
        keyPoints = keyPointsParam;
        refPoint = refPointParam;
    }
};

@implementation CVStitcher

vector<Point2f> allCenterPoints;
vector<cv::Point> allCornerPoints;
vector<cv::Size> allImageSizes;
vector<UMat> allMasks;
vector<UMat> allImages;
vector<Point2f> allStoredGPS;
DistanceAccumulator distanceAccumulator;
vector<vector<KeyPoint>> storedKeyPoints;
vector<Mat> storedDescriptors;
Mat storedImage;
Mat storedMask;
Mat prevTransformation;
Point2f currentReferencePoint;
Mat globalRefImage;
bool stitchCompromised = false;
int processStatus = 0;


-(bool) feed: (UIImage*) image withIndex: (int) index withGPS: (CGPoint) gpsPoint  {
    if (!stitchCompromised) {
        Mat matImage;
        UIImageToMat(image, matImage, true);
        Mat destMat;
        cvtColor(matImage,destMat,COLOR_RGBA2RGB);
        allStoredGPS.push_back( Point2f(gpsPoint.x, gpsPoint.y) );
        
        if (index == 0) {
            storedImage = destMat;
            globalRefImage = destMat;
            currentReferencePoint = Point2f(0,0);
            
            allCornerPoints.push_back(cv::Point(0,0));
            allImages.push_back(destMat.getUMat(ACCESS_READ));
            allMasks.push_back(Mat(cv::Size( storedImage.cols , storedImage.rows ), CV_8UC1, cv::Scalar(255)).getUMat(ACCESS_READ)); //CV_8UC1 = one channel image, 8 bits
            allImageSizes.push_back( storedImage.size() );
            allCenterPoints.push_back(Point2f( destMat.cols/2, destMat.rows/2));
            
            Mat grayImage, descriptors;
            vector<KeyPoint> keypoints;
            cvtColor( storedImage, grayImage, COLOR_RGB2GRAY );
            Ptr<Feature2D> detector = SIFT::create();
            detector->detectAndCompute( grayImage, Mat(), keypoints, descriptors );
            if ( keypoints.size() == 0 ) {
                stitchCompromised = true;
            }
            storedDescriptors.push_back( descriptors );
            storedKeyPoints.push_back( keypoints);
            return true;
            
        } else if (index > 0) {
            Mat H = computeTransformation( globalRefImage,destMat );
            //printf("Image #%d Processed \n", index); // Only for testing
            if (H.empty() || stitchCompromised ) {
                NSLog(@"QUICK MAP -- WARNING: RANSAC COULD NOT COMPUTE TRANSFORMATION \n");
            } else {
                Mat transformedFloatingImage = addToStoredImage(destMat, H );
                globalRefImage = transformedFloatingImage;
            }
            return true;
        }
    }
    return false;
}

-(UIImage*) createMap {
    if (allImages.size() == 0) {
        UIImage* emptyImage = [[UIImage alloc] init];
        NSLog(@"NO IMAGES FOUND");
        return emptyImage;
    }
    
    Ptr<Blender> blender = Blender::createDefault(Blender::MULTI_BAND,false);
    Ptr<SeamFinder> seam_finder = SeamFinder::createDefault(SeamFinder::DP_SEAM);
    seam_finder->find(allImages, allCornerPoints, allMasks);
    addToProcessStatus();
    blender->prepare( allCornerPoints, allImageSizes);
    MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
    mb->setNumBands(5); //Set number of bands to 5 (default value)
    
    for (int i = 0; i < allImages.size(); i++) {
        Mat image_s;
        allImages[i].convertTo(image_s, CV_16SC3); //CV_16SC3 is nessary image configuration for blender. 16 bit, 3 channels.
        blender->feed(image_s, allMasks[i], allCornerPoints[i]);
    }
    
    Mat finalBlendedImage; Mat finalBlendedImageMask;
    blender->blend(finalBlendedImage,finalBlendedImageMask);
    
    Mat threeChannelImage, destMat, finalImage;
    finalBlendedImage.convertTo(threeChannelImage, CV_8UC3); //CV_8UC3 needed to convert RGB to RGBA. 8 bit, 3 channels.
    cvtColor(threeChannelImage, destMat, COLOR_RGB2RGBA);
    finalImage = Mat( finalBlendedImage.size(), CV_8UC4, Scalar(0)); //CV_8UC4, 8 bit, 4 channels. Provides alpha channel.
    destMat.copyTo(finalImage, finalBlendedImageMask);
    
    storedImage = finalImage.clone();
    storedMask = finalBlendedImageMask.clone();
    
    destMat.release();
    finalBlendedImage.release();
    threeChannelImage.release();
    finalImage.release();
    allImages.clear();
    allMasks.clear();
    
    return MatToUIImage( storedImage );
}


-(NSArray*) getMapPointCorners: (NSArray*) mapPoints {
    addToProcessStatus();
    vector<Point2f> imagePointsVector = allCenterPoints;
    vector<Point2f> mapPointsVector;
    
    for( size_t i = 0; i < imagePointsVector.size(); i++ )
    {
        NSValue *value = [mapPoints objectAtIndex:i];
        CGPoint point = [value CGPointValue];
        mapPointsVector.push_back(Point2f(point.x, point.y));
    }
    
    Mat inliers;
    Mat R = estimateAffinePartial2D(imagePointsVector,mapPointsVector , inliers, RANSAC);
    
    if (!R.empty()) {
        Point2f topLeft = Point2f(0,0);
        Point2f topRight = Point2f(storedImage.cols, 0);
        Point2f botLeft = Point2f(0, storedImage.rows);
        Point2f botRight = Point2f(storedImage.cols, storedImage.rows);
        
        double newTopLeftX = (R.at<double>(0,0) *topLeft.x + R.at<double>(0,1)*topLeft.y + R.at<double>(0,2));
        double newTopLeftY = (R.at<double>(1,0) *topLeft.x + R.at<double>(1,1)*topLeft.y + R.at<double>(1,2));
        
        double newTopRightX = (R.at<double>(0,0) *topRight.x + R.at<double>(0,1)*topRight.y + R.at<double>(0,2));
        double newTopRightY = (R.at<double>(1,0) *topRight.x + R.at<double>(1,1)*topRight.y + R.at<double>(1,2));
        
        double newBotLeftX = (R.at<double>(0,0) *botLeft.x + R.at<double>(0,1)*botLeft.y + R.at<double>(0,2));
        double newBotLeftY = (R.at<double>(1,0) *botLeft.x + R.at<double>(1,1)*botLeft.y + R.at<double>(1,2));
        
        double newBotRightX = (R.at<double>(0,0) *botRight.x + R.at<double>(0,1)*botRight.y + R.at<double>(0,2));
        double newBotRightY = (R.at<double>(1,0) *botRight.x + R.at<double>(1,1)*botRight.y + R.at<double>(1,2));
        
        Point2f transformedTopLeft = Point2f(newTopLeftX,newTopLeftY );
        Point2f transformedTopRight = Point2f(newTopRightX,newTopRightY );
        Point2f transformedBotLeft = Point2f(newBotLeftX,newBotLeftY );
        Point2f transformedBotRight = Point2f(newBotRightX,newBotRightY );
        
        NSMutableArray *array = [[NSMutableArray alloc] initWithCapacity:0];
        [array addObject:[NSValue valueWithCGPoint: CGPointMake(transformedTopLeft.x, transformedTopLeft.y)]];
        [array addObject:[NSValue valueWithCGPoint: CGPointMake(transformedTopRight.x, transformedTopRight.y)]];
        [array addObject:[NSValue valueWithCGPoint: CGPointMake(transformedBotLeft.x, transformedBotLeft.y)]];
        [array addObject:[NSValue valueWithCGPoint: CGPointMake(transformedBotRight.x, transformedBotRight.y)]];
        
        return array;
    }
    return nil;
}

-(int) getProcessStatus {
    return processStatus;
}

-(void) stopStitching {
    stitchCompromised = true;
}

void addToProcessStatus() {
    processStatus += 1;
}

-(void) deconstruct {
    allImages.clear();
    allMasks.clear();
    storedKeyPoints.clear();
    allCenterPoints.clear();
    allCornerPoints.clear();
    allImageSizes.clear();
    allStoredGPS.clear();
    distanceAccumulator.all.clear();
    processStatus = 0;
    stitchCompromised = false;
    storedMask.release();
    storedImage.release();
    globalRefImage.release();
    storedDescriptors.clear();
    storedKeyPoints.clear();
}

double calcMedianGPSDist() {
    vector<double> diffs;
    for (int i = 1; i < allStoredGPS.size(); i++) {
        double xDiff = allStoredGPS[i - 1].x - allStoredGPS[i].x;
        double yDiff = allStoredGPS[i - 1].y - allStoredGPS[i].y;
        double totalDiff = sqrt( xDiff * xDiff + yDiff * yDiff );
        diffs.push_back( totalDiff );
    }
    sort(diffs.begin(), diffs.end());
    return diffs[ ceil( diffs.size()/2 ) ];
}

vector<int> getIndicesOfNeighboringImages(double distFactor) {
    double medDist = calcMedianGPSDist();
    vector<int> returnIndices;
    Point2f currentGPS = allStoredGPS[allStoredGPS.size() - 1];
    for (int i = 0; i < allStoredGPS.size() - 1; i++) {
        double xDiff = currentGPS.x - allStoredGPS[i].x;
        double yDiff = currentGPS.y - allStoredGPS[i].y;
        double totalDiff = sqrt( xDiff * xDiff + yDiff * yDiff );
        if (totalDiff < medDist * distFactor) {
            returnIndices.push_back(i);
        }
    }
    return returnIndices;
}

vector<vector<Point2f>> getAdditionalKeyPoints(Mat descriptors_floating, vector<KeyPoint> keypoints_floating, int index) {
    FlannBasedMatcher matcher;
    vector< DMatch > matches;
    Mat descriptors_reference = storedDescriptors[index];
    vector<KeyPoint> keypoints_reference = storedKeyPoints[index];
    
    matcher.match( descriptors_floating, descriptors_reference, matches );
    double max_dist = 0; double min_dist = 100;
    
    for( int i = 0; i < descriptors_floating.rows; i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    
    vector< DMatch > good_matches;
    for( int i = 0; i < descriptors_floating.rows; i++ )
    { if( matches[i].distance <= 3*min_dist )
    { good_matches.push_back( matches[i]); }
    }
    
    vector<Point2f> floatingPoints;
    vector<Point2f> referencePoints;
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        floatingPoints.push_back( keypoints_floating[ good_matches[i].queryIdx ].pt );
        referencePoints.push_back( keypoints_reference[ good_matches[i].trainIdx ].pt );
    }
    
    return vector<vector<Point2f>>{floatingPoints,referencePoints};
}

vector<Point2f> getRefPointsInSet( vector<int> indices, Point2f initialPoint ) {
    vector<Point2f> retval{ initialPoint };
    for (int i = 0; i < indices.size(); i++) {
        Point2f referencePoint =  allCornerPoints[ indices[i]   ];
        retval.push_back(referencePoint);
    }
    return retval;
}

vector<Point2f> adjustCornersInSet(vector<Point2f> cornersInSet, Point2f newRefPoint) {
    for (int i = 0; i < cornersInSet.size(); i++) {
        cornersInSet[i].x -= newRefPoint.x;
        cornersInSet[i].y -= newRefPoint.y;
    }
    return cornersInSet;
}


KeyPointReturn calculateNewKeyPoints( vector<int> indices, vector<Point2f> refPoints, vector<Point2f> floatingPoints, Mat descriptors_floating, vector<KeyPoint> keypoints_floating, bool includeOGRefImage ) {
    vector<Point2f> allReferencePoints, allFloatingKeyPoints, cornersInSet;
    if (!includeOGRefImage && indices.size() > 0) {
        cornersInSet = getRefPointsInSet(indices, allCornerPoints[indices[0]]); // set duplicate corner
        floatingPoints.clear();
        refPoints.clear();
    } else {
        cornersInSet = getRefPointsInSet(indices, currentReferencePoint);
    }
    vector<double> extremities = findExtremities(cornersInSet); // Get xMin, yMin
    Point2f newRefPoint = Point2f(extremities[0], extremities[1]);
    cornersInSet = adjustCornersInSet(cornersInSet,newRefPoint); // Recalculate ref point
    
    for (int i = 0; i < cornersInSet.size(); i++) {
        Point2f referencePoint =  cornersInSet[i];
        vector<vector<Point2f>> extraKeyPoints;
        if (i != 0) {
            extraKeyPoints = getAdditionalKeyPoints(descriptors_floating, keypoints_floating, indices[i - 1]);
        } else {
            extraKeyPoints = {floatingPoints, refPoints};
        }
        vector<Point2f> extraFloatingKeyPoints = extraKeyPoints[0];
        vector<Point2f> extraReferenceKeyPoints = extraKeyPoints[1];
        vector<Point2f> filteredRefKeyPoints;
        vector<Point2f> filteredFloatKeyPoints;
        
        for (int idx = 0; idx < extraFloatingKeyPoints.size(); idx++) {
            Point2f currentKeyPoint = extraReferenceKeyPoints[idx];
            Point2f adjustedKeyPoint = Point2f( currentKeyPoint.x + referencePoint.x, currentKeyPoint.y + referencePoint.y );
            filteredRefKeyPoints.push_back( adjustedKeyPoint );
            filteredFloatKeyPoints.push_back( extraFloatingKeyPoints[idx] );
        }
        
        allReferencePoints.insert( allReferencePoints.end() , filteredRefKeyPoints.begin(), filteredRefKeyPoints.end());
        allFloatingKeyPoints.insert(allFloatingKeyPoints.end(), filteredFloatKeyPoints.begin(),filteredFloatKeyPoints.end() );
    }
    return KeyPointReturn(vector<vector<Point2f>>{ allReferencePoints, allFloatingKeyPoints }, newRefPoint);
}

bool findExtremeTransformation(cv::Size size, Mat transformation) {
    if (!transformation.empty()) {
        vector<Point2f> transformedCorners;
        vector<Point2f> imgCorners { Point2f(0, 0), Point2f(0, size.height), Point2f(size.width, size.height),Point2f(size.width, 0)} ;
        perspectiveTransform(imgCorners,transformedCorners, transformation);
        vector<double> extremities = findExtremities(transformedCorners);
        CGFloat xMin = extremities[0];
        CGFloat yMin = extremities[1];
        CGFloat xMax = extremities[2];
        CGFloat yMax = extremities[3];
        
        Mat img_mask = Mat( size, CV_8UC1, Scalar(255));
        Mat warped_mask;
        Mat trans_mat = (Mat_<double>(3,3) << 1, 0,-1 * xMin, 0, 1, -1 * yMin, 0,0,1);
        Mat translated_H = trans_mat *  transformation;
        warpPerspective( img_mask,warped_mask,translated_H, cv::Size(xMax - xMin, yMax - yMin), INTER_LINEAR, BORDER_CONSTANT );
        vector<vector<cv::Point>> contours;
        findContours(warped_mask, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
        warped_mask.release();
        img_mask.release();
        
        double warpedArea, originalArea;
        if (contours.size() > 0) {
            warpedArea = contourArea(contours[0]);
            originalArea = double( cv::Rect(0,0,size.width,size.height).area() );
        } else {
            return true;
        }
        ///For debugging:
        //printf( "Warped Area: %f \n", warpedArea);
        //printf( "Original Area: %f \n", originalArea);
        
        if ( warpedArea > originalArea * 6 ||  warpedArea < originalArea * 0.2 ) { // Very Extreme Case
            return true;
        }
        return false;
    }
    return true;
}

Mat computeTransformation( Mat const &referenceImage, Mat const &floatingImage ) {
    Mat img_floating;
    Mat img_reference;
    
    cvtColor( floatingImage, img_floating, COLOR_RGB2GRAY );
    cvtColor( referenceImage, img_reference, COLOR_RGB2GRAY );
    
    Ptr<Feature2D> detector = SIFT::create(); // SIFT is most robust, we can also use ORB, SURF, AKAZE, KAZE
    vector<KeyPoint> keypoints_floating, keypoints_reference;
    Mat descriptors_floating, descriptors_reference;
    
    vector<int> neighboringIndices = getIndicesOfNeighboringImages(2.5); // An enigma as if this needs to change or not
    ///For debugging:
    //printf("SIZE OF NEIGHBORING INDICES: %lu \n", neighboringIndices.size() );
    
    if (!storedDescriptors.empty()) {
        detector->detectAndCompute( img_floating, Mat(), keypoints_floating, descriptors_floating );
        ///For debugging:
        //printf("SIZE OF KEYPOINTS: %lu \n", keypoints_floating.size());
        if (keypoints_floating.size() < 300) {
            detector = SIFT::create(0, 3, 0.02);
            detector->detectAndCompute( img_floating, Mat(), keypoints_floating, descriptors_floating );
        }
        keypoints_reference = storedKeyPoints[storedKeyPoints.size() - 1] ;
        descriptors_reference = storedDescriptors[storedDescriptors.size() - 1];
    } else {
        detector->detectAndCompute( img_floating, Mat(), keypoints_floating, descriptors_floating );
        detector->detectAndCompute( img_reference, Mat(), keypoints_reference, descriptors_reference );
    }
    
    if ( keypoints_reference.size() == 0) {
        NSLog(@"QUICK MAP -- WARNING: KEYPOINTS FOR OG REF IMAGE FAILED \n");
    }
    
    //-- Step 2: Matching descriptor vectors using FLANN matchers
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    
    matcher.match( descriptors_floating, descriptors_reference, matches );
    double max_dist = 0; double min_dist = 100;
    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_floating.rows; i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    
    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< DMatch > good_matches;
    for( int i = 0; i < descriptors_floating.rows; i++ )
    { if( matches[i].distance <= 3*min_dist )
    { good_matches.push_back( matches[i]); }
    }
    
    //-- Localize the object
    std::vector<Point2f> floatingPoints;
    std::vector<Point2f> referencePoints;
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        floatingPoints.push_back( keypoints_floating[ good_matches[i].queryIdx ].pt );
        referencePoints.push_back( keypoints_reference[ good_matches[i].trainIdx ].pt );
    }
    ///For debugging:
    //printf("SIZE OF ACTUAL POINTS: %lu \n", floatingPoints.size());
    vector<Point2f> finalReferencePoints, finalFloatingPoints;
    KeyPointReturn newKeyPoints = calculateNewKeyPoints( neighboringIndices, referencePoints, floatingPoints, descriptors_floating, keypoints_floating, true );
    finalReferencePoints = newKeyPoints.keyPoints[0];
    finalFloatingPoints = newKeyPoints.keyPoints[1];
    
    
    if (finalFloatingPoints.size() == 0) { // If we can't find common matches between reference image & neighboring return previous transformation
        NSLog(@"QUICK MAP -- ERROR: NO GOOD MATCHES FOUND \n");
        return getTransformationMatrixFromGPS();
    }
    Mat inliers;
    Mat R = estimateAffine2D(finalFloatingPoints ,finalReferencePoints , inliers, RANSAC); // RANSAC is most robust least squars algorithm. LMEDS is also an option.
    Mat H = convertAffineToHomography(R);
    
    if ( findExtremeTransformation(floatingImage.size(), H) || H.empty() ) { // First level just compares image with previous one
        ///For debugging:
        //printf("EXTREME TRANFORMATION \n");
        finalReferencePoints = referencePoints;
        finalFloatingPoints = floatingPoints;
        R = estimateAffine2D(finalFloatingPoints ,finalReferencePoints , inliers, RANSAC);
        H = convertAffineToHomography(R);
        if ( findExtremeTransformation(floatingImage.size(), H) || H.empty() ) { // Second level matches all nearest neighbors except the current reference image
            ///For debugging:
            //printf("SECOND LEVEL EXTREME TRANSFORMATION \n");
            KeyPointReturn newKeyPoints = calculateNewKeyPoints( neighboringIndices, referencePoints, floatingPoints, descriptors_floating, keypoints_floating, false );
            finalReferencePoints = newKeyPoints.keyPoints[0];
            finalFloatingPoints = newKeyPoints.keyPoints[1];
            currentReferencePoint = newKeyPoints.refPoint;
            R = estimateAffine2D(finalFloatingPoints ,finalReferencePoints , inliers, RANSAC);
            H = convertAffineToHomography(R);
            if (  (findExtremeTransformation(floatingImage.size(), H) || H.empty() ) && !prevTransformation.empty() ) { // If both previous attempts fail, fill in with previous transformation
                NSLog(@"QUICK MAP -- WARNING: GPS TRANSFORMATION \n");
                H = getTransformationMatrixFromGPS();
            } else {
                H = Mat();
            }
        }
    } else {
        currentReferencePoint = newKeyPoints.refPoint;
    }
    
    storedKeyPoints.push_back(keypoints_floating);
    storedDescriptors.push_back(descriptors_floating);
    return H;
}

Mat convertAffineToHomography(Mat R) {
    if (!R.empty()) {
        Mat H = Mat(3,3,R.type());
        H.at<double>(0,0) = R.at<double>(0,0);
        H.at<double>(0,1) = R.at<double>(0,1);
        H.at<double>(0,2) = R.at<double>(0,2);
        
        H.at<double>(1,0) = R.at<double>(1,0);
        H.at<double>(1,1) = R.at<double>(1,1);
        H.at<double>(1,2) = R.at<double>(1,2);
        
        H.at<double>(2,0) = 0.0;
        H.at<double>(2,1) = 0.0;
        H.at<double>(2,2) = 1.0;
        return H;
    }
    return Mat();
}


Mat getTransformationMatrixFromGPS() {
    vector<Point2f> imagePointsVector = allCenterPoints;
    vector<Point2f> mapPointsVector;
    for (size_t i = 0; i < imagePointsVector.size(); i++) {
        mapPointsVector.push_back(Point2f(allStoredGPS[i].x, allStoredGPS[i].y));
    }
    if ( mapPointsVector.size() < 2) {
        return Mat();
    }
    Mat inliers;
    Mat R = estimateAffine2D(mapPointsVector, imagePointsVector,inliers, RANSAC);
    Mat H = convertAffineToHomography(R);
    if ( H.empty() ) {
        return Mat();
    }
    vector<Point2f> imageCenterPointsFromGPS;
    vector<Point2f> mostRecentCenterPoints {
        allStoredGPS[allStoredGPS.size() - 1],
        allStoredGPS[allStoredGPS.size() - 2] }; // Will never throw NullPointer exception
    perspectiveTransform(mostRecentCenterPoints, imageCenterPointsFromGPS, H);
    
    double tx = imageCenterPointsFromGPS[0].x - imageCenterPointsFromGPS[1].x;
    double ty = imageCenterPointsFromGPS[0].y - imageCenterPointsFromGPS[1].y;
    
    Mat returnH = prevTransformation;
    returnH.at<double>(0,2) = tx;
    returnH.at<double>(1,2) = ty;
    return returnH;
}

Mat addToStoredImage(Mat const &floatingImage, Mat H) {
    CGFloat floatImgWidth = floatingImage.cols;
    CGFloat floatImgHeight = floatingImage.rows;
    
    vector<Point2f> transformedCorners;
    vector<Point2f> floatImgCorners{Point2f(0, 0),Point2f(0, floatImgHeight),Point2f(floatImgWidth, floatImgHeight),Point2f(floatImgWidth, 0) };
    perspectiveTransform(floatImgCorners,transformedCorners, H);
    vector<double> extremities = findExtremities(transformedCorners);
    CGFloat xMin = extremities[0];
    CGFloat yMin = extremities[1];
    CGFloat xMax = extremities[2];
    CGFloat yMax = extremities[3];
    
    Point2f topLeftReferenceCorner = transformedCorners[0];
    
    int xOffset = topLeftReferenceCorner.x - xMin; // Refers to the displacement of the top left corner (of the warped image) relative to the new image canvas
    int yOffset = topLeftReferenceCorner.y - yMin;

    Mat warpedFloatingImage;
    Mat floatingImg_trans_mat = (Mat_<double>(3,3) << 1, 0,-1 * xMin, 0, 1, -1 * yMin, 0,0,1);
    Mat translated_H = floatingImg_trans_mat *  H;
    warpPerspective( floatingImage,warpedFloatingImage,translated_H, cv::Size(xMax - xMin, yMax - yMin), INTER_LINEAR, BORDER_REFLECT );

    Mat floatingMask = cv::Mat(floatingImage.size(), CV_8UC1, cv::Scalar(255));
    Mat warpedFloatingMask;
    warpPerspective(floatingMask , warpedFloatingMask, translated_H,  cv::Size(xMax - xMin, yMax - yMin),INTER_LINEAR, BORDER_CONSTANT  );
    
    Point2f centerPoint = Point2f(floatImgWidth/2, floatImgHeight/2);
    double tz = 1 / (translated_H.at<double>(2,0) * centerPoint.x + translated_H.at<double>(2,1) * centerPoint.y + translated_H.at<double>(2,2)  );
    double centerTX = (translated_H.at<double>(0,0) *centerPoint.x + translated_H.at<double>(0,1)*centerPoint.y + translated_H.at<double>(0,2)) * tz;
    double centerTY = (translated_H.at<double>(1,0) *centerPoint.x + translated_H.at<double>(1,1)*centerPoint.y + translated_H.at<double>(1,2)) * tz;
    
    // Adjust transformation matrix
    H.at<double>(0,2) -= (xOffset);
    H.at<double>(0,5) -= (yOffset);
    double changeX = H.at<double>(0,2);
    double changeY = H.at<double>(0,5);
    
    if ( distanceAccumulator.calcTotalDist(changeX, changeY) < distanceAccumulator.getMedianDistance() * 4 || !distanceAccumulator.avgExists() ) {
        distanceAccumulator.add(changeX, changeY);
        prevTransformation = H;
    } else {
        NSLog(@"QUICK MAP -- WARNING: THIS POINT MIGHT BE INCORRECT \n");
        stitchCompromised = true;
    }
    
    updateReferenceKeyPoints(translated_H);
    
    if (!stitchCompromised) {
        currentReferencePoint = mergeImages(H , currentReferencePoint, storedImage, warpedFloatingImage, warpedFloatingMask);
        allCenterPoints.push_back(Point2f(currentReferencePoint.x + centerTX, currentReferencePoint.y + centerTY));
    }
    return warpedFloatingImage;
}

void updateReferenceKeyPoints(Mat transformation) {
    vector<Point2f> points;
    vector<float> sizes;
    vector<KeyPoint> referenceKeyPoints = storedKeyPoints[storedKeyPoints.size() - 1];
    
    for (int i = 0; i < referenceKeyPoints.size(); i++) {
        points.push_back(storedKeyPoints[storedKeyPoints.size() - 1][i].pt);
        sizes.push_back( storedKeyPoints[storedKeyPoints.size() - 1][i].size);
    }
    vector<Point2f> newPoints;
    vector<KeyPoint> newKeyPoints;
    perspectiveTransform(points, newPoints, transformation);
    for (int i = 0; i < points.size(); i++) {
        newKeyPoints.push_back( KeyPoint(newPoints[i], sizes[i]) );
    }
    storedKeyPoints[storedKeyPoints.size() - 1] = newKeyPoints;
}

void updateAllCenterCoords(double updateX, double updateY) {
    if (allCenterPoints.size() > 0) {
        for (int i = 0; i < allCenterPoints.size(); i++) {
            allCenterPoints[i].x += updateX;
            allCenterPoints[i].y += updateY;
        }
    }
}

void updateAllCornerCoords(double updateX, double updateY) {
    if (allCornerPoints.size() > 0) {
        for (int i = 0; i < allCornerPoints.size(); i++) {
            allCornerPoints[i].x += updateX;
            allCornerPoints[i].y += updateY;
        }
    }
}


Point2f mergeImages(Mat H, Point2f referencePoint, Mat const &stored_image, Mat const &warped_floating_image, Mat const &warped_floating_mask ){
    CGFloat fullXExtent, fullYExtent;
    CGFloat changeX = H.at<double>(0,2);
    CGFloat changeY = H.at<double>(0,5);
    CGFloat total_addX = 0; // Default value for additional space is zero
    CGFloat total_addY = 0;
    
    // Find whether the translation will require a larger image or not
    if ( changeX > 0) { // If our x translation is positive
        fullXExtent = referencePoint.x + changeX + warped_floating_image.cols;
        total_addX = ceil( std::fmax(0, fullXExtent - (stored_image.cols)) );
    } else { // Gets more complicated for a negative x translation
        fullXExtent = referencePoint.x + changeX + warped_floating_image.cols;
        if (referencePoint.x + changeX < 0) {
            total_addX += abs(referencePoint.x + changeX);
        }
    }
    
    if ( changeY < 0 ) { // If y translation is negative
        fullYExtent = referencePoint.y + changeY + warped_floating_image.rows;
        if (referencePoint.y + changeY < 0) {
            total_addY += abs(referencePoint.y + changeY);
        }
    } else { // If it's positive
        fullYExtent = referencePoint.y + changeY + warped_floating_image.rows;
        total_addY = ceil( std::fmax(0, fullYExtent - stored_image.rows ) ) ;
    }
    
    // New point of reference adjusted to Mat coordinate system for next image
    if (total_addX > 0 && changeX < 0) {
        referencePoint.x = 0;
        updateAllCenterCoords(abs(total_addX),0 );
        updateAllCornerCoords(abs(total_addX),0 );
    } else if (changeX < 0 || changeX > 0) {
        referencePoint.x += changeX;
    }
    
    if (total_addY > 0 && changeY < 0) {
        referencePoint.y = 0;
        updateAllCenterCoords(0,abs(total_addY) );
        updateAllCornerCoords(0,abs(total_addY) );
    } else if ( changeY < 0 || changeY > 0) {
        referencePoint.y += changeY;
    }
    
    allImages.push_back(warped_floating_image.getUMat(ACCESS_READ));
    allMasks.push_back(warped_floating_mask.getUMat(ACCESS_READ));
    allCornerPoints.push_back( cv::Point(referencePoint.x, referencePoint.y) );
    allImageSizes.push_back(warped_floating_image.size());
    
    return referencePoint;
}


vector<double> findExtremities(vector<Point2f> corners) {
    CGFloat xMin = corners[0].x;
    CGFloat yMin = corners[0].y;
    CGFloat xMax = corners[0].x;
    CGFloat yMax = corners[0].y;
    for (int i = 0; i < corners.size(); i++) {
        if (xMax <  corners[i].x) {
            xMax = corners[i].x;
        }
        if (xMin > corners[i].x) {
            xMin = corners[i].x;
        }
        if (yMax < corners[i].y) {
            yMax = corners[i].y;
        }
        if (yMin > corners[i].y) {
            yMin = corners[i].y;
        }
    }
    return {xMin,yMin,xMax,yMax};
}
@end

