//
//  StitcherUtils.swift
//  Quick Map
//
//  Created by Andre on 7/3/19.
//  Copyright © 2019 3d Robotics. All rights reserved.
//

import Foundation
import MobileCoreServices
import MapKit

extension UIImage {
    func resize(targetSize: CGSize) -> UIImage {
        let newSize:CGSize = CGSize(width: ceil( targetSize.width / 2 ), height: ceil( targetSize.height / 2 ))
        return UIGraphicsImageRenderer(size:newSize).image { _ in
            self.draw(in: CGRect(origin: .zero, size: newSize))
        }
    }
}

class StitcherUtils {
    
    static func getPreferredImageSize(imagePath:String, preferredWidth:CGFloat) -> CGSize? {
        /** All images are stored in the Images folder, and are loaded through this method  */
        if let path = Bundle.main.resourcePath {
            let imagePath = path + imagePath
            let url = URL(fileURLWithPath: imagePath)
            let fileManager = FileManager.default
            let properties = [URLResourceKey.localizedNameKey,URLResourceKey.creationDateKey, URLResourceKey.localizedTypeDescriptionKey]
            do {
                let imageURLs = try fileManager.contentsOfDirectory(at: url, includingPropertiesForKeys: properties, options:FileManager.DirectoryEnumerationOptions.skipsHiddenFiles)
                let sampleImage = UIImage(contentsOfFile: imageURLs[0].path)!
                let sampleSize = sampleImage.size
                let scaleRatio = preferredWidth / sampleSize.width
                return CGSize(width: scaleRatio * sampleSize.width, height: scaleRatio * sampleSize.height)
            } catch let error  {
                print(error)
            }
        }
        return nil
    }
    
    static func getAllImagePaths(imagePath:String ) -> [String] {
        var allImagePaths:[String] = []
        /** All images are stored in the Images folder, and are loaded through this method  */
        if let path = Bundle.main.resourcePath {
            let imagePath = path + imagePath
            let url = URL(fileURLWithPath: imagePath)
            let fileManager = FileManager.default
            let properties = [URLResourceKey.localizedNameKey,URLResourceKey.creationDateKey, URLResourceKey.localizedTypeDescriptionKey]
            do {
                let imageURLs = try fileManager.contentsOfDirectory(at: url, includingPropertiesForKeys: properties, options:FileManager.DirectoryEnumerationOptions.skipsHiddenFiles)
                
                for i in 0...(imageURLs.count - 1) {
                    
                    allImagePaths.append( imageURLs[i].path )
                    
                }
                allImagePaths = allImagePaths.sorted{$0.localizedStandardCompare($1) == .orderedAscending}
            } catch let error  {
                print(error)
            }
        }
        return allImagePaths
    }
    
    static func getImage(imagePath:String) -> UIImage {
        return UIImage(contentsOfFile: imagePath)!
    }
    
    static func getInitialLocation(imagePath:String) -> CLLocationCoordinate2D {
        var allImagePaths:[String] = []
        var selectedImageURL:URL?
        if let path = Bundle.main.resourcePath {
            let imagePath = path + imagePath
            let url = URL(fileURLWithPath: imagePath)
            let fileManager = FileManager.default
            let properties = [URLResourceKey.localizedNameKey,URLResourceKey.creationDateKey, URLResourceKey.localizedTypeDescriptionKey]
            do {
                let imageURLs = try fileManager.contentsOfDirectory(at: url, includingPropertiesForKeys: properties, options:FileManager.DirectoryEnumerationOptions.skipsHiddenFiles)
                
                for imageURL in imageURLs {
                    allImagePaths.append(imageURL.absoluteString)
                }
                allImagePaths = allImagePaths.sorted{$0.localizedStandardCompare($1) == .orderedAscending}
                
                selectedImageURL = URL(string: allImagePaths[0])
                
            } catch let error  {
                print(error)
            }
        }
        return locationForImage(selectedImageURL!)!
    }
    
    static func locationForImage(_ url: URL) -> CLLocationCoordinate2D? { // Returns location of image
        let createOptions: [String: AnyObject] = [kCGImageSourceTypeIdentifierHint as String: kUTTypeJPEG as String as String as AnyObject]
        guard let source = CGImageSourceCreateWithURL(url as CFURL, createOptions as CFDictionary?) else {
            print("Unable to determine location for image at \(url). Could not create image source.")
            return nil
        }
        let propertyOptions: [String: AnyObject] = [kCGImageSourceShouldCache as String: NSNumber(value: false as Bool)]
        guard let properties = CGImageSourceCopyPropertiesAtIndex(source, 0, propertyOptions as CFDictionary?) as NSDictionary? else {
            print("Unable to determine location for image at \(url). Could not get properties from image.")
            return nil
        }
        guard let gps = properties[kCGImagePropertyGPSDictionary as String] as? NSDictionary,
            let latitudeNumber = gps[kCGImagePropertyGPSLatitude as String] as? NSNumber,
            let latitudeRef = gps[kCGImagePropertyGPSLatitudeRef as String] as? String,
            let longitudeNumber = gps[kCGImagePropertyGPSLongitude as String] as? NSNumber,
            let longitudeRef = gps[kCGImagePropertyGPSLongitudeRef as String] as? String else {
                print("Unable to determine location for image at \(url). Required properties missing or invalid.")
                return nil
        }
        let latitude = latitudeNumber.doubleValue * (latitudeRef == "N" ? 1 : -1)
        let longitude = longitudeNumber.doubleValue * (longitudeRef == "E" ? 1 : -1)
        return CLLocationCoordinate2D(latitude: latitude, longitude: longitude)
    }
}
