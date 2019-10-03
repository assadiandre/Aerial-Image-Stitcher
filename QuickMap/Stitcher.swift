//
//  Stitcher.swift
//  Quick Map
//
//  Created by Andre on 7/11/19.
//  Copyright © 2019 3d Robotics. All rights reserved.
//

import Foundation
import MapKit

public class Stitcher {
    
    private let openCVStitcher: CVStitcher
    private var index: Int32
    private var numberOfImages: Int
    private var allGPS: [CGPoint]
    private var stitcherRunning: Bool
    public typealias StitcherCornerPoints = (topLeft: CLLocationCoordinate2D, bottomLeft: CLLocationCoordinate2D, bottomRight: CLLocationCoordinate2D, topRight: CLLocationCoordinate2D)
    
    public init() {
        self.openCVStitcher = CVStitcher()
        self.index = 0
        self.numberOfImages = 0
        self.stitcherRunning = true
        self.allGPS = []
    }
    deinit {
        openCVStitcher.deconstruct()
    }
    
    public func setNumberOfImages(numberOfImages: Int) {
        self.numberOfImages = numberOfImages
    }

    public func getStitcherRunning() -> Bool {
        return stitcherRunning
    }
    
    public func setStitcherRunning(value: Bool) {
        stitcherRunning = value
    }
    
    public func deconstruct() {
        self.openCVStitcher.deconstruct()
    }
    
    public func getGPSCorners() -> StitcherCornerPoints? {
        if isGPSEnabled() {
            guard let corners = self.openCVStitcher.getMapPointCorners(allGPS) as? [CGPoint], corners.count == 4 else {
                return nil
            }
            let topLeftCoord = MKMapPoint(x: Double( corners[0].x ),y: Double( corners[0].y) ).coordinate
            let topRightCoord = MKMapPoint(x: Double( corners[1].x ),y: Double( corners[1].y) ).coordinate
            let botLeftCoord = MKMapPoint(x: Double( corners[2].x ),y: Double( corners[2].y) ).coordinate
            let botRightCoord = MKMapPoint(x: Double( corners[3].x ),y: Double( corners[3].y) ).coordinate
            let coordinates = (
                topLeft: topLeftCoord,
                bottomLeft: botLeftCoord,
                bottomRight: botRightCoord,
                topRight: topRightCoord )
            return coordinates
        }
        return nil
    }
    
    public func isGPSEnabled() -> Bool {
        return allGPS.count > 2
    }
    
    public func feed(image: UIImage, location: CLLocationCoordinate2D) {
        if stitcherRunning {
            let mapPoint = MKMapPoint(location)
            allGPS.append(CGPoint(x: mapPoint.x, y: mapPoint.y))
            let feedSuccessful = openCVStitcher.feed(image, with: index, withGPS: CGPoint(x: mapPoint.x, y: mapPoint.y))
            stitcherRunning = feedSuccessful
            index += 1
        }
    }
    
    public func getStoredImage() -> UIImage {
        return openCVStitcher.createMap()
    }
    
    public func stopProcess() {
        openCVStitcher.stopStitching()
        stitcherRunning = false
        index = 0
        numberOfImages = 0
        allGPS = []
    }
    
    public func getPercentEstimation() -> Double {
        if (numberOfImages != 0) {
            let firstHalf = ( Double( index ) / (Double(numberOfImages )) * 65 )
            let secondHalf = ( Double(openCVStitcher.getProcessStatus()) / 2) * 35
            return Double( round(firstHalf + secondHalf) / 100 )
        }
        return 0
    }
    
    private func convertToMapCoords(coordinates:[CLLocationCoordinate2D] ) -> [CGPoint] {
        var allCGPoints:[CGPoint] = []
        for coordinate in coordinates {
            let mapCoord = MKMapPoint(coordinate)
            allCGPoints.append(CGPoint(x:mapCoord.x, y:mapCoord.y))
        }
        return allCGPoints
    }
}
