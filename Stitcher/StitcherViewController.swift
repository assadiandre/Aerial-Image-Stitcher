//
//  StitcherViewController.swift
//  Stitcher
//
//  Created by Andre on 7/11/19.
//  Copyright © 2019 3d Robotics. All rights reserved.
//

import UIKit
import MapKit
import QuickMap

class StitcherViewController: UIViewController {

    @IBOutlet weak var infoLabel: UILabel!
    @IBOutlet weak var finalImageView: UIImageView!
    var allGPSPoints:[CGPoint] = []
    var allCoords:[CLLocationCoordinate2D] = []
    var stitcher:Stitcher = Stitcher()
    var numberOfImages:Int = 0
    var programTime:Float = 0
    
    override func viewDidLoad() {
        super.viewDidLoad()
        setupStitcher(path: "/Images/Museo-Ambar-Data-Set", fullDataSet: true, loopStride:(from:1,to:15,by:1), preferredImageWidth:540)
    }
    
    func setupStitcher(path:String, fullDataSet:Bool, loopStride:(from:Int,to:Int,by:Int), preferredImageWidth:CGFloat ) {
        let displayTimer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(self.addToTimer), userInfo: nil, repeats: true)
        guard let imageSize = Utils.getPreferredImageSize(imagePath: path, preferredWidth: preferredImageWidth) else {
            print("ERROR: Couldn't get preferred image size")
            return
        }
        DispatchQueue.global(qos: .background).async {
            let finalImage = self.loadAllImagesAndStitch( imagePath: path, fullDataSet: fullDataSet, loopStride:loopStride, size:imageSize )
            DispatchQueue.main.async {
                displayTimer.invalidate()
                self.finalImageView.image = finalImage
            }
        }
    }
    
    @objc func addToTimer() {
        programTime += 0.1
        self.infoLabel.text = "Time: \(Double(round(programTime))) | Images: \(numberOfImages) | Progress:\(stitcher.getPercentEstimation() * 100)%"
    }
    
    func loadAllImagesAndStitch(imagePath:String, fullDataSet:Bool, loopStride:(from:Int,to:Int,by:Int), size:CGSize  ) -> UIImage {
        var index:Int32 = 0
        var allPaths = Utils.configureImageData(imagePath:imagePath)
        var updatedLoopStride:(from:Int,to:Int,by:Int) = (from:0,to:0,by:0)
        if (fullDataSet) {
            updatedLoopStride = (from:1,to:allPaths.count - 1,by:1)
            numberOfImages = allPaths.count - 2
        } else {
            updatedLoopStride = loopStride
            numberOfImages = Int( (updatedLoopStride.to - updatedLoopStride.from ) / updatedLoopStride.by )
        }
        for i in stride(from: updatedLoopStride.from, to: updatedLoopStride.to, by: updatedLoopStride.by) {
            let coordinate = Utils.locationForImage( URL(fileURLWithPath: allPaths[i] ) )!
            allCoords.append(coordinate)
        }
        stitcher.setNumberOfImages(numberOfImages: numberOfImages)
        for i in stride(from:updatedLoopStride.from, to: updatedLoopStride.to, by: updatedLoopStride.by) {
            let resizedImage = Utils.getImage(imagePath: allPaths[i] ).resize(targetSize:size)
            stitcher.feed( image: resizedImage, location: allCoords[Int(index)] )
            index += 1
        }
        return stitcher.getStoredImage()
    }
}
