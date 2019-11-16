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
    var numberOfImages:Int = 0
    var timeElapsed:Float = 0
    var stitcher:Stitcher?

    override func viewDidLoad() {
        super.viewDidLoad()
        // Initialize the stitcher with a data set path
        self.stitcher = Stitcher(path: "/Images/Museo-Ambar-Data-Set")
        // Create a timer to document process
        let displayTimer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(self.addToTimer), userInfo: nil, repeats: true)
        // Start stitching
        self.stitcher!.run { (image, mapCoordinates) in
            // The stitcher also returns a set of map coordinates which can be used to georeference the
            // resulting image on to a map.
            self.finalImageView.image = image
            self.updateLabel()
            displayTimer.invalidate()
        }
    }
    
    @objc func addToTimer() {
        timeElapsed += 0.1
        updateLabel()
    }
    
    func updateLabel() {
        self.infoLabel.text = "Time: \(Double(round(timeElapsed))) | Images: \(stitcher!.getNumberOfImages()) | Progress:\(stitcher!.getPercentEstimation() * 100)%" // Force unwrapping is OK for now.
    }

}
