//
//  CVStitcher.h
//  Quick Map
//
//  Created by Andre on 7/29/19.
//  Copyright © 2019 3d Robotics. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>

@interface CVStitcher : NSObject {
}
-(UIImage*) createMap;
-(NSArray*) getMapPointCorners: (NSArray*) mapPoints;
-(void) stopStitching;
-(bool) feed: (UIImage*) image withIndex: (int) index withGPS: (CGPoint) gpsPoint;
-(void) deconstruct;
-(int) getProcessStatus;

@end

