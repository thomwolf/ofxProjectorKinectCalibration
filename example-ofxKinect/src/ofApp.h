#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

#include "ofxUI.h"

#include "ofxKinect.h"
#include "RGBDCamCalibWrapperOfxKinect.h"
#include "ofxKinectProjectorCalibration.h"

#include "ofxXmlSettings.h"

using namespace cv;

class ofApp : public ofBaseApp{

	public:
		void setup();
 //   void setupGui();
		void update();
		void draw();
    void drawProj(ofEventArgs & args);
      void exit();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    
    // gui
    void setupGui();
    ofxUISuperCanvas *          gui;
    ofxUISuperCanvas *			guiImageSettings;
    void guiEvent(ofxUIEventArgs &e);
    void guiUpdateLabels();
    //ofxPanel gui;
    shared_ptr<ofAppBaseWindow> projWindow;
    

private:
        
        // kinect & the wrapper
        
        ofxKinect                   kinect;
        ofxCvColorImage				kinectColorImage;
        ofxCvGrayscaleImage			kinectDepthImage;
        
        RGBDCamCalibWrapper*		kinectWrapper;
        
        // calibration
        KinectProjectorCalibration	kinectProjectorCalibration;
        bool						enableCalibration;
        ofxCv::ContourFinder        contourFinder;
        
        // output
        KinectProjectorOutput		kinectProjectorOutput;
        bool						enableTestmode;
        float                       chessboardThreshold;
        ofImage						thresholdedKinect;
        float                       lowThresh;
        float                       highThresh;
        int							blur;
        float                       maxReprojError;
    
        // settings
        int                         projectorWidth;
        int                         projectorHeight;
        
        ofParameterGroup labels;
    
        // second window
//        ofxSecondWindow             secondWindow;
		
};
