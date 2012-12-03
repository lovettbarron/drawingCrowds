#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxKinect.h"
#include "ofxAutoControlPanel.h"
#include "ofxBox2d.h"



class Scene {
    
public:
    
    Scene();
    ~Scene();
    void setup(string path, ofxBox2dJoint _joint, int _x, int _w, int select);
    void setPoint(ofVec2f * _pnt);
    void update();
    void draw();
    bool isDone();
    ofxBox2dJoint joint;
    ofVec2f pos;
    int x,w;
    ofVideoPlayer video;
    bool done;
private:
};


class testApp : public ofBaseApp{
    
	public:
		void setup();
		void update();
		void draw();
        void exit();
    
        void attractMode();
        void setupGUI();
        void setupCamera();
        void updateCamera();
        void drawCamDebug();

		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    
        int movieSelector, numOfMovies;
        ofShader shader, shader2;
        int rectWidth, rectHeight;
        ofTrueTypeFont font;
        bool attract, debug;
        float playDist;
        float depthMulti;
    int resetCounter;
        
        // Kinect
        ofxKinect kinect;
        ofVideoGrabber cam;
        
        // Video
        vector      <ofVideoPlayer>     video;
        vector      <Scene>             scenes;
        vector<ofVec3f> points;
        ofTrueTypeFont raleway;
        
        ofxAutoControlPanel panel;
        ofImage thresh;
        ofImage bgThresh;
        ofImage kDepth;
        cv::Mat kDepthMat;
        cv::Mat threshMat;
        
        ofxCv::RunningBackground background;
        int panelWidth; // Debugging away from img
        int angle; // Kinect angle
        ofPolyline brush;
        float threshold;
        ofxCv::ContourFinder contourFinder;
        
        //Box2d
        ofxBox2d						box2d;			  //	the box2d world
        ofxBox2dCircle					anchor1,anchor2;  //	fixed anchor
        vector		<ofxBox2dJoint>		joints;		
        vector      <ofxBox2dJoint>     vidJoint;
        vector		<ofxBox2dCircle>		boxes;			  //	defalut box2d rects
        vector      <ofxBox2dCircle>    homeBase;
    private:
};