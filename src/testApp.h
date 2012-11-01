#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxAutoControlPanel.h"
#include "ofxKinect.h"
#include "ofxBox2d.h"
#include "ofxFft.h"

class testApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
    void exit();

		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
    void audioReceived(float* input, int bufferSize, int nChannels);
    void plot(vector<float>& buffer, float scale, float offset);
    
private:
    // Vectors
    ofVec2f overallRotation;
    ofxBox2d box2d;
    vector <ofxBox2dCircle> circles;
    vector <ofxBox2dRect> boxes;
    ofPolyline primary;
    ofTessellator tess;
    ofMesh mesh;
    ofShader toon;
    ofSoundStream mic;
    ofxAutoControlPanel panel;
    
    // Cam
    ofxKinect kinect;
    ofVideoGrabber cam;
    ofImage img;
    cv::Mat imgMat;
    ofxCv::FlowFarneback farneback;    
    ofxCv::Flow* curFlow;
    ofFbo tex;
    ofCamera camera;
    int sphereRadius;
    int density;
    
    //FFT
    int plotHeight, bufferSize;
    vector <float> left;
		vector <float> right;
		vector <float> volHistory;
		
		int 	bufferCounter;
		int 	drawCounter;
    float maxValue;
    
		float smoothedVol;
		float scaledVol;

    ofxFft* fft;
    ofColor clr;
    
    ofMutex soundMutex;
    vector<float> drawBins, middleBins, audioBins;
    vector<float> drawBuffer, middleBuffer, audioBuffer;
    // Debug
    bool debug;
};
