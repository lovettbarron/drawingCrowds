#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxKinect.h"
#include "ofxAutoControlPanel.h"
#include "ofxBox2d.h"
#include "pcl.h"


class Light {
    
public:
    
    Light(ofVec3f _position, int _id, int _clock, int _data, ofArduino * _arduino);
    ~Light();
    void update();
    void draw();
    unsigned char * getBuffer();
    int getBufferLength();
    void lightUpdate();
    void setStrength(float _power);
    void setLocation(ofVec3f _position);
    
    void drawArm(int num);
    void debug();
    bool isDone();
    ofVec3f position;
    bool done;
private:
    int lightId;
    int clockPin;
    int dataPin;
    int ledCount; // Per arm
    int ledPerMeter; // Num LEDs per meter
    int numOfArms; // Num of arms per light
    int width;
    int height;
    float power; // 1.0 for light strength
    ofArduino * arduino;
    vector<float> leds;
    vector<unsigned char> buffer;
    
};

class Scene {
  
public:
    void draw();
    
    
private:
    
};


class People {
    
public:
    People(ofVec3f _position, int _id);
    ~People();
    void draw();
    void update();
    void debug();
private:
    int height, width;
    ofVec3f position;
    int personId;
    
};

class testApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    void exit();
    
    void setupGUI();
    void setupCamera();
    void setupArduino();
    void updateCamera();
    void updateSettings();
    void writeArduino();
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
    
    // Globals
    bool debug, rotate;
    float depthMulti;
    int numberOfLights;
    ofVec3f room;
    
    // Kinect
    ofEasyCam camera;
    ofxKinect kinect;
    ofVideoGrabber cam;
    
    // Lights
    vector <Light*> lights;
    vector <People*> people;
    vector<ofVec3f> points;
    
    Light * testLight;
    
    ofArduino serial;
    
    // Kinects and controls
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
    ofxBox2d box2d;
    vector<ofxBox2dCircle> boxes;
private:
};