#include "testApp.h"

using namespace cv;
using namespace ofxCv;

//--------------------------------------------------------------
void testApp::setup(){
    ofSetVerticalSync(true);
//    ofEnableLighting();
 
    box2d.init();
	box2d.setGravity(0, 0);
    box2d.createBounds(0,0,2000,400);
	box2d.setFPS(10.0);
	box2d.registerGrabbing();
    
    for (int i=0; i<5; i++) {
		
        ofxBox2dCircle home;
        home.setup(box2d.getWorld(), i*128,100,40);

		ofxBox2dCircle circle;
		circle.setPhysics(9999.0, 0.01 , 2.9);
		circle.setup(box2d.getWorld(),  i*128,100, 4);
		boxes.push_back(circle);
	}
    
//    testLight = new Light(ofVec3f(0,0,0), 0);
    
    camera.disableMouseInput();
    
    numberOfLights = 6;
    room = ofVec3f(1000,400,500);
    
    for(int i=0;i<numberOfLights;i++) {
        lights.push_back( new Light(ofVec3f(ofRandom(0,room.x), room.y, ofRandom(0,room.z) ), i, 10+i, 12+i, &serial) );
    }
    
    for(int i=0;i<10;i++) {
        people.push_back( new People( ofVec3f(ofRandom(0,room.x),0,ofRandom(0,room.z)),i));
    }
    setupGUI();   
    setupCamera();
    for(int i=0;i<1;i++) {
        cameras.push_back( new Camera( ofVec3f(), 0, &kinect) );
    }
    
    setupArduino();
}

void testApp::setupArduino() {
    
    // Protocol!
    // a : armNumber
    // l : lightNumber
    // e : End count
    
    //eg l1a2:255:255:166:177e LightId 1, arm 2, led 1, led2, led3, end line
    
 //   serial.listDevices();
//	vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();
	serial.connect("/dev/tty.usbserial-A900acdV",57600); // Firmata
    
}

void testApp::writeArduino() {

//}

    
    
    
    for(int l = 0; l<lights.size();l++) {
        lights[l]->update();
//        serial.writeBytes(lights[l]->getBuffer(),lights[l]->getBufferLength());   
    }
}

void testApp::exit() {
    kinect.close();
    kinect.clear();
}

void testApp::setupGUI() {
    panelWidth = 300;
    panel.setup(panelWidth, 1024);
    
    panel.addPanel("PointCloud");
    panel.addSlider("cameraDistance",700,0,1000,false);
    panel.addLabel("Camera");
    panel.addSlider("cam1x", 0, 0., 1., false);
    panel.addSlider("cam1y", 0, 0., 1., false);
    panel.addSlider("cam1z", 0, 0., 1., false);
    
    for( int i=0;i<numberOfLights;i++) {
        panel.addPanel("Light" + ofToString(i));    void setLocation(ofVec3f _position);
        panel.addLabel("Light" + ofToString(i) );
        panel.addSlider("l" + ofToString(i) + "pwr", 1., 0., 1., false);
        panel.addSlider("l" + ofToString(i) + "x", lights[i]->getLocation().x / room.x, 0., 1., false);
        panel.addSlider("l" + ofToString(i) + "y", lights[i]->getLocation().y / room.y, 0., 1., false);
        panel.addSlider("l" + ofToString(i) + "z", lights[i]->getLocation().z / room.z, 0., 1., false);
    }
    
    panel.addPanel("Tracking Bits");
    panel.addLabel("Image Processing");
    panel.addSlider("maxThreshold", 15, 0, 255, true);
    panel.addSlider("minAreaRadius", 7, 0, 640, true);
    panel.addSlider("maxAreaRadius", 100, 0, 640, true);
    panel.addSlider("DepthMultiplier", .01,0,5.,false);
    panel.addLabel("Background Subtraction");
    panel.addSlider("learningTime",900,0,2000,true);
    panel.addSlider("backgroundThresh",10,0,50,true);
    panel.addToggle("resetBg", false);
    panel.addSlider("OverlapDistance", 500, 0,1000,true);
    
    panel.addSlider("idScale",.3,0,1.,false);
    panel.addSlider("idPosx",-500,-700,700,true);
    panel.addSlider("idPosy",-500,-700,700,true);
    
    panel.addPanel("Kinect");   
    panel.addSlider("angle", 0, -40, 40, true);
    angle = panel.getValueI("angle");
}

void testApp::setupCamera() {
    kinect.setRegistration(true);
    ofLog() << "Starting first kinect";
    kinect.init(false, false, true); // infrared=false, video=true, texture=true
    kinect.open(0);
    kinect.setCameraTiltAngle(angle);
    
    if(!kinect.isConnected()) {
        cam.initGrabber(640, 480);
    }
    
    thresh.allocate(640, 480, OF_IMAGE_GRAYSCALE);
    kDepth.allocate(640, 480, OF_IMAGE_GRAYSCALE);
    kDepthMat.create(480, 640, CV_8UC1);
    //threshMat.create(480, 640, CV_32F);
    imitate(threshMat, kDepthMat);
    //    imitate(kDepth, kDepthMat);
    //    imitate(thresh, threshMat);
    
    background.setThresholdValue(panel.getValueI("backgroundThresh"));
    background.setLearningTime(panel.getValueI("learningTime"));
    
    contourFinder.setMinAreaRadius(panel.getValueI("minAreaRadius"));
    contourFinder.setMaxAreaRadius(panel.getValueI("maxAreaRadius"));
    contourFinder.setThreshold(panel.getValueI("maxThreshold"));
    // wait for half a frame before forgetting something
    contourFinder.getTracker().setPersistence(15);
    // an object can move up to 32 pixels per frame
    contourFinder.getTracker().setMaximumDistance(32);
}

void testApp::updateCamera() {
    contourFinder.setMinAreaRadius(panel.getValueI("minAreaRadius"));
    contourFinder.setMaxAreaRadius(panel.getValueI("maxAreaRadius"));
    contourFinder.setThreshold(panel.getValueI("maxThreshold"));
    background.setLearningTime(panel.getValueI("learningTime"));
    background.setThresholdValue(panel.getValueI("backgroundThresh"));
    if(panel.hasValueChanged("angle")) {
        angle = panel.getValueI("angle");
        kinect.setCameraTiltAngle(angle);
    }   
    
    
    if(panel.getValueB("resetBg")) {
        //        background.reset();
        threshMat = kDepthMat.clone();
        panel.setValueB("resetBg",false);
    }
    
    if(!kinect.isConnected()) {
        cam.update();
        if(cam.isFrameNew()) {            
            background.update(cam, thresh);
            thresh.update();
            blur(cam, 10);
            contourFinder.findContours(cam);
        }
    } else {
        kinect.update();
//        if(kinect.isFrameNew()) {
//            kDepthMat = toCv(kinect.getDepthPixelsRef());
//            blur(kDepthMat, 10);
//            kDepthMat -= threshMat;
//            contourFinder.findContours(kDepthMat);
////            brush = getContour(&contourFinder);
//        }
        
    }
}


//--------------------------------------------------------------
void testApp::update(){
	box2d.update();	
    updateCamera();
    depthMulti = panel.getValueF("DepthMultiplier");
	ofVec2f mouse(ofGetMouseX(), 5);
	float minDis = ofGetMousePressed() ? 300 : 200;
    
    camera.setTarget(ofVec3f(room.x/2,room.y/2,room.z/2));
    camera.setPosition(room.x, room.y, room.z);
    camera.setDistance(panel.getValueF("cameraDistance"));
//    for(int i=0;i<lights.size();i++) {
//    }
    
    for(int i=0;i<cameras.size();i++) {
        ofVec3f camPosition(
            panel.getValueF("cam1x") * room.x,
            panel.getValueF("cam1y") * room.y,
            panel.getValueF("cam1z") * room.z
        );
        cameras[i]->setLocation(camPosition);
        cameras[i]->update();
    }
    
    for(int i=0;i<people.size();i++) {
        people[i]->update();
    }
    updateSettings();
    writeArduino();
}

//--------------------------------------------------------------
void testApp::updateSettings(){
    for( int i=0;i<numberOfLights;i++) {
        float pwr = panel.getValueF("l" + ofToString(i) + "pwr");
        float x = panel.getValueF("l" + ofToString(i) + "x");
        float y = panel.getValueF("l" + ofToString(i) + "y");
        float z = panel.getValueF("l" + ofToString(i) + "z");
        lights[i]->setLocation(ofVec3f(x*room.x,y*room.y,z*room.z));
        lights[i]->setStrength(pwr);
    }
}

//--------------------------------------------------------------
void testApp::draw(){
    ofBackgroundGradient(ofColor(200),ofColor(170) );    
    
    glEnable(GL_DEPTH_TEST);
    
	camera.begin();	
//        ofTranslate(0,-400);
        if(rotate)
            ofRotateY(ofRadToDeg(  ofGetElapsedTimeMillis()*.0001 ));
    
        ofSetColor(200);
        
//        testLight->draw();
        for(int i=0;i<lights.size();i++) {
            lights[i]->draw();
        }
        for(int i=0;i<people.size();i++) {
            people[i]->draw();
        }
    
        for(int i=0;i<cameras.size();i++) {
            cameras[i]->draw();
        }
        ofPushMatrix();
            ofSetColor(100,100,100);
            ofTranslate(room.x/2,0,room.z/2);
            ofScale(room.x,0,room.z);
            ofBox(1);
        ofPopMatrix();
    
    camera.end();
    
    
    
    if(debug) {
    ofSetHexColor(0x123456);
    
        for(int i=0;i<lights.size();i++) {
            lights[i]->debug(); 
        }
    
    for(int i = 0; i < contourFinder.size(); i++) {
        ofPoint center = toOf(contourFinder.getCenter(i));
        
        float depth;
        if(kinect.isConnected())
            depth = kinect.getDistanceAt(center)*.1;
        else depth = 1.0;
        
//        ofEllipse((center.x/640) * ofGetWidth(),100, depth,depth);
    }
    drawCamDebug();
    }
    

    
    glDisable(GL_ALPHA_TEST);
    glDisable(GL_DEPTH_TEST);
}

void testApp::drawCamDebug() {
    
    ofPushMatrix();
    ofTranslate(ofGetWidth()/2, 0);
    ofScale(.5,.5);
        ofSetColor(255);
        if(!kinect.isConnected()) cam.draw(0, 0);
        else kinect.drawDepth(0,0);
        
        contourFinder.draw();
        
        for(int i = 0; i < contourFinder.size(); i++) {
            ofPoint center = toOf(contourFinder.getCenter(i));
            
            float depth;
            if(kinect.isConnected())
                depth = kinect.getDistanceAt(center);
            else depth = 0;
            
            ofPushMatrix();
            ofTranslate(center.x, center.y);
            int label = contourFinder.getLabel(i);
            ofDrawBitmapString(ofToString(depth*depthMulti), 0, 12);
            ofVec2f velocity = toOf(contourFinder.getVelocity(i));
            ofScale(5, 5);
            ofLine(0, 0, velocity.x, velocity.y);
            ofPopMatrix();
    }
    
//    thresh.draw(thresh.width, 0, 2, 256,192);
    ofPopMatrix(); // For panel
}


//--------------------------------------------------------------
void testApp::keyPressed(int key){
    switch(key) {
        case ' ':
            break;
        case 'i':
            break;
        case 'r':
            rotate =!rotate;
        case 'd':
            debug = !debug;
            break;
            
        case 'm':
			if(camera.getMouseInputEnabled()) camera.disableMouseInput();
			else camera.enableMouseInput();
			break;
    }
}


//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}

//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------

Light::Light(ofVec3f _position, int _id, int _clock, int _data, ofArduino * _arduino) {
    position = _position;
    lightId = _id;
    clockPin = _clock;
    dataPin = _data;
    ledCount = 20;
    ledPerMeter = 48;
    width = 10;
    height = 100;
    numOfArms = 3;
    power = 0.;
    
    arduino = _arduino;
    
    for(int i=0;i<numOfArms;i++) {
        arduino->sendDigitalPinMode(clockPin+i, ARD_PWM);
        arduino->sendDigitalPinMode(dataPin+i, ARD_PWM);
    }
    
    for(int i=0;i<ledCount*numOfArms;i++) {
        leds.push_back(ofNoise(i,i,i));
    }
    
}
    
Light::~Light() {
    
}

void Light::setStrength(float _power) {
    power = _power;
}

void Light::draw() {
    ofPushMatrix();
    ofTranslate(position);
    ofRotate(110,-1,0,0);
    for(int i=0;i<numOfArms;i++) {
        int rotation = i * (360/numOfArms);
        ofPushMatrix();
            ofRotate(rotation,0,-1,1);
            ofRotate(45,0,-1,0);
            drawArm(i);
        ofPopMatrix();
    }
    ofPopMatrix();
}

void Light::lightUpdate() {
//    int per = (leds.size() / ledCount) * power;
    int per = floor(( (leds.size()/numOfArms) * (1-power)) + 1);
    //ofLog() << ofToString(per);
//    for(int i=0;i<leds.size();i+=ofRandom(per*.5,per)) {
//        leds[i] = ofRandom(0,1);//ofNoise(leds[i]);
//    }
//    if(!per) per = leds.size() / numOfArms;
    for(int i=0;i<leds.size();i++) {
        if(i%per == 0) leds[i] = ofRandom(0,1);
        else if (ofRandom(0,4) == 0) leds[i] = ofRandom(0,1);
        else leds[i] = 0;
    }
}

void Light::update() {
    lightUpdate();
     // Writing to buffer (straight serial)
    buffer.clear();
    // set Light ID
    buffer.push_back(0);
    buffer.push_back('l');
    
    for(int arm=0;arm<numOfArms;arm++) {
        // Set Arm ID
        buffer.push_back(lightId);
        buffer.push_back('a');
        // Run through arm LED's
        for(int i=0;i<leds.size();i++) {
            buffer.push_back(':');
            int theLed = static_cast<int>(floor(leds[i] * 255));
            buffer.push_back(theLed);
        }
    }
    // End the light value
    buffer.push_back('e');

//    for(int i=0;i<numOfArms;i++) {
//        arduino->sendPwm(clockPin+i, (int)(128 + 128 * sin(ofGetElapsedTimef())));
//    }
}

unsigned char * Light::getBuffer() {
    unsigned char * bufferPtr;
    bufferPtr = &buffer[0];
//    ofLog() << ofToString(bufferPtr);
    string buff;
    for(int i=0;i<buffer.size();i++) {
        buff += buffer[i];
    }
    ofLog() << buff;
    return bufferPtr;
}

int Light::getBufferLength() {
   return buffer.size();
}

void Light::setLocation(ofVec3f _position) {
    position = _position;
}

ofVec3f Light::getLocation() {
    return position;
}

void Light::drawArm(int num) {
    ofSetColor(255);
    ofPushMatrix();
        ofTranslate(width/2,height/2,width/2);
        ofSetColor(56,19,11);
        ofPushMatrix();
            ofScale(width,height, width);
            ofBox(1);
        ofPopMatrix();
    ofPopMatrix();
    for(int i=0;i<ledCount;i++) {
        ofPushMatrix();
        ofTranslate(0, i * (height/ledCount), width*.1 );
        ofSetColor(leds[i+(ledCount*num)]*255, (leds[i+(ledCount*num)]) * 127, 0);
        ofSphere(width/2);
        ofPopMatrix();
    }
}

void Light::debug() {
    glDisable(GL_DEPTH_TEST);
    ofPushMatrix();
    ofTranslate(ofGetWidth()-(ledCount*2),lightId*100);
    int arm = 0;
    ofSetColor(255);
    ofDrawBitmapString("Light" + ofToString(lightId),0,12);
    for(int i=0;i<leds.size();i++) {
        if(i%ledCount == 0) {
            arm+=1;
            ofPushMatrix();
            ofTranslate(-50,arm*20);
            ofSetColor(255);
            ofDrawBitmapString("arm" + ofToString(arm),0,12);
            ofPopMatrix();
        }
        ofPushMatrix();
        ofTranslate(i%ledCount,arm*20);
        ofSetColor(leds[i]*255, 100, 100);
        ofScale(2,1);
        ofLine(0,0,0,20);
        ofPopMatrix();
    }
    ofPopMatrix();
    glEnable(GL_DEPTH_TEST);
}

float Light::lin2log(float _lin) {
    return log10(_lin);
}


//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------

People::People(ofVec3f _position, int _id) {
    position = _position;
    personId = _id;
    width = 10;
    height = 100;
    
}

People::~People() {
    
}

void People::update() {
    position += ofVec3f(ofRandom(-10,10),0,ofRandom(-10,10));
}

void People::draw() {
    ofPushMatrix();
    ofTranslate(position);
    ofScale(width,height,width);
    ofSetColor(127,50,50);
    ofSphere(1);
    ofPopMatrix();
}



//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------


Camera::Camera(ofVec3f _position, int _id, ofxKinect * _kinect) {
    position = _position;
    personId = _id;
    kinect = _kinect;
    width = 100;
    height = 10;
    depth = 10;
    
}

Camera::~Camera() {
    
}

void Camera::update() {

}

void Camera::draw() {
    ofPushMatrix();
    ofTranslate(position);
    ofScale(width,height,depth);
    ofSetColor(127,50,50);
    ofSetColor(0);
    ofBox(1);
    if(kinect->isFrameNew()) {
        ofSetColor(255);
        for(int y=0;y<kinect->getHeight();y++) {
            for(int x=0;y<kinect->getWidth();x++) {
                int ptr = x + (y * width);
                ofRect(x*10,y*10,kinect->getDepthPixelsRef()[ptr], 10,10);
            }
        }
    }
        
    ofPopMatrix();
}

void Camera::setLocation(ofVec3f _position) {
    position = _position;
}

ofVec3f Camera::getLocation() {
    return position;
}