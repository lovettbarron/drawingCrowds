#include "testApp.h"

using namespace cv;
using namespace ofxCv;

//--------------------------------------------------------------
void testApp::setup(){
    setupGUI();
    setupCamera();
    ofSetVerticalSync(true);
    attract = false;
    debug = false;
    shader.load("shader/shader");
    shader2.load("shader/noise");
    rectWidth = 300;
    rectHeight = 300;
    font.loadFont("type/ProximaNova-Extrabold.otf", 127, true, false, true, 0.4, 72);
    resetCounter = 0;
    movieSelector = 0;
    numOfMovies = 19; // Num -1
    
    box2d.init();
	box2d.setGravity(0, 10);
    box2d.createBounds(0,0,2000,400);
	box2d.setFPS(10.0);
	box2d.registerGrabbing();
	
    anchor1.setup(box2d.getWorld(), 0, 0, 20);
    anchor2.setup(box2d.getWorld(), ofGetWidth(), 0, 20);
    
    playDist = ofGetWidth()/3;
    
    int numOfVideos = 15;
    
    for (int i=0; i<numOfVideos; i++) {
		
        ofxBox2dCircle home;
        home.setup(box2d.getWorld(), i*128,100,40);
        homeBase.push_back(home);

		ofxBox2dCircle circle;
		circle.setPhysics(9999.0, 0.01, 2.9);
		circle.setup(box2d.getWorld(),  i*128,100, 4);
		boxes.push_back(circle);
	}
    
    for (int i=0; i<boxes.size(); i++) {
      boxes[i].addAttractionPoint(homeBase[i].getPosition(), .9);
		ofxBox2dJoint joint;
        
        
        
        joint.setup(box2d.getWorld(), homeBase[i].body, boxes[i].body);		
        
//		// if this is the first point connect to the top anchor.
//		if(i == 0) {
//			joint.setup(box2d.getWorld(), anchor1.body, boxes[i].body);		
//                        ofLog() << "Conncted to first anchor";
//		} else
//        if(i == boxes.size()-1) {
//            joint.setup(box2d.getWorld(), boxes[i-1].body, boxes[i].body);
//            joint.setup(box2d.getWorld(), boxes[i].body, anchor2.body);		        
//            ofLog() << "Conncted to last anchor";
//        }
//		else {
//			joint.setup(box2d.getWorld(), boxes[i-1].body, boxes[i].body);
//		}
        
		joint.setLength(200);
        joint.setDamping(.5);
		joints.push_back(joint);
	}
    vidJoint.clear();
}

void testApp::exit() {
    kinect.close();
    kinect.clear();
}

void testApp::setupGUI() {
    panelWidth = 200;
    panel.setup(panelWidth, 800);
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
    panel.addSlider("idPos",-500,-500,500,true);
    
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
        if(kinect.isFrameNew()) {
            kDepthMat = toCv(kinect.getDepthPixelsRef());
            blur(kDepthMat, 10);
            kDepthMat -= threshMat;
            contourFinder.findContours(kDepthMat);
//            brush = getContour(&contourFinder);
        }
        
    }
}


//--------------------------------------------------------------
void testApp::update(){
	box2d.update();	
    updateCamera();
    depthMulti = panel.getValueF("DepthMultiplier");
	ofVec2f mouse(ofGetMouseX(), 5);
	float minDis = ofGetMousePressed() ? 300 : 200;
    
	for(int i=0; i<boxes.size(); i++) {
        int personCount=7;
        if(contourFinder.size()<personCount) personCount = contourFinder.size();
        for(int j = 0; j < personCount; j++) {
            ofPoint center = ofPoint(0,0);
            center = toOf(contourFinder.getCenter(j));
            float depth = 1.0;
            if(kinect.isConnected())
                depth = kinect.getDistanceAt(center)*depthMulti;
            else depth = 10*depthMulti;
            
             boxes[i].addRepulsionForce((center.x/640) * ofGetWidth(),10, depth);
        }
		float dis = mouse.distance(boxes[i].getPosition());
       // float minDis = 10;
        
//        ofLog() << "dis" << ofToString(i) << ": " << ofToString(dis);
//		 boxes[i].addRepulsionForce(mouse.x,mouse.y, 1.0);

         // Lets check distances
//        float distA = boxes[i-1].getPosition().distance(boxes[i].getPosition());
//        float distB = boxes[i+1].getPosition().distance(boxes[i].getPosition());
        
        // if this point is beyond the "play" distance or playing
//        // Make sure that it stays within a bound, so apply a joint
//        if(distA >= playDist) {
//            
//        } else
//        if(distA <= minDis) {
//            
//        }
        
        //If not, then keep it joint free
        if(i!=0) {
           // ofLog() << "Dist" << ofToString(i) << ": " << ofToString(boxes[i-1].getPosition().distance(boxes[i].getPosition()));
            if(resetCounter == 0) {
            if(boxes[i-1].getPosition().distance(boxes[i].getPosition()) > 450) {
                bool isOverlap = false;
                if(scenes.size() < 2) {
                    for(int j=0;j<scenes.size();j++){ 
                        if(boxes[i-1].getPosition().distance(scenes[j].pos) < panel.getValueI("OverlapDistance")) {
                            isOverlap = true;
                            //ofLog() << "Overlap happening.";
                        };
                    }
                    if(!isOverlap) {
                        ofLog() << "New Joint!";
                        ofxBox2dJoint joint;
                        joint.setup(box2d.getWorld(), boxes[i-1].body, boxes[i].body);
                        joint.setLength(500);
                        Scene scn = Scene();
                        scenes.push_back(scn); 
                        scenes[scenes.size()-1].setup("", joint, boxes[i-1].getPosition().x, 640, movieSelector);
                        movieSelector = (movieSelector+1)%numOfMovies;
                    }
                }
                }
            } else {
                resetCounter -= 1;
            }
        }
	}
    bool stillActive = false;
    for(int i=0;i<scenes.size();i++) {
        scenes[i].update();
        if(!stillActive)
        stillActive = scenes[i].isDone();
    }
    if(!stillActive) {
        scenes.clear();
        if(resetCounter == 0) resetCounter = 100;
        ofLog() << "Reseting movies";
    }
   if(scenes.size() == 0) {
       resetCounter = 0;
   }
    
    if(ofGetSeconds()%330 == 0) {
        scenes.clear();
        ofLog() << "Kill switch!";
    }
    
}

//--------------------------------------------------------------
void testApp::draw(){
    ofBackgroundGradient(ofColor(0),ofColor(10) );    
    
    //glEnable(GL_ALPHA_TEST);
	//glDisable(GL_DEPTH_TEST);
    if(attract) {
        attractMode();
    } else {
        if(debug) {
        ofSetHexColor(0x123456);
//        anchor1.draw();
//        anchor2.draw();
        for(int i=0; i<boxes.size(); i++) {
            ofFill();
        ofSetHexColor(0x123456);
            homeBase[i].draw();
            ofSetHexColor(0xBF2545);
            boxes[i].draw();
        }
        
        // draw the ground
        box2d.drawGround();
        for(int i = 0; i < contourFinder.size(); i++) {
            ofPoint center = toOf(contourFinder.getCenter(i));
            
            float depth;
            if(kinect.isConnected())
                depth = kinect.getDistanceAt(center)*.1;
            else depth = 1.0;
            
            ofEllipse((center.x/640) * 1024,100, depth,depth);
        }
        drawCamDebug();
        }
        else {
            shader.begin();
            if(contourFinder.size() >= 1)
                shader.setUniform2f("push1", (float)contourFinder.getCenter(0).x, (float)contourFinder.getCenter(0).y);
            if(contourFinder.size() >= 2)
                shader.setUniform2f("push2", (float)contourFinder.getCenter(1).x, (float)contourFinder.getCenter(1).y );
            if(contourFinder.size() >= 3)
                shader.setUniform2f("push3", (float)contourFinder.getCenter(2).x, (float)contourFinder.getCenter(2).y );
            if(contourFinder.size() >= 4)
                shader.setUniform2f("push4", (float)contourFinder.getCenter(3).x, (float)contourFinder.getCenter(3).y );
            for(int i=1;i<boxes.size();i++) {
                ofSetColor((int)(i*60)%255, (int)(ofGetElapsedTimeMillis()/1000)%255,(int)(i*20)%255);
               float dist;
//                if(i==0)
//                    dist = anchor1.getPosition().distance(boxes[i].getPosition());
//                else dist = boxes[i].getPosition().distance(boxes[i-1].getPosition());
                ofRect(boxes[i-1].getPosition().x, 0, boxes[i].getPosition().x, ofGetHeight());
            }
            shader.end();
            ofSetColor(255);
            for(int i=0;i<scenes.size();i++) {
                scenes[i].draw();
            }
        }
    }
    
    
    glDisable(GL_ALPHA_TEST);
    glDisable(GL_DEPTH_TEST);
}

void testApp::attractMode() {
    glDisable(GL_BLEND);
    ofPushMatrix();
    ofPushMatrix();
    // ofTranslate(ofGetWidth()/2-rectWidth/2,ofGetHeight()/2-rectHeight/2);
    ofSetColor(0);
    ofRect(0,0,ofGetWidth(),ofGetHeight());
    ofPopMatrix();
    ofPushMatrix();
    ofTranslate(0, ofGetHeight()/2);
    shader2.begin();
    shader2.setUniform2f("mouse",ofGetMouseX(),ofGetMouseY());
    shader2.setUniform1f("dist", ofGetMouseX()/ofGetWidth() );
    shader2.setUniform1f("time", ofGetElapsedTimef());
    ofSetColor(255);
    ofPushMatrix();
    ofScale(1.2,1);
    int trans = (int)(ofGetElapsedTimeMillis()/100)%60;
    
    if(trans <= 10)
        font.drawStringAsShapes("ACCESSIBILITY", 0, 0);
    if(trans >= 10 && trans <= 20)
        font.drawStringAsShapes("CITIZEN", 0, 0);
    if(trans >= 20 && trans <= 30)
        font.drawStringAsShapes("CULTURE", 0, 0);
    if(trans >= 30 && trans <= 40)
        font.drawStringAsShapes("CONTROL", 0, 0);
    if(trans >= 40 && trans <= 50)
        font.drawStringAsShapes("MAKE", 0, 0);
    if(trans >= 50 && trans <= 60)
        font.drawStringAsShapes("BOLD", 0, 0);
    
    ofPopMatrix();
    shader2.end();
    
    ofSetColor(255);
    
    ofTranslate(ofGetWidth()-500,0);
    ofPushMatrix();
    ofScale(panel.getValueF("idScale"),1);
    ofTranslate(-ofGetWidth()/2+100,-ofGetHeight()/2+panel.getValueF("idPos"));
    for(int i=1;i<boxes.size();i++) {
        ofSetColor((int)(i*60)%255, (int)(ofGetElapsedTimeMillis()/1000)%255,(int)(i*20)%255);
        float dist;
        ofRect(boxes[i-1].getPosition().x, boxes[i].getPosition().y, boxes[i].getPosition().y,  boxes[i].getPosition().y);
    }
    ofPopMatrix();
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_DST_ALPHA);  
    ofSetColor(255);
    font.drawStringAsShapes("SHIFT", 0, 0);
    glDisable(GL_BLEND);
    
    ofPopMatrix();
    
    
    
    ofPopMatrix();
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
            scenes.clear();
            resetCounter = 100;
//            if(!attract) attract = true;
//            else attract = false;
            break;
        case 'i':
            if(!attract) attract = true;
            else attract = false;
            break;
        case 'd':
            if(!debug) debug = true;
            else debug = false;
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
Scene::Scene() {

}

Scene::~Scene() {
    video.closeMovie();
}

void Scene::setup(string path, ofxBox2dJoint _joint, int _x, int _w, int select) {
    video.setUseTexture(true);
    if(video.isLoaded()) video.close();
    video.setLoopState(OF_LOOP_NONE);
    video.setPixelFormat(OF_PIXELS_RGB);
    video.setVolume(1.0);
    video.loadMovie("movie/" + ofToString(select) + ".mov");
    video.play();
    done = false;
    joint = _joint;
    ofLog() << "Playing movie/" << ofToString(select) << ".mov";
    
    x = _x;
    w = 640;   
    pos = ofVec2f(x,50);
}

void Scene::update(){
//    if(!video.isPlaying() && !done) video.play();
    if(video.isLoaded()) video.update();
    else if(!done) { 
        setup("",joint,x,640,ofRandom(0,18));
      //  ofLog() << "No video loaded";
    } else {
        joint.destroy();
        video.close();
    }
    
//    ofLog() << "Cur Frame " << ofToString(video.getCurrentFrame()) << " of " << ofToString(video.getTotalNumFrames());
    if(video.getCurrentFrame() >= video.getTotalNumFrames()-50 ) {
        video.close();
        done = true;
        ofLog() << "Closing movie.";
    }
}

void Scene::draw() {
//    video.draw(x,0);
    ofPushMatrix();
    ofTranslate(x,0);
    if(video.getCurrentFrame() < 100) {
        ofScale( 1,video.getCurrentFrame()*.01);
    } else if(video.getCurrentFrame()+100 >= video.getTotalNumFrames()) {
        float reduce = (video.getTotalNumFrames()-video.getCurrentFrame())*.01;
        ofScale( 1, reduce );
    } else ofScale(1,1);
    ofSetColor(0);
   ofRect(-10,-10,660,ofGetHeight()+20);
    ofSetColor(255);
    video.getTextureReference().draw(0,300,640,400);
    ofPopMatrix();
}


bool Scene::isDone() {
  //  return video.isPlaying(); 
   return done;
}