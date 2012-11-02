#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    ofSetVerticalSync(true);
    //    ofEnableSmoothing();
    
        kinect.setRegistration(false);
        kinect.init(false, false, true);
        if(kinect.open(0)) ofLog() << "Kinect on";
        else cam.initGrabber(320,240);
    
    camera.setPosition(ofVec3f(0,0,200));
    camera.setNearClip(100);
    camera.lookAt(ofVec3f(0,0,0));
    
    tex.allocate(480,480,GL_RGBA,1);
    tex.begin();
    ofClear(0);
    tex.end();
    
    overallRotation = ofVec2f();
    
    plotHeight = 128;
    bufferSize = 512;
    fft = ofxFft::create(bufferSize, OF_FFT_WINDOW_HAMMING);
        //fft = ofxFft::create(bufferSize, OF_FFT_WINDOW_HAMMING, OF_FFT_FFTW);
    
    drawBuffer.resize(bufferSize);
    middleBuffer.resize(bufferSize);
    audioBuffer.resize(bufferSize);
    
    drawBins.resize(fft->getBinSize());
    middleBins.resize(fft->getBinSize());
    audioBins.resize(fft->getBinSize());
    
    ofSoundStreamSetup(0, 2, this, 44100, bufferSize, 4);
    
    
    
    panel.setup(200, 800);
    
    panel.addPanel("drawingCrowds");
    panel.addSlider("smoothed", 3, 1, 7, true);
    panel.addSlider("impact", 0, 0, 1, false);    
    panel.addLabel("Physics");
    panel.addSlider("density",10,1,10,true);
    panel.addSlider("bounce",5,0,2,false);
    panel.addSlider("friction",5,0,2,false);
    panel.addSlider("attr",1,0,1,false);
    panel.addSlider("sphereRad",300,10,500,true);
    panel.addSlider("lineWidth",1,0,1,false);
    panel.addSlider("strokeRandom",2,0,20,false);
    
    panel.addLabel("Flow");
    panel.addSlider("pyrScale", .5, 0, 1);
    panel.addSlider("levels", 4, 1, 8, true);
    panel.addSlider("winsize", 8, 4, 64, true);
    panel.addSlider("iterations", 2, 1, 8, true);
    panel.addSlider("polyN", 7, 5, 10, true);
    panel.addSlider("polySigma", 1.5, 1.1, 2);
    
    panel.addPanel("cam");
    panel.addLabel("kinect");
    panel.addSlider("kAngle",0,-45,45, true);
    panel.addSlider("texDelay",10,0,255,true);
    panel.addSlider("maxThreshold", 15, 0, 255, true);
    panel.addSlider("minAreaRadius", 7, 0, 640, true);
    panel.addSlider("maxAreaRadius", 100, 0, 640, true);
    panel.addLabel("Background Subtraction");
    panel.addSlider("learningTime",900,0,2000,true);
    panel.addSlider("backgroundThresh",10,0,50,true);
    panel.addToggle("resetBg", false);

    
    
    box2d.init();
    box2d.setGravity(0, 0);
    box2d.setFPS(30.0);
       
    contourFinder.setMinAreaRadius(panel.getValueI("minAreaRadius"));
    contourFinder.setMaxAreaRadius(panel.getValueI("maxAreaRadius"));
    contourFinder.setThreshold(panel.getValueI("maxThreshold"));
        // wait for half a frame before forgetting something
    contourFinder.getTracker().setPersistence(15);
        // an object can move up to 32 pixels per frame
    contourFinder.getTracker().setMaximumDistance(32);
    
    toon.load("shader/noise");
    
		
    for (int i=0; i<30; i++) {
        
        
        float r = ofRandom(10, 20);		
        ofxBox2dCircle circle;
        circle.setPhysics(6.0, 0.3, 1.9);
        circle.setup(box2d.getWorld(), ofGetWidth()/2, ofGetHeight()/2, r);
        circles.push_back(circle);
    }
    curFlow = &farneback;
}

void testApp::exit() {
    kinect.close();
}

//--------------------------------------------------------------
void testApp::update(){
    scaledVol = panel.getValueF("attr");
    sphereRadius = panel.getValueI("sphereRad");
    density = panel.getValueI("density");
    
    if(panel.hasValueChanged("kAngle"))
        kinect.setCameraTiltAngle(panel.getValueI("kAngle"));
    
    if(panel.hasValueChanged("texDelay"))
        delay = panel.getValueI("texDelay");
    
    if(kinect.isConnected()) kinect.update();
    else cam.update();
    
    contourFinder.setMinAreaRadius(panel.getValueI("minAreaRadius"));
    contourFinder.setMaxAreaRadius(panel.getValueI("maxAreaRadius"));
    contourFinder.setThreshold(panel.getValueI("maxThreshold"));

    
    if(kinect.isFrameNew() || cam.isFrameNew()) {
        if(kinect.isConnected()) {
            img.setFromPixels(kinect.getDepthPixels(), kinect.getWidth(), kinect.getHeight(), OF_IMAGE_GRAYSCALE);
                //            img.allocate(kinect.getWidth()/2, kinect.getHeight()/2, OF_IMAGE_GRAYSCALE);
                //            ofxCv::resize(kDepth, img);
            imgMat = ofxCv::toCv(img).clone();
        }
        else {
            img.setFromPixels(cam.getPixels(),cam.getWidth(), cam.getHeight(), OF_IMAGE_COLOR);
            imgMat = ofxCv::toCv(img);
            }
        curFlow = &farneback;
        farneback.setPyramidScale( panel.getValueF("pyrScale") );
        farneback.setNumLevels( panel.getValueF("levels") );
        farneback.setWindowSize( panel.getValueF("winsize") );
        farneback.setNumIterations( panel.getValueF("iterations") );
        farneback.setPolyN( panel.getValueF("polyN") );
        farneback.setPolySigma( panel.getValueF("polySigma") );

        curFlow->calcOpticalFlow(img);
        
        ofxCv::blur(img,10);
        contourFinder.findContours(img);
        brush = getContour(&contourFinder);
        
        /******************************
                FLOW PAINTING
         ******************************/
            tex.begin();
            //ofClear(0,delay);
            //            curFlow->draw(0,0,tex.getWidth(),tex.getHeight());

            strokeRandom = panel.getValueF("strokeRandom");
        
            for(int x=0;x<farneback.getWidth()-(density);x+=density) {
                for(int y=0;y<farneback.getHeight()-(density);y+=density) {
                    int fftIndex =  
                    (floor(
                           x  * (drawBins.size() 
                                 / farneback.getWidth()))
                     );
                    
                    fftIndex %= farneback.getWidth();
                    ofVec2f src = ofVec2f(x,y) + ofVec2f(ofRandom(-strokeRandom,strokeRandom),ofRandom(-strokeRandom,strokeRandom));
                    ofVec2f dst = farneback.getFlowPosition(x,y) + ofVec2f(ofRandom(-strokeRandom,strokeRandom),ofRandom(-strokeRandom,strokeRandom));
                    
                    ofSetLineWidth((5 + sqrt(drawBins[fftIndex])));
                    ofSetColor(src.distance(dst),(sqrt(drawBins[fftIndex])*255),170);
                        // ofSetColor(255);
                    ofLine(src*2,dst*2);
                    ofSetColor(255);
                    ofSetLineWidth(1);
                    
                    
                }
            }
            ofClearAlpha();
            tex.end();
        overallRotation += ofVec3f();
        
        
        overallRotation += farneback.getAverageFlowInRegion(ofRectangle(0,0,farneback.getWidth()/2,farneback.getHeight()));
        
        overallRotation -= farneback.getAverageFlowInRegion(ofRectangle(farneback.getWidth()/2,0,farneback.getWidth()/2,farneback.getHeight()));
    }
    
    soundMutex.lock();
    drawBuffer = middleBuffer;
    drawBins = middleBins;
    soundMutex.unlock();
    
    primary.clear();
    
    box2d.update();	
    ofVec2f mouse(ofGetMouseX(), ofGetMouseY());
        //float minDis = ofGetMousePressed() ? 500 : 100;
    
    float minDis = 500 * (maxValue + .2);
    
    ofVec3f center = ofVec3f(ofGetWidth()/2,ofGetHeight()/2,0);
    
    for(int i=1; i<circles.size(); i++) {
        float dis = center.distance(circles[i].getPosition());
        if(dis < minDis) circles[i].addRepulsionForce(center, 9*scaledVol);
        else circles[i].addAttractionPoint(center, (9.0*scaledVol)+2);
        
        primary.addVertex(circles[i].getPosition());
    }
    
    int count=0;
    for(int i=0;i<primary.size();i++) {
        
        if(drawBins.size()>primary.size())
            count+=floor(drawBins.size()/primary.size());
        else count++;
        
        /******************************
            RANDOM CRAZY MESH YO
         ******************************/
            mesh.clear();
            mesh.enableTextures();
            mesh.enableNormals();
            mesh.enableColors();
            mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
            for(int i = 1; i < primary.size(); i++) {
                ofVec3f thisPoint = primary[i-1];
                ofVec3f nextPoint = primary[i];
                mesh.addColor(drawBins[count]);
                mesh.addVertex(thisPoint * (overallRotation * panel.getValueF("impact")+1 ));
                if(i%1) {
                    mesh.addVertex(nextPoint* (overallRotation * panel.getValueF("impact")+1));
                    mesh.addColor(drawBins[count]);
                    mesh.addTexCoord(ofVec2f((i-1)/primary.size() * tex.getWidth(),1.0f * tex.getHeight()));                    
                }
                mesh.addTexCoord(ofVec2f((i-1)/primary.size() * tex.getWidth(),1.0f * tex.getHeight()));
                mesh.addNormal(ofVec3f(0,0,1.0f));
            }        
    }
}

//--------------------------------------------------------------
ofPolyline testApp::getContour(ofxCv::ContourFinder * _contourFinder) {
    ofPolyline poly;
    
    if(_contourFinder->size() != 0 ) {
        vector<ofPolyline> polylines;
        polylines = _contourFinder->getPolylines();
        for(int i=0; i<polylines.size(); i++) {
            if(i==0) poly = polylines[i];
                // if(polylines[i].getArea() > 20)
            if(polylines[i].size() >= poly.size())
                poly = polylines[i];
                // poly.addVertices(polylines[i].getVertices());
                //    }
            
        }
    } 
    poly.close();    
        //poly.simplify(.3);    
    return poly;
}

//--------------------------------------------------------------
void testApp::draw(){
    ofBackgroundGradient(ofColor(255),ofColor(210) );
    ofSetColor(255);

    ofPushMatrix();
    ofTranslate(200,0);

    glDisable(GL_DEPTH_TEST);

    /******************************
     SUPER FUN CIRCLE THING
     ******************************/
    
    ofPushMatrix();
        ofTranslate(mesh.getCentroid());
        ofRotate(overallRotation.x, 0, 1, 0);
        ofSetColor(255);
        ofNoFill();
        for(int x=0;x<farneback.getWidth()-(density);x+=density) {
            for(int y=0;y<farneback.getHeight()-(density);y+=density) {
                ofPushMatrix();
                
                int index = x + ( y * farneback.getWidth() );
                
                ofVec2f dst = farneback.getFlowPosition(x, y);
                    // ofLine(ofVec2f(x,y), farneback.getFlowPosition(x, y));
                
                ofQuaternion srcX, srcY, dstX, dstY, spinQuat;

                srcX.makeRotate(x, 1, 0, 0);
                srcY.makeRotate(y, 0, 1, 0);
                
                dstX.makeRotate(dst.x, 1, 0, 0);
                dstY.makeRotate(dst.y, 0, 1, 0);
                
                spinQuat.makeRotate(ofGetFrameNum(), 0, 1, 0);
                
                int fftIndex =  
                        (floor(
                          x  * (drawBins.size() 
                                / farneback.getWidth()))
                        );
                
                fftIndex %= farneback.getWidth();
                
                ofVec3f center = ofVec3f(0,0,
                                sphereRadius * (1 + sqrt(drawBins[fftIndex])));
                
                ofVec3f srcPt = srcX * srcY * spinQuat * center;
                ofVec3f dstPt = dstX * dstY * spinQuat * center;
                
                srcPt += ofVec2f(ofRandom(-strokeRandom,strokeRandom),ofRandom(-strokeRandom,strokeRandom));
                dstPt += ofVec2f(ofRandom(-strokeRandom,strokeRandom),ofRandom(-strokeRandom,strokeRandom));
                
                ofSetColor(srcPt.distanceSquared(dstPt));
                ofSetLineWidth(srcPt.distanceSquared(dstPt) * panel.getValueF("lineWidth"));
                ofLine(srcPt, dstPt);
                ofSetLineWidth(1);
                
                ofPopMatrix();
                
            }
        }
    ofPushMatrix();
    ofRotateY(ofGetFrameNum());
        //glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    ofDisableArbTex();
    ofFill();       ofSetColor(255,10);
    
    tex.getTextureReference().bind();
    
    toon.begin();
    toon.setUniform2f("center",tex.getWidth()/2, tex.getHeight()/2);
    toon.setUniform1i("tex",0 );   
    
    toon.setUniform1f("sigma", 0.33f); 
    toon.setUniform1f("blurSize", 0.1f);
    toon.setUniform1f("k", 0.06f);
    
        //    ofSphere(0, 0, 0, sphereRadius);
    
    quadric = gluNewQuadric();
    gluQuadricTexture(quadric, GL_TRUE);
    gluQuadricNormals(quadric, GLU_SMOOTH);
    gluSphere(quadric, sphereRadius, 100, 100);
    toon.end();
    
    tex.getTextureReference().unbind();
    glDisable(GL_TEXTURE_2D);
    ofEnableArbTex();
    ofPopMatrix();
    ofPopMatrix();
    
            //  tex.getTextureReference().bind();
        
        toon.begin();
        mesh.draw();
        mesh.drawWireframe();
        
            //            toon.end();
            // tex.getTextureReference().unbind();
        
            //    camera.end();
    
        for(int i=0; i<circles.size(); i++) {
                //            ofSetColor(((i/circles.size())*255)%255,255,255);
            
            ofFill();
            ofSetColor(sqrt(drawBins[i])*255,sqrt(drawBins[i])*255,sqrt(drawBins[i])*255);
            ofCircle(circles[i].getPosition().x, circles[i].getPosition().y, circles[i].getRadius() * (1 + sqrt(drawBins[i])));
            
            primary.getSmoothed(panel.getValueI("smoothed")*(sqrt(drawBins[i])),0).draw();
            
        }   
            toon.end();

    ofPopMatrix();
    glDisable(GL_DEPTH_TEST);
    /******************************
            DEBUG LIKE A MOFO
     ******************************/
    if(debug) {
        soundMutex.lock();
        drawBins = middleBins;
        soundMutex.unlock();
        ofSetColor(255);
        ofPushMatrix();
        ofTranslate(16, 16);
        if(kinect.isConnected())
            kinect.drawDepth(plotHeight+10,10,320,240);
        else img.draw(plotHeight+10,10,320,240);
        tex.draw(plotHeight+10+320,10,320,240);
        curFlow->draw(plotHeight+10,10,320,240);
        ofDrawBitmapString("Frequency Domain", 0, 0);
        plot(drawBins, -plotHeight, plotHeight / 2);
        
        ofSetColor(225);
        ofTranslate(10,350);
        ofDrawBitmapString("current flow: x" + ofToString(farneback.getTotalFlow().x) + " y" + ofToString(farneback.getTotalFlow().y), 4,18 );
        ofTranslate(0,18);    
        ofDrawBitmapString("Overall flow: x" + ofToString(overallRotation.x) + " y" + ofToString(overallRotation.y), 4,18 );

        ofPopMatrix();
    }
    
}

void testApp::plot(vector<float>& buffer, float scale, float offset) {
    ofNoFill();
    int n = buffer.size();
    ofRect(0, 0, n, plotHeight);
    glPushMatrix();
    glTranslatef(0, plotHeight / 2 + offset, 0);
    ofBeginShape();
    for (int i = 0; i < n; i++) {
        ofVertex(i, buffer[i] * scale);
    }
    ofEndShape();
    glPopMatrix();
}


void testApp::audioReceived(float* input, int bufferSize, int nChannels) {	
 

		memcpy(&audioBuffer[0], input, sizeof(float) * bufferSize);
		
		maxValue = 0;
		for(int i = 0; i < bufferSize; i++) {
        if(abs(audioBuffer[i]) > maxValue) {
            maxValue = abs(audioBuffer[i]);
        }
		}
		for(int i = 0; i < bufferSize; i++) {
        audioBuffer[i] /= maxValue;
		}

    fft->setSignal(&audioBuffer[0]);
    
    float* curFft = fft->getAmplitude();
    memcpy(&audioBins[0], curFft, sizeof(float) * fft->getBinSize());
    
    for(int i = 0; i < fft->getBinSize(); i++) {
        if(abs(audioBins[i]) > maxValue) {
            maxValue = abs(audioBins[i]);
        }
    }
    for(int i = 0; i < fft->getBinSize(); i++) {
        audioBins[i] /= maxValue;
    }
    
    soundMutex.lock();
    middleBuffer = audioBuffer;
    middleBins = audioBins;
    soundMutex.unlock();

}

//--------------------------------------------------------------
void testApp::keyPressed(int key){
    switch(key) {
        case ' ':
            if(debug) debug=false;
            else debug=true;
            break;
        case 's':
            mic.start();
            break;
        case 'a':
            mic.stop();
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