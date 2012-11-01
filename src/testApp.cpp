#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    ofSetVerticalSync(true);
    //    ofEnableSmoothing();
    
    if(kinect.isConnected()) {
        kinect.setRegistration(false);
        kinect.init(false, false, true);
        kinect.open(0);
    } else {
        cam.initGrabber(320,240);
    }
    
    camera.setPosition(ofVec3f(0,0,200));
    camera.setNearClip(100);
    camera.lookAt(ofVec3f(0,0,0));
    
    tex.allocate(640,480,GL_RGBA,1);
    
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
    panel.addSlider("density",5,1,10,true);
    panel.addSlider("bounce",5,0,2,false);
    panel.addSlider("friction",5,0,2,false);
    panel.addSlider("attr",1,0,1,false);
    panel.addSlider("sphereRad",300,10,500,true);
    panel.addSlider("lineWidth",1,0,1,false);
    
    panel.addLabel("Flow");
    panel.addSlider("pyrScale", .5, 0, 1);
    panel.addSlider("levels", 4, 1, 8, true);
    panel.addSlider("winsize", 8, 4, 64, true);
    panel.addSlider("iterations", 2, 1, 8, true);
    panel.addSlider("polyN", 7, 5, 10, true);
    panel.addSlider("polySigma", 1.5, 1.1, 2);
    
    
    box2d.init();
    box2d.setGravity(0, 0);
    box2d.setFPS(30.0);
    
    string fragshader = "\
    varying vec3 vNormal;\
    varying vec3 vVertex;\
    #define shininess 20.0\
    void main (void)\
    {\
        vec4 color0 = vec4(0.8, 0.0, 0.0, 1.0);\
        vec4 color1 = vec4(0.0, 0.0, 0.0, 1.0);\
        vec4 color2 = vec4(0.8, 0.0, 0.0, 1.0);\
        vec3 eyePos = vec3(0.0,0.0,5.0);\
        vec3 lightPos = vec3(0.0,5.0,5.0);\
        vec3 Normal = normalize(gl_NormalMatrix * vNormal);\
        vec3 EyeVert = normalize(eyePos - vVertex);\
        vec3 LightVert = normalize(lightPos - vVertex);\
        vec3 EyeLight = normalize(LightVert+EyeVert);\
        float sil = max(dot(Normal,EyeVert), 0.0);\
        if (sil < 0.3) gl_FragColor = color1;\
        else\
        {\
            gl_FragColor = color0;\
            float spec = pow(max(dot(Normal,EyeLight),0.0), shininess);\
            if (spec < 0.2) gl_FragColor *= 0.8;\
            else gl_FragColor = color2;\
            float diffuse = max(dot(Normal,LightVert),0.0);\
            if (diffuse < 0.5) gl_FragColor *=0.8;\
        }\
    }\
    ";
    
    string vertshader = "\
    varying vec3 vNormal;\
    varying vec3 vVertex\
    void main(void)\
    {\
        vVertex = gl_Vertex.xyz;\
        vNormal = gl_Normal;\
        gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\
    }\
    ";
    
    toon.setupShaderFromSource(GL_VERTEX_SHADER, vertshader);
    toon.setupShaderFromSource(GL_FRAGMENT_SHADER, fragshader);
    
		
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
    mic.stop();
}

//--------------------------------------------------------------
void testApp::update(){
    scaledVol = panel.getValueF("attr");
    sphereRadius = panel.getValueI("sphereRad");
    density = panel.getValueI("density");
    
    if(kinect.isConnected())
        kinect.update();
    else
        cam.update();
    
    if(kinect.isFrameNew() || cam.isFrameNew()) {
        if(kinect.isConnected()) {
            img.setFromPixels(kinect.getDepthPixels(), kinect.getWidth(), kinect.getHeight(), OF_IMAGE_GRAYSCALE);
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
        farneback.setUseGaussian(panel.getValueB("OPTFLOW_FARNEBACK_GAUSSIAN"));

            curFlow->calcOpticalFlow(img);
        
            tex.begin();
            ofClear(0);
            curFlow->draw(0,0,tex.getWidth(),tex.getHeight());
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
void testApp::draw(){
    ofBackgroundGradient(ofColor(255),ofColor(210) );
    ofSetColor(255);

    ofPushMatrix();
    ofTranslate(200,0);

    glDisable(GL_DEPTH_TEST);

        ofPushMatrix();
            ofTranslate(mesh.getCentroid());
            ofRotate(overallRotation.x, 0, 1, 0);
            ofSetColor(overallRotation.x);
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
            
            ofSetColor(srcPt.distanceSquared(dstPt));
            ofSetLineWidth(srcPt.distanceSquared(dstPt) * panel.getValueF("lineWidth"));
            ofLine(srcPt, dstPt);
            ofSetLineWidth(1);
            
            ofPopMatrix();
            
        }
    }

    
    
        //ofSphere(0, 0, 0, sphereRadius);
            ofFill();
        ofPopMatrix();
    
        ofSetColor(255);
            //  tex.getTextureReference().bind();
        
            //            toon.begin();
        mesh.draw();
        mesh.drawWireframe();
        
            //            toon.end();
            // tex.getTextureReference().unbind();
        
            //    camera.end();
    
        for(int i=0; i<circles.size(); i++) {
                //            ofSetColor(((i/circles.size())*255)%255,255,255);
            
            ofFill();
            ofCircle(circles[i].getPosition().x, circles[i].getPosition().y, circles[i].getRadius() * (.5 + sqrt(drawBins[i])));
            
            primary.getSmoothed(panel.getValueI("smoothed")*(sqrt(drawBins[i])),0).draw();
            
        }        

    ofPopMatrix();
    glDisable(GL_DEPTH_TEST);
    
    if(debug) {
        soundMutex.lock();
        drawBins = middleBins;
        soundMutex.unlock();
        ofSetColor(255);
        ofPushMatrix();
        ofTranslate(16, 16);
        img.draw(plotHeight+10,10,320,240);
        curFlow->draw(plotHeight+10,10,320,240);
        ofDrawBitmapString("Frequency Domain", 0, 0);
        plot(drawBins, -plotHeight, plotHeight / 2);
        
        ofSetColor(225);
        ofTranslate(10,300);
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