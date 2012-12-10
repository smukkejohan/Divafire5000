#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
	//ofSetLogLevel(OF_LOG_VERBOSE);
    
    lastCmdTime = ofGetElapsedTimeMillis();
    
    pan = 100;
    tilt = 100;
    focus = 240;
    
    arduino.setup("/dev/tty.usbmodemfa131", 9600);
    
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init(true);
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 255;
	farThreshold = 4;
	bThreshWithOpenCV = true;
    	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	//angle = 0;
	//kinect.setCameraTiltAngle(angle);
	
}

//--------------------------------------------------------------
void testApp::update() {
	
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage.getPixels();
			
			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 40, (kinect.width*kinect.height)/2, 20, true);
        
        
        
        // largest blob is the one we track 
        // this should maybe be acceleration also
        for(int i = 0; i < contourFinder.nBlobs; i++) {
            ofxCvBlob* blob;
            blob = &contourFinder.blobs.at(i);
            
            if (!trackedBlob) {
                trackedBlob = blob;
            }
            
            if (blob->area > trackedBlob->area) {
                trackedBlob = blob;
                
            }
        }
        
        
        
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}



//--------------------------------------------------------------
void testApp::draw() {
	
	ofSetColor(255, 255, 255);
	
		// draw from the live kinect
    //kinect.drawDepth(10, 10, 400, 300);
    //kinect.draw(420, 10, 400, 300);
		
    grayImage.draw(10, 10);
        
    ofNoFill();
    
        for(int i = 0; i < contourFinder.nBlobs; i++) {
            
            ofxCvBlob* blob;
            blob = &contourFinder.blobs.at(i);
            
            ofRectangle r = blob->boundingRect;
            r.x += 10; r.y += 10;
            
            ofSetLineWidth(1);
            ofSetColor(200);
            
            if (blob == trackedBlob) {
                ofSetLineWidth(3);
                ofSetColor(255);
            }
            
            ofRect(r);
        }
    
    

    
    if (trackedBlob) {
        
        
        int x = ofMap(trackedBlob->centroid.x - 70, 0, 600, 150, 0);
        int y = 5;
        int z = ofMap(trackedBlob->centroid.y, 250, 400, -30, 12);
      //int z = 20;
        //cout << trackedBlob->centroid.y << endl;
        
        
      /*
        int x = mouseX- 320;
        int y = mouseY - 240;
        int z = 400;
        */
        
        /*
        int x = mouseX - 320;
        int y = 0;
        int z = mouseY-300;*/
        
         
        if (x != 0) {
            
            ofVec3f vec = ofVec3f(x,y,z);
            
            ofVec3f lamp = ofVec3f(0,-200,0);
            
            vec = vec-lamp;
           // vec.normalize();
            
            float panAngle = atan2(vec.z, vec.x);
            float tiltAngle = asin(vec.y/vec.length());
            
           // cout<<tiltAngle<<endl;
            tiltAngle = HALF_PI - (tiltAngle - HALF_PI);
            panAngle += PI;
            //cout<<panAngle<<endl;
            //panAngle += PI;
            
            ofPushMatrix();
            ofTranslate(50, 50);
            ofRotate(panAngle * RAD_TO_DEG, 0, 0, 1);
            ofRect(0, 0, 100, 10);
            ofPopMatrix();

            ofPushMatrix();
            ofTranslate(50, 150);
            ofRotate(tiltAngle * RAD_TO_DEG, 0, 0, 1);
            ofRect(0, 0, 100, 10);
            ofPopMatrix();
            
            pan = ofMap(panAngle, 0, TWO_PI, 0, 255); 
            tilt = ofMap(tiltAngle, 0, PI, 30, 255-30);
            
            //printf("%i, %i pan: %f tilt: %f\n", pan, tilt, atan2(z, x),acos(y/vec.length()));
            
            /*vec_set(temp, your.x)
             vec_sub(temp, my.x)
             vec_to_angle(my.pan, temp)*/
            
            /*
            pan = ofMap(mouseX, 0, ofGetWidth(), 0, 255);
            tilt = ofMap(mouseY, 0, ofGetHeight(), 0, 255);
            cout<<tilt<<endl;*/
            //tilt = ofMap
            
            //pan = 63;
            //tilt = 255;
            
            setPan();
            setTilt();
            
        }
    }
    
    
    
    
    //contourFinder.draw(10, 320, 400, 300);
		
#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	
	
	// draw instructions
    
    stringstream reportStream;
    
    
	ofSetColor(255, 255, 255);
        
    reportStream << "Pan:" << pan << ", tilt: " << tilt << endl;  
        
    if (trackedBlob) {
        
	reportStream << "Tracking blob at x: " << trackedBlob->centroid.x << " y: " << trackedBlob->centroid.y << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
	<< ofToString(kinect.getMksAccel().y, 2) << " / "
	<< ofToString(kinect.getMksAccel().z, 2) << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
	<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
	
    }
        
    ofDrawBitmapString(reportStream.str(),20,652);
        
}

//--------------------------------------------------------------
void testApp::exit() {
	//kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {  
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case OF_KEY_UP:
            
            focus += 1;
            setFocus();

			break;
			
		case OF_KEY_DOWN:
            focus -= 1;
            setFocus();
			break;
            
        case OF_KEY_LEFT:
            pan += 1;
            setPan();
            
			break;
			
		case OF_KEY_RIGHT:
            pan -= 1;
            setPan();
			break;
	}
}



void testApp::sendDmx(int channel, int value) {
    
    while (ofGetElapsedTimeMillis() - lastCmdTime < 90) {
        ofSleepMillis(5);
    }
    
    string s = ofToString(channel) + "c" + ofToString(value) + "w";
    arduino.writeBytes((unsigned char*)s.c_str(), s.length());
    
    lastCmdTime = ofGetElapsedTimeMillis();
}


void testApp::setPan() {   
//    cout<<pan<<endl;
    if (pan < 0) {
        pan = 0;
    } else if (pan > 255) {
        pan = 255;
    }
    sendDmx(1, pan);
}

void testApp::setTilt() {
    if (tilt < 0) {
        tilt = 0;
    } else if (tilt > 255) {
        tilt = 255;
    }    
    sendDmx(3, tilt);
    
    focus = ofRandom(255);
    setFocus();
}

void testApp::setFocus() {    
    if (focus < 0) {
        focus = 0;
    } else if (focus > 255) {
        focus = 255;
    }
    sendDmx(6, focus); 
}


//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
