#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	//ofDisableAntiAliasing();
	//ofBackgroundHex(0xfdefc2);
	ofBackground(0);
	ofSetLogLevel(OF_LOG_NOTICE);
	ofSetVerticalSync(true);

	// Box2d initialisation
	box2d.init();
	box2d.enableEvents();   // <-- turn on the event listener
	box2d.setGravity(0, 5);
	box2d.createGround();
	box2d.createBounds(0.0f, 0.0f, kinect.width, kinect.height);
	box2d.setFPS(60.0);
	box2d.registerGrabbing();

	// register the box2D listener so that we get the events
	//ofAddListener(box2d.contactStartEvents, this, &ofApp::contactStart);
	//ofAddListener(box2d.contactEndEvents, this, &ofApp::contactEnd);

	// load the lines we saved as a text file...
	/*
	ifstream f;
	f.open(ofToDataPath("lines.txt").c_str());
	vector <string> strLines;
	while (!f.eof()) {
		string ptStr;
		getline(f, ptStr);
		strLines.push_back(ptStr);
	}
	f.close();

	for (int i = 0; i < strLines.size(); i++) {
		vector <string> pts = ofSplitString(strLines[i], ",");
		if (pts.size() > 0) {
			auto edge = make_shared<ofxBox2dEdge>();
			for (int j = 0; j < pts.size(); j += 2) {
				if (pts[j].size() > 0) {
					float x = ofToFloat(pts[j]);
					float y = ofToFloat(pts[j + 1]);
					edge->addVertex(x, y);
				}
			}
			edge->create(box2d.getWorld());
			edges.push_back(edge);
		}
	}*/

	ofSetLogLevel(OF_LOG_VERBOSE);
	// enable depth->video image calibration
	kinect.setRegistration(true);
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)

	kinect.open();        // opens first available kinect
	//kinect.open(1);    // open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");    // open a kinect using it's unique serial #

	// print the intrinsic IR sensor values
	if (kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}

#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	// set up CV image buffers we are going to use
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	// set up initial values for kinect sensor
	nearThreshold = 230;
	farThreshold = 70;
	//nearThreshold = 100;
	//farThreshold = 1062;
	bThreshWithOpenCV = true;

	ofSetFrameRate(60);

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);

	// start the display from the front showing debug vews from kinect sensor
	bDrawPointCloud = false;
	bDrawDebug = true;
}

//--------------------------------------------------------------
void ofApp::update() {
	// add some circles every so often
	
	if ((int)ofRandom(0, 20) == 0) {
		auto c = make_shared<ofxBox2dCircle>();
		c->setPhysics(0.2, 0.2, 0.002);
		//c->setup(box2d.getWorld(), ofRandom(20, 400), -20, ofRandom(3, 20)); // create new particle with size, start position
		c->setup(box2d.getWorld(), ofRandom(0, kinect.width-10), 0, 5); // create new particle with size, start position
		c->setVelocity(0, 15); // drop them down
		circles.push_back(c);
	}
	box2d.update();
	kinect.update();

	// there is a new frame and we are connected to a kinect sensor
	if (kinect.isFrameNew()) {
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels());
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if (bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		}
		else {
			// or we do it ourselves - show people how they can work with the pixels
			ofPixels & pix = grayImage.getPixels();
			int numPixels = pix.size();
			for (int i = 0; i < numPixels; i++) {
				if (pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				}
				else {
					pix[i] = 0;
				}
			}
		}
		// update the cv images
		grayImage.flagImageChanged();
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height) / 2, 20, false);
	}
	faceToLine();

	// Turn off gravity
	/*
	box2d.setGravity(0, 0);

		//set gravity to specific circle
	centerBody.setup(box2d.getWorld(), kinect.width / 2, kinect.height / 2, 10); // Center of the screen
	float G = 1; // modifier of gravity value - you can make it bigger to have stronger gravity

	for (auto &circle : circles) {
		ofVec2f direction = centerBody.getPosition() - circle->getPosition();
		float distance = direction.length();
		float forceValue = G / (distance * distance);

		// Normalize the direction vector
		if (distance > 0) {
			direction /= distance;
		}

		// Apply force to the circle
		circle->addForce(direction * forceValue, 1.0);
	}*/
}

//--------------------------------------------------------------
void ofApp::draw() {

	string info = "";
	info += "FPS: " + ofToString(ofGetFrameRate()) + "\n";

	//Draw kinnect image
	kinect.drawDepth(0, ofGetHeight()-320, 400, 300);
	grayImage.draw(410, ofGetHeight()-320, 400, 300); // and the threshholded view from openCV

	//Draw framerate
	//if (bdrawDebug)
		ofDrawBitmapString(info, 0, ofGetHeight()-10);

	for (auto &circle : circles) { // draw all the box2D circles that are on the screen
		//ofFill();
		//ofSetHexColor(0xc0dd3b);
		ofFill();
		ofSetColor(255);
		circle->draw();
	}

	ofSetColor(255); // draw the lines that we know of
	if (bDrawDebug)
	{
		
		ofSetLineWidth(2);
		ofNoFill();
		for (auto &line : lines) {
			line.draw();
		}
		for (auto & edge : edges) {
			edge->draw();
		}
	}

	//contourFinder.draw(10, 10, 400, 300); // and draw the contours over it
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

	/*
	 // want to save out some line...
	 if(key == ' ') {
	 ofstream f;
	 f.clear();
	 f.open(ofToDataPath("lines.txt").c_str());
	 for (int i=0; i<lines.size(); i++) {
	 for (int j=0; j<lines[i].size(); j++) {
	 float x = lines[i][j].x;
	 float y = lines[i][j].y;
	 f << x << "," << y << ",";
	 }
	 f << "\n";
	 }
	 f.close();lines.clear();
	 }*/

	

	switch (key) {

	//case '1': //Add more circles
		//c->setPhysics(1, 0.5, 0.5);
		//c->setup(box2d.getWorld(), mouseX, mouseY, 10);
    	//circles.push_back(c);
		//break;
	//case 'g':
	//	bGravity = !bGravity;
	//	break;

	case 'f':
		ofToggleFullscreen();
		break;
	case 'd':
		bDrawDebug = !bDrawDebug;
		break;

	case '>':
	case '.':
		farThreshold++;
		if (farThreshold > 255) farThreshold = 255;
		break;

	case '<':
	case ',':
		farThreshold--;
		if (farThreshold < 0) farThreshold = 0;
		break;

	case '+':
	case '=':
		nearThreshold++;
		if (nearThreshold > 255) nearThreshold = 255;
		break;

	case '-':
		nearThreshold--;
		if (nearThreshold < 0) nearThreshold = 0;
		break;

	case 'w': //Invert depth
		kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
		break;
	case 'c': //Clear all circles
		circles.clear();
		break;

	case OF_KEY_UP:
		angle++;
		if (angle > 30) angle = 30;
		kinect.setCameraTiltAngle(angle);
		break;

	case OF_KEY_DOWN:
		angle--;
		if (angle < -30) angle = -30;
		kinect.setCameraTiltAngle(angle);
		break;

	default:
		break;
	}
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
	lines.back().addVertex(x, y);
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	 
	if (button == 0) //left
	{
		//// Generate circles at random positions at the top of the screen
		//for (int i = 0; i < 10; i++) { // Adjust the number of circles as needed
		//	circles.push_back(make_shared<ofxBox2dCircle>());
		//
		//	// Density is used in conjunction with other objects to figure out how the mass should be distributed in a parent body.
		//	// Bounce is exactly what it sounds like: it tells the object how much to bounce when it hits another object.
		//	// Friction is used to determine how much two objects slow down when they’re sliding against each other.
		//	circles.back()->setPhysics(1000, 0, 0); // density, bounce, and friction
		//
		//	// Randomly position the circle at the top of the screen
		//	circles.back()->setup(box2d.getWorld(), kinect.width/2, kinect.height/2, 5);
		//}

		
		circles.push_back(make_shared<ofxBox2dCircle>());
		
		//Density is used in conjuction with other objects to figure out how the mass should be distributed in a parent body.
		//Bounce is exactly what it sounds like : it tells the object how much to bounce when it hits another object.
		//Friction is used to determine how much two objects slow down when they’re sliding against each other.
		circles.back()->setPhysics(1000, 0, 0); //density, bounce and friction

		circles.back()->setup(box2d.getWorld(), mouseX, mouseY, 5);
		
	}

	//lines.push_back(ofPolyline());
	//lines.back().addVertex(x, y);

	//Add more circles
	//auto c = make_shared<ofxBox2dCircle>();
	//c->setPhysics(1, 0.5, 0.5);
	//c->setup(box2d.getWorld(), mouseX, mouseY, 10);
	//c->circles.push_back(c);

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
	//auto edge = make_shared<ofxBox2dEdge>();
	//lines.back().simplify();
	//for (int i = 0; i < lines.back().size(); i++) {
	//	edge->addVertex(lines.back()[i]);
	//}
	//// edge->setPhysics(1, .2, 1);  // uncomment this to see it fall!
	//edge->create(box2d.getWorld());
	//edges.push_back(edge);
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {
}
//--------------------------------------------------------------

void ofApp::faceToLine() {
	lines.clear();
	edges.clear();

	for (int i = 0; i < contourFinder.nBlobs; i++) {
		lines.push_back(ofPolyline());

		for (int p = 0; p < contourFinder.blobs[i].nPts; p++) {
			lines.back().addVertex(contourFinder.blobs[i].pts[p]);
		}
		auto edge = make_shared<ofxBox2dEdge>();
		lines.back().simplify();
		for (int i = 0; i < lines.back().size(); i++) {
			edge->addVertex(lines.back()[i]);
		}
		edge->create(box2d.getWorld());
		edges.push_back(edge);
	}
}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void ofApp::contactStart(ofxBox2dContactArgs &e) {
	if (e.a != NULL && e.b != NULL) {
		// if we collide with the ground we do not
		// want to play a sound. this is how you do that
		if (e.a->GetType() == b2Shape::e_circle && e.b->GetType() == b2Shape::e_circle) {
			//cout << "hit" << endl;
			//When circle gets in contact with other circle
		}
	}
}

//--------------------------------------------------------------
void ofApp::contactEnd(ofxBox2dContactArgs &e) {
	if (e.a != NULL && e.b != NULL) {
		//When circle is not in contact anymore
	}
}