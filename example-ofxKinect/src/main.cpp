#include "ofMain.h"
#include "ofApp.h"
#include "ofAppGLFWWindow.h"

//========================================================================
int main( ){
//	ofSetupOpenGL(1024,768,OF_WINDOW);			// <-------- setup the GL context

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
//	ofRunApp(new ofApp());
	ofGLFWWindowSettings settings;
	settings.width = 1024;
	settings.height = 768;
	settings.setPosition(ofVec2f(0,0));
	settings.resizable = true;
	settings.title = "Main";
	shared_ptr<ofAppBaseWindow> mainWindow = ofCreateWindow(settings);
    
	settings.width = 800;
	settings.height = 600;
	settings.setPosition(ofVec2f(0,0));
	settings.title = "Proj";
	settings.resizable = true;
	// uncomment next line to share main's OpenGL resources with gui
	//settings.shareContextWith = mainWindow;
	shared_ptr<ofAppBaseWindow> guiWindow = ofCreateWindow(settings);
	guiWindow->setVerticalSync(false);
    
	shared_ptr<ofApp> mainApp(new ofApp);
//	mainApp->setupGui();
	ofAddListener(guiWindow->events().draw,mainApp.get(),&ofApp::drawProj);
    
	ofRunApp(mainWindow, mainApp);
	ofRunMainLoop();

}
