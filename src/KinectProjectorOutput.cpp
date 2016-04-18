/** OFXKINECTPROJECTORCALIBRATION **/
/** work in progress, not even beta! **/
/** Kj1, www.hangaar.net **/

#include "KinectProjectorOutput.h"

using namespace ofxCv ;
using namespace cv;

KinectProjectorOutput::KinectProjectorOutput() {	
	projectorResolutionX = 800;
	projectorResolutionY = 600;
	reprojError = -1;
	isReady = false;
}


void	KinectProjectorOutput::setup(RGBDCamCalibWrapper* _kinect, int _projectorResolutionX, int _projectorResolutionY) {
	kinect = _kinect;
	projectorResolutionX = _projectorResolutionX;
	projectorResolutionY = _projectorResolutionY;
}

bool KinectProjectorOutput::isCalibrationReady() {
	return isReady;
}

ofPoint KinectProjectorOutput::projectFromDepthXYZ(const ofPoint o) const {
	if (!isReady) return ofPoint(0,0);
	ofPoint ptWorld = kinect->getWorldFromRgbCalibratedXYZ(o, mirrorHoriz, mirrorVert);
	vector<Point3f> vo;
	vo.push_back(Point3f(ptWorld.x, ptWorld.y, ptWorld.z));
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;		
	projectPoints(vo, mr, mt, cameraMatrix, distCoeffs, projected);
	return toOf(projected[0]);
}

ofPoint KinectProjectorOutput::projectFromDepthXY(const ofPoint o) const {
	if (!isReady) return ofPoint(0,0);
	ofPoint ptWorld = kinect->getWorldFromRgbCalibrated(o, mirrorHoriz, mirrorVert);
	vector<Point3f> vo;
	vo.push_back(Point3f(ptWorld.x, ptWorld.y, ptWorld.z));
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];
	vector<Point2f> projected;
	projectPoints(vo, mr, mt, cameraMatrix, distCoeffs, projected);
	return toOf(projected[0]);
}

//vector<ofVec2f> KinectProjectorCalibration::project(vector<Point3f> wrldSrc, int i) const {
//	if (!calibrated)  { vector<ofVec2f> v; v.push_back(ofVec2f(0,0)); return v; }
//	Mat mt = boardTranslations[0];
//	Mat mr = boardRotations[0];
//	vector<Point2f> projected;
//	projectPoints(wrldSrc, mr, mt, cameraMatrix, distCoeffs, projected);
//	vector<ofVec2f> projectedOF;
//	for (int i = 0; i < projected.size(); i++) {
//		projectedOF.push_back(toOf(projected[i]));
//	}
//	return projectedOF;
//}

ofVec2f KinectProjectorOutput::project(const Point3f o) const {
	if (!isReady) return ofVec2f(0,0);
	vector<cv::Point3f> vo(1, o);	
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;		
	projectPoints(Mat(vo), mr, mt, cameraMatrix, distCoeffs, projected);	
	return toOf(projected[0]);
}

vector<ofVec2f> KinectProjectorOutput::project(vector<Point3f> wrldSrc, int i) const {
	if (!isReady)  { vector<ofVec2f> v; v.push_back(ofVec2f(0,0)); return v; }
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;		
	projectPoints(wrldSrc, mr, mt, cameraMatrix, distCoeffs, projected);	
	vector<ofVec2f> projectedOF;
	for (int i = 0; i < projected.size(); i++) {
		projectedOF.push_back(toOf(projected[i]));
	}
	return projectedOF;
}

vector<ofPoint> KinectProjectorOutput::project(vector<ofPoint> wrldSrc, int i) const {
	if (!isReady)  { vector<ofPoint> v; v.push_back(ofPoint(0,0)); return v; }
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;		
	vector<Point3f> src = toCv(wrldSrc);		
	projectPoints(src, mr, mt, cameraMatrix, distCoeffs, projected);	
	vector<ofPoint> projectedOF;
	for (int i = 0; i < projected.size(); i++) {
		projectedOF.push_back(toOf(projected[i]));
	}
	return projectedOF;
}

void	KinectProjectorOutput::loadCalibratedView(){
	if (!isReady) return;
	glPushMatrix();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glMatrixMode(GL_MODELVIEW);

	intrinsics.loadProjectionMatrix(0.001, 2000);
	applyMatrix(modelMatrix);
}

void	KinectProjectorOutput::unloadCalibratedView(){
	if (!isReady) return;
	
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

//----------------------------------------
ofVec2f KinectProjectorOutput::worldToScreen(ofVec3f WorldXYZ) const {

	if (!isReady) return ofVec2f(0,0);
    float nearDist = 500;
    float farDist = 2000;
    
    float w = intrinsics.getImageSize().width;
    float h = intrinsics.getImageSize().height;
    float fx = intrinsics.getCameraMatrix().at<double>(0, 0);
    float fy = intrinsics.getCameraMatrix().at<double>(1, 1);
    float cx = intrinsics.getPrincipalPoint().x;
    float cy = intrinsics.getPrincipalPoint().y;
    
    ofMatrix4x4 ProjectionMatrix;
    ProjectionMatrix.makeFrustumMatrix(
                              nearDist * (-cx) / fx, nearDist * (w - cx) / fx,
                              nearDist * (cy) / fy, nearDist * (cy - h) / fy,
                              nearDist, farDist);
//    ProjectionMatrix.makeFrustumMatrix(left,right,bottom,top,nearDist, farDist);
//    ProjectionMatrix.translate(-lensOffset.x, -lensOffset.y, 0);

    ofMatrix4x4 ModelViewMatrix;
    ModelViewMatrix.makeIdentityMatrix(); //=    modelMatrix;//.getInverse();

    ofMatrix4x4 ModelViewProjectionMatrix = ModelViewMatrix * ProjectionMatrix;

    ofVec3f CameraXYZ = WorldXYZ * ModelViewProjectionMatrix;
	ofVec3f ScreenXYZ;
    
	ScreenXYZ.x = (CameraXYZ.x + 1.0f) / 2.0f * w;//viewport.width + viewport.x;
	ScreenXYZ.y = (1.0f - CameraXYZ.y) / 2.0f * h;//viewport.height + viewport.y;
    
	ScreenXYZ.z = CameraXYZ.z;
    
	return ScreenXYZ;
    
}
float KinectProjectorOutput::getReprojectionError() const {
	if (!isReady) return 0.0;
	return reprojError;
}

void KinectProjectorOutput::setMirrors(bool horizontal, bool vertical) {
    mirrorHoriz = horizontal;
    mirrorVert = vertical;
}

bool KinectProjectorOutput::load(string path, bool absolute) {
    isReady = false;
    boardRotations.clear();
    boardTranslations.clear();
    FileStorage fs(ofToDataPath(path, absolute), FileStorage::READ);
    cv::Mat	rvec, tvec;
    fs["intrisics"] >> cameraMatrix;
    fs["projResX"] >> projectorResolutionX;
    fs["projResY"] >> projectorResolutionY;
    fs["rotation"] >> rvec;
    fs["translation"] >> tvec;
    fs["reprojectionError"] >>  reprojError;
    intrinsics.setup(cameraMatrix, Size2i(projectorResolutionX, projectorResolutionY));
    modelMatrix = makeMatrix(rvec, tvec);
    boardRotations.push_back(rvec);
    boardTranslations.push_back(tvec);
    distCoeffs = Mat::zeros(8, 1, CV_64F);
    
    // Setup Camera
    float fov=intrinsics.getFov().y;
    double aspectRatio = intrinsics.getAspectRatio();
    double* rm = rvec.ptr<double>(0);
    double* tm = tvec.ptr<double>(0);
    ofVec3f /*translation,*/ transcam;
    ofQuaternion rotation, rotcam;
    ofVec3f scale;
    ofQuaternion so;
    ofVec3f rvecof = ofVec3f(rm[0], -rm[1], -rm[2]);
    ofVec3f translation = ofVec3f(tm[0], tm[1],tm[2]);
    
    //modelMatrix.decompose(translation, rotation, scale,so);
    rotation = ofQuaternion(rvecof.length()/M_PI*180, rvecof);
    camera.setAutoDistance(false);
    camera.resetTransform();
    camera.setFov(fov);
    camera.setAspectRatio(aspectRatio);
    transcam = -rotation.inverse()*translation;
    rotcam = rotation.inverse();
    camera.setGlobalPosition(transcam);
    camera.setGlobalOrientation(rotcam);
   // camera.setTransformMatrix(modelMatrix);

    isReady = true;
	return true;
}