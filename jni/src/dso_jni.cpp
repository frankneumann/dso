#include <string.h>
#include <jni.h>
#include <boost/thread.hpp>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/ImageDisplay.h"
#include "IOWrapper/Android/AndroidOutput3DWrapper.h"

#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/DatasetReader.h"
#include "util/globalCalib.h"
#include "util/logger.h"

#include "util/NumType.h"
#include "FullSystem/FullSystem.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "FullSystem/PixelSelector2.h"
#include "util/MinimalImage.h"



// FIXME: remove hard code
#define IMAGE_DIR "/sdcard/LSD/images"
std::string source = IMAGE_DIR;
std::string calib = "";
std::string gammaCalib = "";
std::string vignette = "";


double rescale = 1;
bool reverse = false;
bool disableROS = false;
int start=0;
int end=100000;
bool prefetch = false;
float playbackSpeed=0;	// 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.
bool preload=false;
bool useSampleOutput=false;

bool firstRosSpin=false;

using namespace dso;

class DSOSlamSystem {
public:
    DSOSlamSystem() {
        reader_ = new ImageFolderReader(source, calib, gammaCalib, vignette);
    	reader_->setGlobalCalibration();

    	undistort_ = Undistort::getUndistorterForFile(calib, gammaCalib, vignette);

        fullSystem_ = new FullSystem();
    	fullSystem_->setGammaFunction(reader_->getPhotometricGamma());
    	fullSystem_->linearizeOperation = (playbackSpeed==0);

    	outputWrapper_ = new IOWrap::AndroidOutput3DWrapper(wG[0], hG[0], false);
    	fullSystem_->outputWrapper.push_back(outputWrapper_);

    	frameId_ = 0;
    }

    ~DSOSlamSystem() {
        if (reader_) {
            delete reader_;
            reader_ = NULL;
        }
        if (fullSystem_) {
            delete fullSystem_;
            fullSystem_ = NULL;
        }
        if (outputWrapper_) {
            delete outputWrapper_;
            outputWrapper_ = NULL;
        }
        if (undistort_) {
            delete undistort_;
            undistort_ = NULL;
        }
    }

    int onFrame(int width, int height, unsigned char* data) {
        // Ref. https://github.com/JakobEngel/dso_ros/blob/master/src/main.cpp vidCb function
        if(setting_fullResetRequested) {
    		std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem_->outputWrapper;
    		delete fullSystem_;
    		for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
    		fullSystem_ = new FullSystem();
    		fullSystem_->linearizeOperation=false;
    		fullSystem_->outputWrapper = wraps;
    	    if(undistort_->photometricUndist != 0)
    	    	fullSystem_->setGammaFunction(undistort_->photometricUndist->getG());
    		setting_fullResetRequested=false;
    	}
    
        MinimalImageB minImg(width, height, data);
        ImageAndExposure* undistImg = undistort_->undistort<unsigned char>(
				&minImg, 1.0f, 0.0);
	    fullSystem_->addActiveFrame(undistImg, frameId_);
        frameId_++;
        delete undistImg;
    }
    
private:
    ImageFolderReader* reader_;
    Undistort* undistort_;;
    FullSystem* fullSystem_;
    IOWrap::AndroidOutput3DWrapper* outputWrapper_;
    int frameId_;
};

static DSOSlamSystem* gSlamSystem = NULL;

extern "C"{
JavaVM* gJvm = NULL;

JNIEXPORT void JNICALL
Java_com_tc_tar_TARNativeInterface_dsoInit(JNIEnv* env, jobject thiz, jstring calibPath) {
	LOGD("dsoInit\n");
    //init jni
	env->GetJavaVM(&gJvm);

    const char *calibFile = env->GetStringUTFChars(calibPath, 0);
    LOGD("calibFile: %s\n", calibFile);
    calib = calibFile;
	env->ReleaseStringUTFChars(calibPath, calibFile);

    gSlamSystem = new DSOSlamSystem();
}

JNIEXPORT void JNICALL
Java_com_tc_tar_TARNativeInterface_dsoRelease(JNIEnv* env, jobject thiz) {
	LOGD("dsoRelease\n");
}

JNIEXPORT int JNICALL
Java_com_tc_tar_TARNativeInterface_dsoOnFrame(JNIEnv* env, jobject thiz, jint width, jint height, jbyteArray array, jint format) {
	LOGD("dsoOnFrame\n");
	unsigned char* yuv = (unsigned char*)env->GetByteArrayElements(array, 0);
	gSlamSystem->onFrame(width, height, yuv);
	env->ReleaseByteArrayElements(array, (jbyte*)yuv, 0);
	
	return 0;
}

JNIEXPORT jfloatArray JNICALL
Java_com_tc_tar_TARNativeInterface_dsoGetIntrinsics(JNIEnv* env, jobject thiz) {    
    jfloatArray result;
    result = env->NewFloatArray(4);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }
    
    jfloat array1[4];
    // TODO: set value
    
    env->SetFloatArrayRegion(result, 0, 4, array1);
    return result;
}

JNIEXPORT jintArray JNICALL
Java_com_tc_tar_TARNativeInterface_dsoGetResolution(JNIEnv* env, jobject thiz) {
    jintArray result;
    result = env->NewIntArray(2);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }
    
    jint array1[2];
    // TODO: set value

    env->SetIntArrayRegion(result, 0, 2, array1);
    return result;
}

JNIEXPORT jfloatArray JNICALL
Java_com_tc_tar_TARNativeInterface_dsoGetCurrentPose(JNIEnv* env, jobject thiz) {
    jfloatArray result;
    int length = 16;
    result = env->NewFloatArray(length);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }

    jfloat mat4[length];
    // TODO: set value


    env->SetFloatArrayRegion(result, 0, length, mat4);
    return result;
}

JNIEXPORT jobjectArray JNICALL
Java_com_tc_tar_TARNativeInterface_dsoGetAllKeyFrames(JNIEnv* env, jobject thiz) {
    jclass classKeyFrame = env->FindClass("com/tc/tar/DSOKeyFrame");
    std::list<jobject> objectList;
    // TODO: set value

    if (objectList.empty())
        return NULL;

    // Add to result
    jobjectArray result = env->NewObjectArray(objectList.size(), classKeyFrame, NULL);
    int i = 0;
    for (std::list<jobject>::iterator it = objectList.begin(); it != objectList.end(); ++it) {
        env->SetObjectArrayElement(result, i++, *it);
    }

    // Release
    // TODO: release ref
    
    return result;
}

JNIEXPORT jint JNICALL
Java_com_tc_tar_TARNativeInterface_dsoGetKeyFrameCount(JNIEnv* env, jobject thiz) {
    // TODO: implementation
    return 0;
}

JNIEXPORT jbyteArray JNICALL
Java_com_tc_tar_TARNativeInterface_dsoGetCurrentImage(JNIEnv* env, jobject thiz, jint format) {
    // TODO: implemetation
    return NULL;
}
}
