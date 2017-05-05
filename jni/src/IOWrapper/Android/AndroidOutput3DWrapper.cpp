#include "AndroidOutput3DWrapper.h"
#include "KeyFrameDisplay.h"

#include "util/settings.h"
#include "util/globalCalib.h"
#include "util/logger.h"
#include "FullSystem/HessianBlocks.h"
#include "FullSystem/FullSystem.h"
#include "FullSystem/ImmaturePoint.h"


namespace dso
{
namespace IOWrap
{

AndroidOutput3DWrapper::AndroidOutput3DWrapper(int w, int h, bool startRunThread)
    : w_(w), h_(h), running_(startRunThread)
{
    currentCam_ = new KeyFrameDisplay();
    
	boost::unique_lock<boost::mutex> lk(openImagesMutex_);
	internalVideoImg_ = new MinimalImageB3(w,h);
	internalKFImg_ = new MinimalImageB3(w,h);

	internalVideoImg_->setBlack();
	internalKFImg_->setBlack();
}

AndroidOutput3DWrapper::~AndroidOutput3DWrapper()
{
}

void AndroidOutput3DWrapper::run()
{
}

void AndroidOutput3DWrapper::close()
{
}

void AndroidOutput3DWrapper::publishGraph(const std::map<uint64_t,Eigen::Vector2i> &connectivity)
{
}

void AndroidOutput3DWrapper::publishKeyframes( std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib)
{
    boost::unique_lock<boost::mutex> lk(model3DMutex_);
	for(FrameHessian* fh : frames)
	{
		if(keyframesByKFID_.find(fh->frameID) == keyframesByKFID_.end())
		{
			KeyFrameDisplay* kfd = new KeyFrameDisplay();
			keyframesByKFID_[fh->frameID] = kfd;
			keyframes_.push_back(kfd);
		}
		keyframesByKFID_[fh->frameID]->setFromKF(fh, HCalib);
	}
}

void AndroidOutput3DWrapper::publishCamPose(FrameShell* frame, CalibHessian* HCalib)
{
    boost::unique_lock<boost::mutex> lk(model3DMutex_);
    currentCam_->setFromF(frame, HCalib);

    float fx = HCalib->fxl();
	float fy = HCalib->fyl();
	float cx = HCalib->cxl();
	float cy = HCalib->cyl();
	LOGD("fx=%f, fy=%f, cx=%f, cy=%f\n", fx, fy, cx, cy);
    
    std::ostringstream out;
    out << currentCam_->camToWorld.matrix();
    LOGD("\ncamera pose:\n%s\n", out.str().c_str());
}

void AndroidOutput3DWrapper::pushLiveFrame(FrameHessian* image)
{
    boost::unique_lock<boost::mutex> lk(openImagesMutex_);
    
    for(int i = 0; i< w_ * h_; i++)
        internalVideoImg_->data[i][0] =
        internalVideoImg_->data[i][1] =
        internalVideoImg_->data[i][2] =
            image->dI[i][0]*0.8 > 255.0f ? 255.0 : image->dI[i][0]*0.8;

}

void AndroidOutput3DWrapper::pushDepthImage(MinimalImageB3* image)
{
    boost::unique_lock<boost::mutex> lk(openImagesMutex_);
	memcpy(internalKFImg_->data, image->data, w_ * h_ * 3);
}

bool AndroidOutput3DWrapper::needPushDepthImage()
{
    return setting_render_displayDepth;
}

void AndroidOutput3DWrapper::join()
{
}

void AndroidOutput3DWrapper::reset()
{
    model3DMutex_.lock();
	for(size_t i=0; i<keyframes_.size();i++) delete keyframes_[i];
	keyframes_.clear();
	keyframesByKFID_.clear();
    model3DMutex_.unlock();
    
    openImagesMutex_.lock();
	internalVideoImg_->setBlack();
	internalKFImg_->setBlack();
	openImagesMutex_.unlock();
}

SE3 AndroidOutput3DWrapper::currentCamPose()
{
    boost::unique_lock<boost::mutex> lk(model3DMutex_);
    return currentCam_->camToWorld;
}

int AndroidOutput3DWrapper::getKeyframeCount()
{
    boost::unique_lock<boost::mutex> lk(model3DMutex_);
    return keyframes_.size();
}

MinimalImageB3* AndroidOutput3DWrapper::cloneVideoImage()
{
    boost::unique_lock<boost::mutex> lk(openImagesMutex_);
    return internalVideoImg_->getClone();
}

MinimalImageB3* AndroidOutput3DWrapper::cloneKeyframeImage()
{
    boost::unique_lock<boost::mutex> lk(openImagesMutex_);
    return internalKFImg_->getClone();
}

}
}
