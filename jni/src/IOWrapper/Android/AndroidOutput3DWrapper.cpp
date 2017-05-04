#include "AndroidOutput3DWrapper.h"
#include "KeyFrameDisplay.h"
#include "logger.h"

namespace dso
{
namespace IOWrap
{

AndroidOutput3DWrapper::AndroidOutput3DWrapper(int w, int h, bool startRunThread)
    : w_(w), h_(h), running_(startRunThread)
{
    currentCam_ = new KeyFrameDisplay();
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
}

void AndroidOutput3DWrapper::publishCamPose(FrameShell* frame, CalibHessian* HCalib)
{
    boost::unique_lock<boost::mutex> lk(model3DMutex_);
    currentCam_->setFromF(frame, HCalib);
    
    std::ostringstream out;
    out << currentCam_->camToWorld.matrix();
    //LOGD("\ncamera pose:\n%s\n", out.str().c_str());
}

void AndroidOutput3DWrapper::pushLiveFrame(FrameHessian* image)
{
}

void AndroidOutput3DWrapper::pushDepthImage(MinimalImageB3* image)
{
}

bool AndroidOutput3DWrapper::needPushDepthImage()
{
}

void AndroidOutput3DWrapper::join()
{
}

void AndroidOutput3DWrapper::reset()
{
}

SE3 AndroidOutput3DWrapper::currentCamPose()
{
    boost::unique_lock<boost::mutex> lk(model3DMutex_);
    return currentCam_->camToWorld;
}

}
}
