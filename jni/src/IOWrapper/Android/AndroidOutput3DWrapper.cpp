#include "AndroidOutput3DWrapper.h"

namespace dso
{
namespace IOWrap
{

AndroidOutput3DWrapper::AndroidOutput3DWrapper(int w, int h, bool startRunThread)
    : w_(w), h_(h), running_(startRunThread)
{

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

}
}
