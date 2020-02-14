#include <stdio.h>
#include <stdlib.h>
#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <opencv2/opencv.hpp>

#include "Thread.h"
#include "Error.h"
#include "EGLGlobal.h"
#include "ArgusHelpers.h"
#include "CommonOptions.h"

using namespace Argus;
using namespace EGLStream;

namespace ArgusSamples {

EGLDisplayHolder g_display;

static const Size2D<uint32_t> STREAM_SIZE(960, 540);
static const uint32_t CAPTURE_TIME = 60;

#define PRODUCER_PRINT(...) printf("PRODUCER: " __VA_ARGS__)
#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)

class StereoConsumerThread : public Thread {

public:
    explicit StereoConsumerThread(OutputStream *leftStream, OutputStream *rightStream)
	    : m_leftStream(leftStream), m_rightStream(rightStream) {}
    ~StereoConsumerThread() {}

private:
    virtual bool threadInitialize();
    virtual bool threadExecute();
    virtual bool threadShutdown();

    OutputStream *m_leftStream;
    OutputStream *m_rightStream;
    UniqueObj<FrameConsumer> m_leftConsumer;
    UniqueObj<FrameConsumer> m_rightConsumer;
};

bool StereoConsumerThread::threadInitialize() {
    m_leftConsumer = UniqueObj<FrameConsumer>(FrameConsumer::create(m_leftStream));
    m_rightConsumer = UniqueObj<FrameConsumer>(FrameConsumer::create(m_rightStream));
    if (!m_leftConsumer || !m_rightConsumer) {
        ORIGINATE_ERROR("Failed to create FrameConsumers");
    }
    return true;
}

bool StereoConsumerThread::threadExecute() {
    IEGLOutputStream *leftIStream = interface_cast<IEGLOutputStream>(m_leftStream);
    IEGLOutputStream *rightIStream = interface_cast<IEGLOutputStream>(m_rightStream);
    IFrameConsumer *leftIFrameConsumer = interface_cast<IFrameConsumer>(m_leftConsumer);
    IFrameConsumer *rightIFrameConsumer = interface_cast<IFrameConsumer>(m_rightConsumer);
    //Argus::Status status_left;
    //Argus::Status status_right;

    CONSUMER_PRINT("Waiting until producer is connected...\n");
    if (leftIStream->waitUntilConnected() != STATUS_OK || rightIStream->waitUntilConnected() != STATUS_OK) {
        ORIGINATE_ERROR("Stream failed to connect.");
    }
    CONSUMER_PRINT("Producer has connected; continuing.\n");

    int frameCount = 0;
    while (true) {
	UniqueObj<Frame> left_frame(leftIFrameConsumer->acquireFrame());
        UniqueObj<Frame> right_frame(rightIFrameConsumer->acquireFrame());
        IFrame *left_iframe = interface_cast<IFrame>(left_frame);
        IFrame *right_iframe = interface_cast<IFrame>(right_frame);

        Image *left_image = left_iframe->getImage();
        Image *right_image = right_iframe->getImage();
	
	/*IImage *left_iimage = interface_cast<IImage>(left_image);
	IImage *right_iimage = interface_cast<IImage>(right_image);
	IImage2D *left_iimage2d = interface_cast<IImage2D>(left_image);
	IImage2D *right_iimage2d = interface_cast<IImage2D>(right_image);
        
	cv::Mat imgbuf_left = cv::Mat(cv::Size(STREAM_SIZE.width(), STREAM_SIZE.height()), CV_8UC1, (void*)left_iimage->mapBuffer(), (size_t)left_iimage2d->getStride());
	cv::Mat imgbuf_right = cv::Mat(cv::Size(STREAM_SIZE.width(), STREAM_SIZE.height()), CV_8UC1, (void*)right_iimage->mapBuffer(), (size_t)right_iimage2d->getStride());
	cv::imwrite("/home/lunarleopard2/Desktop/left.jpg", imgbuf_left);
	cv::imwrite("/home/lunarleopard2/Desktop/right.jpg", imgbuf_right);
	//cv::imshow("img_left", imgbuf_left);
        //cv::imshow("img_right", imgbuf_right);
        cv::waitKey(1);*/

        IImageJPEG *iJPEG_left = interface_cast<IImageJPEG>(left_image);
	IImageJPEG *iJPEG_right = interface_cast<IImageJPEG>(right_image);
        if (!iJPEG_left || !iJPEG_right)
            ORIGINATE_ERROR("Failed to get IImageJPEG interface.");

        std::ostringstream fileName1;
	std::ostringstream fileName2;
        fileName1 << "left_" << std::setfill('0') << std::setw(4) << frameCount << ".jpg";
	fileName2 << "right_" << std::setfill('0') << std::setw(4) << frameCount++ << ".jpg";
        if (iJPEG_left->writeJPEG(fileName1.str().c_str()) == STATUS_OK)
            CONSUMER_PRINT("Captured a still image to '%s'\n", fileName1.str().c_str());
        else
            ORIGINATE_ERROR("Failed to write JPEG to '%s'\n", fileName1.str().c_str());

        if (iJPEG_right->writeJPEG(fileName2.str().c_str()) == STATUS_OK)
            CONSUMER_PRINT("Captured a still image to '%s'\n", fileName2.str().c_str());
        else
            ORIGINATE_ERROR("Failed to write JPEG to '%s'\n", fileName2.str().c_str());

    }
    
    CONSUMER_PRINT("No more frames. Cleaning up.\n");
    PROPAGATE_ERROR(requestShutdown());
    return true;
}

bool StereoConsumerThread::threadShutdown() {
    CONSUMER_PRINT("Done.\n");
    return true;
}

static bool execute() {
    PROPAGATE_ERROR(g_display.initialize());

    UniqueObj<CameraProvider> cameraProvider(CameraProvider::create());
    ICameraProvider *iCameraProvider = interface_cast<ICameraProvider>(cameraProvider);
    if (!iCameraProvider) {
        ORIGINATE_ERROR("Failed to get ICameraProvider interface");
    }
    printf("Argus Version: %s\n", iCameraProvider->getVersion().c_str());

    std::vector<CameraDevice*> cameraDevices;
    iCameraProvider->getCameraDevices(&cameraDevices);
    if (cameraDevices.size() < 2) {
        ORIGINATE_ERROR("Insufficient number of sensors available");
    }
    std::vector<CameraDevice*> lrCameras;
    lrCameras.push_back(cameraDevices[0]);
    lrCameras.push_back(cameraDevices[1]);

    UniqueObj<CaptureSession> captureSession(iCameraProvider->createCaptureSession(lrCameras));
    ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(captureSession);
    if (!iCaptureSession) {
        ORIGINATE_ERROR("Failed to get capture session interface");
    }

    UniqueObj<OutputStreamSettings> streamSettings(iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL));
    IOutputStreamSettings *iStreamSettings = interface_cast<IOutputStreamSettings>(streamSettings);
    IEGLOutputStreamSettings *iEGLStreamSettings = interface_cast<IEGLOutputStreamSettings>(streamSettings);
    if (!iEGLStreamSettings) {
        ORIGINATE_ERROR("Failed to create OutputStreamSettings");
    }
    iEGLStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
    iEGLStreamSettings->setResolution(STREAM_SIZE);
    iEGLStreamSettings->setEGLDisplay(g_display.get());
    iEGLStreamSettings->setMetadataEnable(true);

    PRODUCER_PRINT("Creating left stream.\n");
    iStreamSettings->setCameraDevice(lrCameras[0]);
    UniqueObj<OutputStream> streamLeft(iCaptureSession->createOutputStream(streamSettings.get()));
    if (!streamLeft.get()) {
        ORIGINATE_ERROR("Failed to create left stream");
    }

    PRODUCER_PRINT("Creating right stream.\n");
    iStreamSettings->setCameraDevice(lrCameras[1]);
    UniqueObj<OutputStream> streamRight(iCaptureSession->createOutputStream(streamSettings.get()));
    if (!streamRight.get()) {
        ORIGINATE_ERROR("Failed to create right stream");
    }

    PRODUCER_PRINT("Launching stereo consumer.\n");
    StereoConsumerThread stereoConsumer(streamLeft.get(), streamRight.get());
    PROPAGATE_ERROR(stereoConsumer.initialize());
    PROPAGATE_ERROR(stereoConsumer.waitRunning());

    UniqueObj<Request> request(iCaptureSession->createRequest());
    IRequest *iRequest = interface_cast<IRequest>(request);
    if (!iRequest) {
        ORIGINATE_ERROR("Failed to create Request");
    }
    iRequest->enableOutputStream(streamLeft.get());
    iRequest->enableOutputStream(streamRight.get());

    PRODUCER_PRINT("Starting repeat capture requests.\n");
    for (uint32_t i = 0; i < CAPTURE_TIME; i++) {
    	if (iCaptureSession->capture(request.get()) == 0) {
            ORIGINATE_ERROR("Failed to start repeat capture request for preview");
    	}
	sleep(1);
    }
    iCaptureSession->stopRepeat();
    iCaptureSession->waitForIdle();

    PRODUCER_PRINT("Captures complete, disconnecting producer.\n");
    streamLeft.reset();
    streamRight.reset();
    PROPAGATE_ERROR(stereoConsumer.shutdown());
    cameraProvider.reset();

    PRODUCER_PRINT("Done -- exiting.\n");
    return 0;
}

};

int main (int argc, char *argv[]) {
    if (!ArgusSamples::execute()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
