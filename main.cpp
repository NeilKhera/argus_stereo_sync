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

#include <iostream>
#include <chrono>
#include <thread>
#include <ctime>
#include <fstream>

using namespace Argus;
using namespace EGLStream;

std::ofstream outfile;

namespace ArgusSamples {

EGLDisplayHolder g_display;

static const Size2D<uint32_t> STREAM_SIZE(1280, 720);
static const uint32_t FRAMERATE = 5;

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

    CONSUMER_PRINT("Waiting until producer is connected...\n");
    if (leftIStream->waitUntilConnected() != STATUS_OK || rightIStream->waitUntilConnected() != STATUS_OK) {
        ORIGINATE_ERROR("Stream failed to connect.");
    }
    CONSUMER_PRINT("Streams connected, processing frames.\n");

    int frameCount = 0;
    while (true) {
	EGLint streamState = EGL_STREAM_STATE_CONNECTING_KHR;
	if (!eglQueryStreamKHR(
		leftIStream->getEGLDisplay(), 
		leftIStream->getEGLStream(),
		EGL_STREAM_STATE_KHR,
		&streamState) || (streamState == EGL_STREAM_STATE_DISCONNECTED_KHR)) {
	    CONSUMER_PRINT("EGL_STREAM_STATE_DISCONNECTED_KHR received\n");
	    break;
	}

        if (!eglQueryStreamKHR(
                rightIStream->getEGLDisplay(),
                rightIStream->getEGLStream(),
                EGL_STREAM_STATE_KHR,
                &streamState) || (streamState == EGL_STREAM_STATE_DISCONNECTED_KHR)) {
            CONSUMER_PRINT("EGL_STREAM_STATE_DISCONNECTED_KHR received\n");
            break;
        }
	//std::cout << "Boop" << std::endl;

	UniqueObj<Frame> left_frame(leftIFrameConsumer->acquireFrame());
	auto frame1_timestamp = std::chrono::system_clock::now();
        UniqueObj<Frame> right_frame(rightIFrameConsumer->acquireFrame());
	auto frame2_timestamp = std::chrono::system_clock::now();

	std::chrono::duration<double> diff = frame1_timestamp - frame2_timestamp;
	outfile << -diff.count() << "\n";

        if (!left_frame || !right_frame) {
	    break;
	}

	IFrame *left_iframe = interface_cast<IFrame>(left_frame);
	IFrame *right_iframe = interface_cast<IFrame>(right_frame);
	if (!left_iframe || !right_iframe) {
	    break;
	}

        Image *left_image = left_iframe->getImage();
        Image *right_image = right_iframe->getImage();
        
	IImageJPEG *iJPEG_left = interface_cast<IImageJPEG>(left_image);
	IImageJPEG *iJPEG_right = interface_cast<IImageJPEG>(right_image);
        if (!iJPEG_left || !iJPEG_right) {
            ORIGINATE_ERROR("Failed to get IImageJPEG interface.");
	}

        std::ostringstream fileName1;
	std::ostringstream fileName2;
        fileName1 << "left_" << std::setfill('0') << std::setw(4) << frameCount << ".jpg";
	fileName2 << "right_" << std::setfill('0') << std::setw(4) << frameCount << ".jpg";
        if (iJPEG_left->writeJPEG(fileName1.str().c_str()) == STATUS_OK) {
            //CONSUMER_PRINT("Captured a still image to '%s'\n", fileName1.str().c_str());
    	} else {
            ORIGINATE_ERROR("Failed to write JPEG to '%s'\n", fileName1.str().c_str());
	}

        if (iJPEG_right->writeJPEG(fileName2.str().c_str()) == STATUS_OK) {
            //CONSUMER_PRINT("Captured a still image to '%s'\n", fileName2.str().c_str());
	} else {
            ORIGINATE_ERROR("Failed to write JPEG to '%s'\n", fileName2.str().c_str());
	}
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
    if (!iStreamSettings || !iEGLStreamSettings) {
        ORIGINATE_ERROR("Failed to create OutputStreamSettings");
    }
    iEGLStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
    iEGLStreamSettings->setResolution(STREAM_SIZE);
    iEGLStreamSettings->setEGLDisplay(g_display.get());
    iEGLStreamSettings->setMetadataEnable(true);

    PRODUCER_PRINT("Creating left stream.\n");
    iStreamSettings->setCameraDevice(lrCameras[0]);
    UniqueObj<OutputStream> streamLeft(iCaptureSession->createOutputStream(streamSettings.get()));
    IEGLOutputStream *iStreamLeft = interface_cast<IEGLOutputStream>(streamLeft);
    if (!iStreamLeft) {
        ORIGINATE_ERROR("Failed to create left stream");
    }

    PRODUCER_PRINT("Creating right stream.\n");
    iStreamSettings->setCameraDevice(lrCameras[1]);
    UniqueObj<OutputStream> streamRight(iCaptureSession->createOutputStream(streamSettings.get()));
    IEGLOutputStream *iStreamRight = interface_cast<IEGLOutputStream>(streamRight);
    if (!iStreamRight) {
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

    ISourceSettings *iSourceSettings = interface_cast<ISourceSettings>(iRequest->getSourceSettings());
    iSourceSettings->setFrameDurationRange(Range<uint64_t>(1e9/FRAMERATE));

    iRequest->enableOutputStream(streamLeft.get());
    iRequest->enableOutputStream(streamRight.get());

    PRODUCER_PRINT("Starting repeat capture requests.\n");
    if (iCaptureSession->repeat(request.get()) != STATUS_OK) {
	ORIGINATE_ERROR("Failed to start repeat capture request for preview");
    }
    sleep(3600);

    iCaptureSession->stopRepeat();
    iCaptureSession->waitForIdle();

    PRODUCER_PRINT("Captures complete, disconnecting producer.\n");
    iStreamLeft->disconnect();
    iStreamRight->disconnect();

    PROPAGATE_ERROR(stereoConsumer.shutdown());
    cameraProvider.reset();
    PROPAGATE_ERROR(g_display.cleanup());

    PRODUCER_PRINT("Done -- exiting.\n");
    return true;
}

};

int main (int argc, char *argv[]) {
    outfile.open("test3.txt");
    if (!ArgusSamples::execute()) {
        return EXIT_FAILURE;
    }
    outfile.close();
    return EXIT_SUCCESS;
}
