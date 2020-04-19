#include "tic_toc.h"
#include <csignal>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>

#include "Argus/Argus.h"
#include "EGLStream/EGLStream.h"

#include "ArgusHelpers.h"
#include "CommonOptions.h"
#include "EGLGlobal.h"
#include "Error.h"
#include "Thread.h"

//#include <cuda_runtime.h>
//#include "yuv2rgb.cuh"

using namespace Argus;
using namespace EGLStream;

ros::Publisher left_img_pub;
ros::Publisher right_img_pub;

namespace ArgusSamples {

EGLDisplayHolder g_display;
ICaptureSession *iCaptureSession;
IEGLOutputStream *iStreamLeft;
IEGLOutputStream *iStreamRight;
UniqueObj<CameraProvider> cameraProvider;

static const Size2D<uint32_t> STREAM_SIZE(3840, 2160);
static const uint32_t FRAMERATE = 60;

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

    IFrame *left_iframe;
    IFrame *right_iframe;
    Image *left_image;
    Image *right_image;

    unsigned char* RGB = new unsigned char[3 * STREAM_SIZE.width() * STREAM_SIZE.height() * sizeof(unsigned char)];
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
  if (leftIStream->waitUntilConnected() != STATUS_OK ||
      rightIStream->waitUntilConnected() != STATUS_OK) {
    ORIGINATE_ERROR("Stream failed to connect.");
  }
  CONSUMER_PRINT("Streams connected, processing frames.\n");

  while (true) {
    tic();
    EGLint streamState = EGL_STREAM_STATE_CONNECTING_KHR;
    if (!eglQueryStreamKHR(leftIStream->getEGLDisplay(), leftIStream->getEGLStream(), 
	EGL_STREAM_STATE_KHR, &streamState) ||
        (streamState == EGL_STREAM_STATE_DISCONNECTED_KHR)) {
      CONSUMER_PRINT("EGL_STREAM_STATE_DISCONNECTED_KHR received\n");
      break;
    }

    if (!eglQueryStreamKHR(rightIStream->getEGLDisplay(), rightIStream->getEGLStream(),
	EGL_STREAM_STATE_KHR, &streamState) ||
        (streamState == EGL_STREAM_STATE_DISCONNECTED_KHR)) {
      CONSUMER_PRINT("EGL_STREAM_STATE_DISCONNECTED_KHR received\n");
      break;
    }
    ROS_ERROR("1: %f", toc());

    tic();
    UniqueObj<Frame> left_frame(leftIFrameConsumer->acquireFrame());
    UniqueObj<Frame> right_frame(rightIFrameConsumer->acquireFrame());

    left_iframe = interface_cast<IFrame>(left_frame);
    right_iframe = interface_cast<IFrame>(right_frame);

    left_image = left_iframe->getImage();
    right_image = right_iframe->getImage();

    IImage *iImage = interface_cast<IImage>(left_image);
    IImage2D *iImage2D = interface_cast<IImage2D>(left_image);
    ROS_ERROR("2: %f", toc());

    tic();
    Size2D<uint32_t> size = iImage2D->getSize(0);
    uint32_t width = size.width();
    uint32_t height = size.height();
    
    //gpuConvertYUV420toRGB((uint8_t*) iImage->mapBuffer((uint32_t) 0), (uint8_t*) iImage->mapBuffer((uint32_t) 1), RGB, width, height);
    //ROS_ERROR("3: %f", toc());

    tic();
    sensor_msgs::Image left_img_msg;
    sensor_msgs::fillImage(left_img_msg, sensor_msgs::image_encodings::RGB8, height, width, 3 * width, RGB);
    left_img_pub.publish(left_img_msg);
    ROS_ERROR("4: %f", toc());
  }

  CONSUMER_PRINT("No more frames. Cleaning up.\n");
  PROPAGATE_ERROR(requestShutdown());
  return true;
}

bool StereoConsumerThread::threadShutdown() {
  delete[] RGB;
  CONSUMER_PRINT("Done.\n");
  return true;
}

std::unique_ptr<StereoConsumerThread> stereoConsumer;

static void argusSigintHandler(int sig) {
  iCaptureSession->stopRepeat();
  iCaptureSession->waitForIdle();

  PRODUCER_PRINT("Captures complete, disconnecting producer.\n");
  iStreamLeft->disconnect();
  iStreamRight->disconnect();

  if (!stereoConsumer->shutdown()) {
    PRODUCER_PRINT("Error. Consumer shutdown failed");
  }
  cameraProvider.reset();
  if (!g_display.cleanup()) {
    PRODUCER_PRINT("Error. Display cleanup failed");
  }

  PRODUCER_PRINT("Done -- exiting.\n");
  ros::shutdown();
}

static bool execute() {
  PROPAGATE_ERROR(g_display.initialize());

  cameraProvider = UniqueObj<CameraProvider>(CameraProvider::create());
  ICameraProvider *iCameraProvider = interface_cast<ICameraProvider>(cameraProvider);
  if (!iCameraProvider) {
    ORIGINATE_ERROR("Failed to get ICameraProvider interface");
  }
  printf("Argus Version: %s\n", iCameraProvider->getVersion().c_str());

  std::vector<CameraDevice *> cameraDevices;
  iCameraProvider->getCameraDevices(&cameraDevices);
  if (cameraDevices.size() < 2) {
    ORIGINATE_ERROR("Insufficient number of sensors available");
  }
  std::vector<CameraDevice *> lrCameras;
  lrCameras.push_back(cameraDevices[0]);
  lrCameras.push_back(cameraDevices[1]);

  UniqueObj<CaptureSession> captureSession(iCameraProvider->createCaptureSession(lrCameras));
  iCaptureSession = interface_cast<ICaptureSession>(captureSession);
  if (!iCaptureSession) {
    ORIGINATE_ERROR("Failed to get capture session interface");
  }

  UniqueObj<OutputStreamSettings> streamSettings(
      iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL));
  IOutputStreamSettings *iStreamSettings =
      interface_cast<IOutputStreamSettings>(streamSettings);
  IEGLOutputStreamSettings *iEGLStreamSettings =
      interface_cast<IEGLOutputStreamSettings>(streamSettings);
  if (!iStreamSettings || !iEGLStreamSettings) {
    ORIGINATE_ERROR("Failed to create OutputStreamSettings");
  }
  iEGLStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
  iEGLStreamSettings->setResolution(STREAM_SIZE);
  iEGLStreamSettings->setEGLDisplay(g_display.get());
  iEGLStreamSettings->setMetadataEnable(true);

  PRODUCER_PRINT("Creating left stream.\n");
  iStreamSettings->setCameraDevice(lrCameras[0]);
  UniqueObj<OutputStream> streamLeft(
      iCaptureSession->createOutputStream(streamSettings.get()));
  iStreamLeft = interface_cast<IEGLOutputStream>(streamLeft);
  if (!iStreamLeft) {
    ORIGINATE_ERROR("Failed to create left stream");
  }

  PRODUCER_PRINT("Creating right stream.\n");
  iStreamSettings->setCameraDevice(lrCameras[1]);
  UniqueObj<OutputStream> streamRight(
      iCaptureSession->createOutputStream(streamSettings.get()));
  iStreamRight = interface_cast<IEGLOutputStream>(streamRight);
  if (!iStreamRight) {
    ORIGINATE_ERROR("Failed to create right stream");
  }

  PRODUCER_PRINT("Launching stereo consumer.\n");
  stereoConsumer = std::make_unique<StereoConsumerThread>(streamLeft.get(), streamRight.get());
  PROPAGATE_ERROR(stereoConsumer->initialize());
  PROPAGATE_ERROR(stereoConsumer->waitRunning());

  UniqueObj<Request> request(iCaptureSession->createRequest());
  IRequest *iRequest = interface_cast<IRequest>(request);
  if (!iRequest) {
    ORIGINATE_ERROR("Failed to create Request");
  }

  ISourceSettings *iSourceSettings =
      interface_cast<ISourceSettings>(iRequest->getSourceSettings());
  iSourceSettings->setFrameDurationRange(Range<uint64_t>(1e9 / FRAMERATE));

  iRequest->enableOutputStream(streamLeft.get());
  iRequest->enableOutputStream(streamRight.get());

  PRODUCER_PRINT("Starting repeat capture requests.\n");
  if (iCaptureSession->repeat(request.get()) != STATUS_OK) {
    ORIGINATE_ERROR("Failed to start repeat capture request for preview");
  }
  
  //ros::spin();
  // Temporary
  sleep(60);
  argusSigintHandler(0);
  return true;
}

};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "argus_stereo_sync_node");
  ros::NodeHandle nh;

  signal(SIGINT, ArgusSamples::argusSigintHandler);

  left_img_pub = nh.advertise<sensor_msgs::Image>("/camera/left/image_raw", 1);
  right_img_pub = nh.advertise<sensor_msgs::Image>("/camera/right/image_raw", 1);
  if (!ArgusSamples::execute()) {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
