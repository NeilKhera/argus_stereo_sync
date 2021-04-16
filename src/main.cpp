#include <csignal>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <cuda.h>
#include <cudaEGL.h>

#include "ArgusHelpers.h"
#include "CUDAHelper.h"
#include "EGLGlobal.h"
#include "Error.h"
#include "Thread.h"

#include "convert.h"

using namespace Argus;

static const uint32_t FRAMERATE = 30;
static const Size2D<uint32_t> STREAM_SIZE(960, 540);
static const Range<float> GAIN_RANGE(1, 44);
static const Range<float> ISP_DIGITAL_GAIN_RANGE(1, 1);
static const Range<uint64_t> EXPOSURE_TIME_RANGE(44000, 1000000);

ros::Publisher left_image_pub;
ros::Publisher left_camera_info_pub;
ros::Publisher right_image_pub;
ros::Publisher right_camera_info_pub;
uint8_t* oBuffer = new uint8_t[3 * STREAM_SIZE.width() * STREAM_SIZE.height()];

namespace ArgusSamples {

EGLDisplayHolder g_display;

#define PRODUCER_PRINT(...) printf("PRODUCER: " __VA_ARGS__)
#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)

class StereoConsumer : public Thread {
  public:
    explicit StereoConsumer(IEGLOutputStream *leftStream, IEGLOutputStream *rightStream)
		            : m_leftStream(leftStream), m_rightStream(rightStream)
                            , m_cuStreamLeft(NULL), m_cuStreamRight(NULL) 
	                    , m_cudaContext(0) {}
    ~StereoConsumer() {}

  private:
    virtual bool threadInitialize();
    virtual bool threadExecute();
    virtual bool threadShutdown();

    IEGLOutputStream *m_leftStream;
    IEGLOutputStream *m_rightStream;
    CUeglStreamConnection m_cuStreamLeft;
    CUeglStreamConnection m_cuStreamRight;
    CUcontext m_cudaContext;
};

class CudaFrameAcquire {
  public:
    CudaFrameAcquire(CUeglStreamConnection& connection);
    ~CudaFrameAcquire();

    bool publish(bool leftFrame);
  
  private:
    CUeglStreamConnection& m_connection;
    CUgraphicsResource m_resource;
    CUeglFrame m_frame;
    CUstream m_stream;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

bool StereoConsumer::threadInitialize() {
  PROPAGATE_ERROR(initCUDA(&m_cudaContext));

  CONSUMER_PRINT("Connecting CUDA consumer to left stream\n");
  CUresult cuResult = cuEGLStreamConsumerConnect(&m_cuStreamLeft, m_leftStream->getEGLStream());
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to connect CUDA to EGLStream (%s)", getCudaErrorString(cuResult));
  }

  CONSUMER_PRINT("Connecting CUDA consumer to right stream\n");
  cuResult = cuEGLStreamConsumerConnect(&m_cuStreamRight, m_rightStream->getEGLStream());
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to connect CUDA to EGLStream (%s)", getCudaErrorString(cuResult));
  }
  return true;
}

bool StereoConsumer::threadExecute() {
  CONSUMER_PRINT("Waiting for Argus producer to connect to left stream.\n");
  m_leftStream->waitUntilConnected();

  CONSUMER_PRINT("Waiting for Argus producer to connect to right stream.\n");
  m_rightStream->waitUntilConnected();

  CONSUMER_PRINT("Streams connected, processing frames.\n");
  while (true) {
    EGLint streamState = EGL_STREAM_STATE_CONNECTING_KHR;

    if (!eglQueryStreamKHR(m_leftStream->getEGLDisplay(), m_leftStream->getEGLStream(),
        EGL_STREAM_STATE_KHR, &streamState) || (streamState == EGL_STREAM_STATE_DISCONNECTED_KHR)) {
      CONSUMER_PRINT("left : EGL_STREAM_STATE_DISCONNECTED_KHR received\n");
      break;
    }

    if (!eglQueryStreamKHR(m_rightStream->getEGLDisplay(), m_rightStream->getEGLStream(),
        EGL_STREAM_STATE_KHR, &streamState) || (streamState == EGL_STREAM_STATE_DISCONNECTED_KHR)) {
      CONSUMER_PRINT("right : EGL_STREAM_STATE_DISCONNECTED_KHR received\n");
      break;
    }

    CudaFrameAcquire left(m_cuStreamLeft);
    CudaFrameAcquire right(m_cuStreamRight);

    PROPAGATE_ERROR(left.publish(true) && right.publish(false));
  }
    
  CONSUMER_PRINT("No more frames. Cleaning up.\n");
  PROPAGATE_ERROR(requestShutdown());
  return true;
}

bool StereoConsumer::threadShutdown() {
  CUresult cuResult = cuEGLStreamConsumerDisconnect(&m_cuStreamLeft);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to disconnect CUDA stream (%s)", getCudaErrorString(cuResult));
  }

  cuResult = cuEGLStreamConsumerDisconnect(&m_cuStreamRight);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to disconnect CUDA stream (%s)", getCudaErrorString(cuResult));
  }

  PROPAGATE_ERROR(cleanupCUDA(&m_cudaContext));
  CONSUMER_PRINT("Done.\n");
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CudaFrameAcquire::CudaFrameAcquire(CUeglStreamConnection& connection)
                                   : m_connection(connection)
                                   , m_stream(NULL), m_resource(0) {
  CUresult result = cuEGLStreamConsumerAcquireFrame(&m_connection, &m_resource, &m_stream, -1);
  if (result == CUDA_SUCCESS) {
    cuGraphicsResourceGetMappedEglFrame(&m_frame, m_resource, 0, 0);
  }
}

CudaFrameAcquire::~CudaFrameAcquire() {
  if (m_resource) {
    cuEGLStreamConsumerReleaseFrame(&m_connection, m_resource, &m_stream);
  }
}

bool CudaFrameAcquire::publish(bool leftFrame) {
  CUDA_RESOURCE_DESC cudaResourceDesc;
  memset(&cudaResourceDesc, 0, sizeof(cudaResourceDesc));
  cudaResourceDesc.resType = CU_RESOURCE_TYPE_ARRAY;

  cudaResourceDesc.res.array.hArray = m_frame.frame.pArray[0];
  CUsurfObject cudaSurfObj1 = 0;
  CUresult cuResult = cuSurfObjectCreate(&cudaSurfObj1, &cudaResourceDesc);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to create surface object 1 (%s)", getCudaErrorString(cuResult));
  }
  
  cudaResourceDesc.res.array.hArray = m_frame.frame.pArray[1];
  CUsurfObject cudaSurfObj2 = 0;
  cuResult = cuSurfObjectCreate(&cudaSurfObj2, &cudaResourceDesc);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to create surface object 2 (%s)", getCudaErrorString(cuResult));
  }

  float delta = convert(cudaSurfObj1, cudaSurfObj2, m_frame.width, m_frame.height, oBuffer);
  cuSurfObjectDestroy(cudaSurfObj1);
  cuSurfObjectDestroy(cudaSurfObj2);

  sensor_msgs::Image output;
  output.header.stamp = ros::Time::now();
  sensor_msgs::fillImage(output, sensor_msgs::image_encodings::BGR8, m_frame.height, m_frame.width, 3 * m_frame.width, (void*) oBuffer);

  if (leftFrame) {
    left_image_pub.publish(output);
  } else {
    right_image_pub.publish(output);
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

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
  printf("CAMERA DEVICES COUNT: %d\n", cameraDevices.size());
  if (cameraDevices.size() < 2) {
    ORIGINATE_ERROR("Must have at least 2 sensors available");
  }

  std::vector <CameraDevice*> lrCameras;
  lrCameras.push_back(cameraDevices[0]);
  lrCameras.push_back(cameraDevices[2]);

  UniqueObj<CaptureSession> captureSession(iCameraProvider->createCaptureSession(lrCameras));
  ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(captureSession);
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
  iEGLStreamSettings->setMode(EGL_STREAM_MODE_MAILBOX);
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

  UniqueObj<Request> request(iCaptureSession->createRequest());
  IRequest *iRequest = interface_cast<IRequest>(request);
  if (!iRequest) {
    ORIGINATE_ERROR("Failed to create Request");
  }

  iRequest->enableOutputStream(streamLeft.get());
  iRequest->enableOutputStream(streamRight.get());

  ISourceSettings *iSourceSettings = interface_cast<ISourceSettings>(request);
  if (!iSourceSettings) {
    ORIGINATE_ERROR("Failed to get source settings request interface");
  }
  iSourceSettings->setFrameDurationRange(Range<uint64_t>(1e9 / FRAMERATE));
  iSourceSettings->setExposureTimeRange(EXPOSURE_TIME_RANGE);
  iSourceSettings->setGainRange(GAIN_RANGE);
  
  IAutoControlSettings *iAutoControlSettings = 
	  interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());
  iAutoControlSettings->setIspDigitalGainRange(ISP_DIGITAL_GAIN_RANGE);

  PRODUCER_PRINT("Launching disparity checking consumer\n");
  StereoConsumer disparityConsumer(iStreamLeft, iStreamRight);
  PROPAGATE_ERROR(disparityConsumer.initialize());
  PROPAGATE_ERROR(disparityConsumer.waitRunning());

  PRODUCER_PRINT("Starting repeat capture requests.\n");
  if (iCaptureSession->repeat(request.get()) != STATUS_OK) {
    ORIGINATE_ERROR("Failed to start repeat capture request for preview");
  }

  ros::spin();

  iCaptureSession->stopRepeat();
  iCaptureSession->waitForIdle();

  PRODUCER_PRINT("Captures complete, disconnecting producer.\n");
  iStreamLeft->disconnect();
  iStreamRight->disconnect();

  PROPAGATE_ERROR(disparityConsumer.shutdown());
  cameraProvider.reset();
  PROPAGATE_ERROR(g_display.cleanup());

  PRODUCER_PRINT("Done -- exiting.\n");
  ros::shutdown();
  
  return true;
}

};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "argus_stereo_node");
  ros::NodeHandle nh;

  left_image_pub = nh.advertise<sensor_msgs::Image>("/camera/left/image_raw", 1);
  left_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/left/camera_info", 1);
  right_image_pub = nh.advertise<sensor_msgs::Image>("/camera/right/image", 1);
  right_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/right/camera_info", 1);
  
  if (!ArgusSamples::execute()) {
    delete[] oBuffer;
    return EXIT_FAILURE;
  }
  delete[] oBuffer;
  return EXIT_SUCCESS;
}
