#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#include <Argus/Argus.h>
#include <cuda.h>
#include <cudaEGL.h>

#include "ArgusHelpers.h"
#include "CUDAHelper.h"
#include "EGLGlobal.h"
#include "Error.h"
#include "Thread.h"

#include "convert.h"

using namespace Argus;

ros::Publisher left_image_pub;
ros::Publisher right_image_pub;

namespace ArgusSamples {

static const uint32_t FRAMERATE = 60;
static const Size2D<uint32_t> STREAM_SIZE(1920, 1080);

EGLDisplayHolder g_display;
uint8_t* oBuffer = new uint8_t[STREAM_SIZE.width() * STREAM_SIZE.height()];

#define PRODUCER_PRINT(...) printf("PRODUCER: " __VA_ARGS__)
#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)

class StereoConsumer : public Thread {
  public:
    explicit StereoConsumer(IEGLOutputStream *leftStream, 
		            IEGLOutputStream *rightStream)
		            : m_leftStream(leftStream)
			    , m_rightStream(rightStream)
                            , m_cudaContext(0)
                            , m_cuStreamLeft(NULL)
                            , m_cuStreamRight(NULL) {}
    ~StereoConsumer() {}

  private:
    /** @name Thread methods */
    /**@{*/
    virtual bool threadInitialize();
    virtual bool threadExecute();
    virtual bool threadShutdown();
    /**@}*/

    IEGLOutputStream *m_leftStream;
    IEGLOutputStream *m_rightStream;
    CUcontext m_cudaContext;
    CUeglStreamConnection m_cuStreamLeft;
    CUeglStreamConnection m_cuStreamRight;
};

class CudaFrameAcquire {
  public:
    CudaFrameAcquire(CUeglStreamConnection& connection);
    ~CudaFrameAcquire();

    bool hasValidFrame() const;
    bool publish(bool leftFrame);
    Size2D<uint32_t> getSize() const;

  private:
    CUeglStreamConnection& m_connection;
    CUstream m_stream;
    CUgraphicsResource m_resource;
    CUeglFrame m_frame;
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

    if (!left.hasValidFrame() || !right.hasValidFrame()) {
      break;
    }

    if (left.publish(true) && right.publish(false)) {
      CONSUMER_PRINT("Debug\n");
    }
  }
    
  CONSUMER_PRINT("No more frames. Cleaning up.\n");
  PROPAGATE_ERROR(requestShutdown());
  return true;
}

bool StereoConsumer::threadShutdown() {
  CUresult cuResult = cuEGLStreamConsumerDisconnect(&m_cuStreamLeft);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to disconnect CUDA (%s)", getCudaErrorString(cuResult));
  }

  cuResult = cuEGLStreamProducerDisconnect(&m_cuStreamRight);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to disconnect CUDA (%s)", getCudaErrorString(cuResult));
  }

  PROPAGATE_ERROR(cleanupCUDA(&m_cudaContext));
  CONSUMER_PRINT("Done.\n");
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CudaFrameAcquire::CudaFrameAcquire(CUeglStreamConnection& connection)
                                   : m_connection(connection)
                                   , m_stream(NULL)
                                   , m_resource(0) {
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

bool CudaFrameAcquire::hasValidFrame() const {
  return m_resource;
}

bool CudaFrameAcquire::publish(bool leftFrame) {
  CUDA_RESOURCE_DESC cudaResourceDesc;
  memset(&cudaResourceDesc, 0, sizeof(cudaResourceDesc));
  cudaResourceDesc.resType = CU_RESOURCE_TYPE_ARRAY;
  cudaResourceDesc.res.array.hArray = m_frame.frame.pArray[0];
  CUsurfObject cudaSurfObj = 0;
  CUresult cuResult = cuSurfObjectCreate(&cudaSurfObj, &cudaResourceDesc);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to create surface object (%s)", getCudaErrorString(cuResult));
  }
  
  float delta = convert(cudaSurfObj, m_frame.width, m_frame.height, oBuffer);
  cuSurfObjectDestroy(cudaSurfObj);

  sensor_msgs::Image output;
  output.header.stamp = ros::Time::now();
  sensor_msgs::fillImage(output, sensor_msgs::image_encodings::MONO8, m_frame.height, m_frame.width, m_frame.width, (void*) oBuffer);

  if (leftFrame) {
    CONSUMER_PRINT("Left frame output: %f", delta);
    left_image_pub.publish(output);
  } else {
    CONSUMER_PRINT("Right frame output: %f", delta);
    right_image_pub.publish(output);
  }
}

Size2D<uint32_t> CudaFrameAcquire::getSize() const {
  if (hasValidFrame()) {
    return Size2D<uint32_t>(m_frame.width, m_frame.height);
  }
  return Size2D<uint32_t>(0, 0);
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
  if (cameraDevices.size() < 2) {
    ORIGINATE_ERROR("Must have at least 2 sensors available");
  }
  /*SensorMode* sensorMode = ArgusHelpers::getSensorMode(cameraDevices[0], 2);
  ISensorMode *iSensorMode = interface_cast<ISensorMode>(sensorMode);
  if (!iSensorMode) {
    ORIGINATE_ERROR("Selected sensor mode not available");
  }*/

  std::vector <CameraDevice*> lrCameras;
  lrCameras.push_back(cameraDevices[0]);
  lrCameras.push_back(cameraDevices[1]);

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
  iEGLStreamSettings->setMode(EGL_STREAM_MODE_FIFO);

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
  //iSourceSettings->setSensorMode(sensorMode);
  iSourceSettings->setFrameDurationRange(Range<uint64_t>(1e9 / FRAMERATE));

  PRODUCER_PRINT("Launching disparity checking consumer\n");
  StereoConsumer disparityConsumer(iStreamLeft, iStreamRight);
  PROPAGATE_ERROR(disparityConsumer.initialize());
  PROPAGATE_ERROR(disparityConsumer.waitRunning());

  PRODUCER_PRINT("Starting repeat capture requests.\n");
  if (iCaptureSession->repeat(request.get()) != STATUS_OK) {
    ORIGINATE_ERROR("Failed to start repeat capture request for preview");
  }
  sleep(100);

  iCaptureSession->stopRepeat();
  iCaptureSession->waitForIdle();

  PRODUCER_PRINT("Captures complete, disconnecting producer.\n");
  iStreamLeft->disconnect();
  iStreamRight->disconnect();

  PROPAGATE_ERROR(disparityConsumer.shutdown());
  cameraProvider.reset();
  PROPAGATE_ERROR(g_display.cleanup());

  PRODUCER_PRINT("Done -- exiting.\n");
  return true;
}

};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "argus_stereo_node");
  ros::NodeHandle nh;

  left_image_pub = nh.advertise<sensor_msgs::Image>("/camera/left/image", 1);
  right_image_pub = nh.advertise<sensor_msgs::Image>("/camera/right/image", 1);
  if (!ArgusSamples::execute()) {
    delete[] ArgusSamples::oBuffer;
    return EXIT_FAILURE;
  }
  delete[] ArgusSamples::oBuffer;
  return EXIT_SUCCESS;
}
