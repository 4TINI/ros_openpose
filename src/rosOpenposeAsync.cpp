// todo: merge the 'for' loop for body and hand keypoints into one

// OpenPose headers
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>
#include <realtime_tools/realtime_publisher.h>

// ros_openpose headers
#include <human_skeleton_msgs/SkeletonArray.h>
#include <ros_openpose/cameraReader.hpp>

// define a macro for compatibility with older versions
#define OPENPOSE1POINT6_OR_HIGHER OpenPose_VERSION_MAJOR >= 1 && OpenPose_VERSION_MINOR >= 6
#define OPENPOSE1POINT7POINT1_OR_HIGHER OpenPose_VERSION_MAJOR >= 1 && OpenPose_VERSION_MINOR >= 7 && OpenPose_VERSION_PATCH >=1

// define sleep for input and output worker in milliseconds
const int SLEEP_MS = 10;

// define a few datatype
typedef std::shared_ptr<op::Datum> sPtrDatum;
typedef std::shared_ptr<std::vector<sPtrDatum>> sPtrVecSPtrDatum;

// the input worker. the job of this worker is to provide color imagees to
// openpose wrapper
class WUserInput : public op::WorkerProducer<sPtrVecSPtrDatum>
{
public:
  WUserInput(const std::shared_ptr<ros_openpose::CameraReader>& sPtrCameraReader) : mSPtrCameraReader(sPtrCameraReader)
  {
  }

  void initializationOnThread()
  {
  }

  sPtrVecSPtrDatum workProducer()
  {
    try
    {
      auto frameNumber = mSPtrCameraReader->getFrameNumber();
      if (frameNumber == 0 || frameNumber == mFrameNumber)
      {
        // display the error at most once per 10 seconds
        ROS_WARN_THROTTLE(10, "Waiting for color image frame...");
        std::this_thread::sleep_for(std::chrono::milliseconds{SLEEP_MS});
        return nullptr;
      }
      else
      {
        // update frame number
        mFrameNumber = frameNumber;

        // get the latest color image from the camera
        auto& colorImage = mSPtrCameraReader->getColorFrame();

        if (!colorImage.empty())
        {
          // create new datum
          auto datumsPtr = std::make_shared<std::vector<sPtrDatum>>();
          datumsPtr->emplace_back();
          auto& datumPtr = datumsPtr->at(0);
          datumPtr = std::make_shared<op::Datum>();

// fill the datum
#if OPENPOSE1POINT6_OR_HIGHER
          datumPtr->cvInputData = OP_CV2OPCONSTMAT(colorImage);
#else
          datumPtr->cvInputData = colorImage;
#endif
          return datumsPtr;
        }
        else
        {
          // display the error at most once per 10 seconds
          ROS_WARN_THROTTLE(10, "Empty color image frame detected. Ignoring...");
          return nullptr;
        }
      }
    }
    catch (const std::exception& e)
    {
      this->stop();
      // display the error at most once per 10 seconds
      ROS_ERROR_THROTTLE(10, "Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__,
                         __FILE__);
      return nullptr;
    }
  }

private:
  ullong mFrameNumber = 0ULL;
  const std::shared_ptr<ros_openpose::CameraReader> mSPtrCameraReader;
};

// the outpout worker. the job of the output worker is to receive the keypoints
// detected in 2D space. it then converts 2D pixels to 3D coordinates (wrt
// camera coordinate system).
class WUserOutput : public op::WorkerConsumer<sPtrVecSPtrDatum>
{
public:
  // clang-format off
  WUserOutput(const std::shared_ptr<realtime_tools::RealtimePublisher<ros_openpose::SkeletonArray>>& framePublisher,
              const std::shared_ptr<ros_openpose::CameraReader>& sPtrCameraReader,
              const std::string& frameId, const bool /*noDepth*/)
    : mFramePublisher(framePublisher), mSPtrCameraReader(sPtrCameraReader)
  {
    mFrame.header.frame_id = frameId;
  }
  // clang-format on

  void initializationOnThread()
  {
  }

  template <typename Array>
  void fillBodyROSMsg(Array& poseKeypoints, int person, int bodyPartCount)
  {
#pragma omp parallel for
    // for (auto bodyPart = 0; bodyPart < bodyPartCount; bodyPart++)
    for (auto bodyPart = 0; bodyPart < bodyPartCount; bodyPart++)
    {
      // src:
      // https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md#keypoint-format-in-the-c-api
      const auto baseIndex = poseKeypoints.getSize(2) * (person * bodyPartCount + bodyPart);

      const auto x = poseKeypoints[baseIndex];
      const auto y = poseKeypoints[baseIndex + 1];
      const auto score = poseKeypoints[baseIndex + 2];

      // mFrame.skeletons[person].body_parts[bodyPart].score = score;
      mFrame.skeletons[person].bodyParts[bodyPart].pixel.x = x;
      mFrame.skeletons[person].bodyParts[bodyPart].pixel.y = y;
      mFrame.skeletons[person].bodyParts[bodyPart].score = score;
    }
  }
  template <typename ArrayOfArray>
  void fillHandROSMsg(ArrayOfArray& handKeypoints, int person, int handPartCount)
  {
#pragma omp parallel for
    for (auto handPart = 0; handPart < handPartCount; handPart++)
    {
      const auto baseIndex = handKeypoints[0].getSize(2) * (person * handPartCount + handPart);

      // left hand
      const auto xLeft = handKeypoints[0][baseIndex];
      const auto yLeft = handKeypoints[0][baseIndex + 1];
      const auto scoreLeft = handKeypoints[0][baseIndex + 2];

      // right hand
      const auto xRight = handKeypoints[1][baseIndex];
      const auto yRight = handKeypoints[1][baseIndex + 1];
      const auto scoreRight = handKeypoints[1][baseIndex + 2];

      mFrame.skeletons[person].leftHandParts[handPart].pixel.x = xLeft;
      mFrame.skeletons[person].leftHandParts[handPart].pixel.y = yLeft;
      mFrame.skeletons[person].leftHandParts[handPart].score = scoreLeft;

      mFrame.skeletons[person].rightHandParts[handPart].pixel.x = xRight;
      mFrame.skeletons[person].rightHandParts[handPart].pixel.y = yRight;
      mFrame.skeletons[person].rightHandParts[handPart].score = scoreRight;
    }
  }

  template <typename ArrayOfArray>
  void fillFaceROSMsg(ArrayOfArray& faceKeypoints, int person, int facePartCount)
  {
#pragma omp parallel for
    for (auto facePart = 0; facePart < facePartCount; facePart++)
    {
      const auto baseIndex = faceKeypoints.getSize(2) * (person * facePartCount + facePart);

      // face
      const auto xFace = faceKeypoints[baseIndex];
      const auto yFace = faceKeypoints[baseIndex + 1];
      const auto scoreFace = faceKeypoints[baseIndex + 2];


      mFrame.skeletons[person].faceParts[facePart].pixel.x = xFace;
      mFrame.skeletons[person].faceParts[facePart].pixel.y = yFace;
      mFrame.skeletons[person].faceParts[facePart].score = scoreFace;
    }
  }

  void workConsumer(const sPtrVecSPtrDatum& datumsPtr)
  {
    try
    {
      if (datumsPtr != nullptr && !datumsPtr->empty())
      {
        // update timestamp
        mFrame.header.stamp = ros::Time::now();

        // make sure to clear previous data
        mFrame.skeletons.clear();

        // accesing each element of the keypoints
        const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
        const auto& handKeypoints = datumsPtr->at(0)->handKeypoints;
        const auto& faceKeypoints = datumsPtr->at(0)->faceKeypoints;

        // get the size
        const auto personCount = poseKeypoints.getSize(0);
        const auto bodyPartCount = poseKeypoints.getSize(1);
        const auto handPartCount = handKeypoints[0].getSize(1);
        const auto facePartCount = faceKeypoints.getSize(1); 

        mFrame.skeletons.resize(personCount);

        // update with the new data
        for (auto person = 0; person < personCount; person++)
        {
          mFrame.skeletons[person].bodyParts.resize(bodyPartCount);
          mFrame.skeletons[person].leftHandParts.resize(handPartCount);
          mFrame.skeletons[person].rightHandParts.resize(handPartCount);
          mFrame.skeletons[person].faceParts.resize(facePartCount);          

          fillBodyROSMsg(poseKeypoints, person, bodyPartCount);
          fillHandROSMsg(handKeypoints, person, handPartCount);
          fillFaceROSMsg(faceKeypoints, person, facePartCount);
        }        
        if (mFramePublisher->trylock())
        {
          mFramePublisher->msg_= mFrame;
          // mFramePublisher.publish(mFrame);
          mFramePublisher->unlockAndPublish();
        }
      }
      else
      {
        // display the error at most once per 10 seconds
        ROS_WARN_THROTTLE(10, "Waiting for datum...");
        std::this_thread::sleep_for(std::chrono::milliseconds{SLEEP_MS});
      }
    }
    catch (const std::exception& e)
    {
      this->stop();
      ROS_ERROR("Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
  }

private:
  ros_openpose::SkeletonArray mFrame;
  std::shared_ptr<realtime_tools::RealtimePublisher<ros_openpose::SkeletonArray>> mFramePublisher;


  const std::shared_ptr<ros_openpose::CameraReader> mSPtrCameraReader;
};

// clang-format off
void configureOpenPose(op::Wrapper& opWrapper,
                       const std::shared_ptr<ros_openpose::CameraReader>& cameraReader,
                       const std::shared_ptr<realtime_tools::RealtimePublisher<ros_openpose::SkeletonArray>>& framePublisher,
                       const std::string& frameId)
// clang-format on
{
  try
  {
// Configuring OpenPose

// clang-format off
// logging_level
#if OPENPOSE1POINT6_OR_HIGHER
    op::checkBool(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255,
#else
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255,
#endif
              "Wrong logging_level value.",
              __LINE__,
              __FUNCTION__,
              __FILE__);

    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
    op::Profiler::setDefaultX(FLAGS_profile_speed);

// Applying user defined configuration - GFlags to program variables
// outputSize
#if OPENPOSE1POINT6_OR_HIGHER
    const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
#else
    const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
#endif

// netInputSize
#if OPENPOSE1POINT6_OR_HIGHER
    const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
#else
    const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
#endif

// faceNetInputSize
#if OPENPOSE1POINT6_OR_HIGHER
    const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
#else
    const auto faceNetInputSize = op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)");
#endif

// handNetInputSize
#if OPENPOSE1POINT6_OR_HIGHER
    const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
#else
    const auto handNetInputSize = op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)");
#endif

    // poseMode
    const auto poseMode = op::flagsToPoseMode(FLAGS_body);

// poseModel
#if OPENPOSE1POINT6_OR_HIGHER
    const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
#else
    const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
#endif

    // JSON saving
    if (!FLAGS_write_keypoint.empty())
      ROS_INFO("Flag `write_keypoint` is deprecated and will eventually be removed. Please, use `write_json` instead.");

    // keypointScaleMode
    const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);

    // heatmaps to add
    const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts,
                                                  FLAGS_heatmaps_add_bkg,
                                                  FLAGS_heatmaps_add_PAFs);

    const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);

    // >1 camera view?
    // const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1 || FLAGS_flir_camera);
    const auto multipleView = false;

    // Face and hand detectors
    const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
    const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);

    // Enabling Google Logging
    const bool enableGoogleLogging = true;

    // Initializing the user custom classes
    auto wUserInput = std::make_shared<WUserInput>(cameraReader);
    auto wUserOutput = std::make_shared<WUserOutput>(framePublisher, cameraReader, frameId, false);

    // Add custom processing
    const auto workerInputOnNewThread = true;
    opWrapper.setWorker(op::WorkerType::Input, wUserInput, workerInputOnNewThread);

    const auto workerOutputOnNewThread = true;
    opWrapper.setWorker(op::WorkerType::Output, wUserOutput, workerOutputOnNewThread);

    // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
    const op::WrapperStructPose wrapperStructPose{poseMode,
                                                  netInputSize,
#if OPENPOSE1POINT7POINT1_OR_HIGHER
                                                  FLAGS_net_resolution_dynamic,
                                                  outputSize,
#else
                                                  outputSize,
#endif
                                                  keypointScaleMode,
                                                  FLAGS_num_gpu,
                                                  FLAGS_num_gpu_start,
                                                  FLAGS_scale_number,
                                                  (float)FLAGS_scale_gap,
                                                  op::flagsToRenderMode(FLAGS_render_pose,
                                                                        multipleView),
                                                  poseModel,
                                                  !FLAGS_disable_blending,
                                                  (float)FLAGS_alpha_pose,
                                                  (float)FLAGS_alpha_heatmap,
                                                  FLAGS_part_to_show,
#if OPENPOSE1POINT6_OR_HIGHER
                                                  op::String(FLAGS_model_folder),
#else
                                                  FLAGS_model_folder,
#endif
                                                  heatMapTypes,
                                                  heatMapScaleMode,
                                                  FLAGS_part_candidates,
                                                  (float)FLAGS_render_threshold,
                                                  FLAGS_number_people_max,
                                                  FLAGS_maximize_positives,
                                                  FLAGS_fps_max,
#if OPENPOSE1POINT6_OR_HIGHER
                                                  op::String(FLAGS_prototxt_path),
                                                  op::String(FLAGS_caffemodel_path),
#else
                                                  FLAGS_prototxt_path,
                                                  FLAGS_caffemodel_path,
#endif
                                                  (float)FLAGS_upsampling_ratio,
                                                  enableGoogleLogging};
    opWrapper.configure(wrapperStructPose);

    // Face configuration (use op::WrapperStructFace{} to disable it)
    const op::WrapperStructFace wrapperStructFace{FLAGS_face,
                                                  faceDetector,
                                                  faceNetInputSize,
                                                  op::flagsToRenderMode(FLAGS_face_render,
                                                                        multipleView,
                                                                        FLAGS_render_pose),
                                                  (float)FLAGS_face_alpha_pose,
                                                  (float)FLAGS_face_alpha_heatmap,
                                                  (float)FLAGS_face_render_threshold};
    opWrapper.configure(wrapperStructFace);

    // Hand configuration (use op::WrapperStructHand{} to disable it)
    const op::WrapperStructHand wrapperStructHand{FLAGS_hand,
                                                  handDetector,
                                                  handNetInputSize,
                                                  FLAGS_hand_scale_number,
                                                  (float)FLAGS_hand_scale_range,
                                                  op::flagsToRenderMode(FLAGS_hand_render,
                                                                        multipleView,
                                                                        FLAGS_render_pose),
                                                  (float)FLAGS_hand_alpha_pose,
                                                  (float)FLAGS_hand_alpha_heatmap,
                                                  (float)FLAGS_hand_render_threshold};
    opWrapper.configure(wrapperStructHand);

    // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
    const op::WrapperStructExtra wrapperStructExtra{FLAGS_3d,
                                                    FLAGS_3d_min_views,
                                                    FLAGS_identification,
                                                    FLAGS_tracking,
                                                    FLAGS_ik_threads};
    opWrapper.configure(wrapperStructExtra);

    // Output (comment or use default argument to disable any output)
    const op::WrapperStructOutput wrapperStructOutput{FLAGS_cli_verbose,
#if OPENPOSE1POINT6_OR_HIGHER
                                                      op::String(FLAGS_write_keypoint),
#else
                                                      FLAGS_write_keypoint,
#endif
                                                      op::stringToDataFormat(FLAGS_write_keypoint_format),
#if OPENPOSE1POINT6_OR_HIGHER
                                                      op::String(FLAGS_write_json),
                                                      op::String(FLAGS_write_coco_json),
#else
                                                      FLAGS_write_json,
                                                      FLAGS_write_coco_json,
#endif
                                                      FLAGS_write_coco_json_variants,
                                                      FLAGS_write_coco_json_variant,
#if OPENPOSE1POINT6_OR_HIGHER
                                                      op::String(FLAGS_write_images),
                                                      op::String(FLAGS_write_images_format),
                                                      op::String(FLAGS_write_video),
#else
                                                      FLAGS_write_images,
                                                      FLAGS_write_images_format,
                                                      FLAGS_write_video,
#endif
                                                      FLAGS_write_video_fps,
                                                      FLAGS_write_video_with_audio,
#if OPENPOSE1POINT6_OR_HIGHER
                                                      op::String(FLAGS_write_heatmaps),
                                                      op::String(FLAGS_write_heatmaps_format),
                                                      op::String(FLAGS_write_video_3d),
                                                      op::String(FLAGS_write_video_adam),
                                                      op::String(FLAGS_write_bvh),
                                                      op::String(FLAGS_udp_host),
                                                      op::String(FLAGS_udp_port)};
#else
                                                      FLAGS_write_heatmaps,
                                                      FLAGS_write_heatmaps_format,
                                                      FLAGS_write_video_3d,
                                                      FLAGS_write_video_adam,
                                                      FLAGS_write_bvh,
                                                      FLAGS_udp_host,
                                                      FLAGS_udp_port};
#endif
    opWrapper.configure(wrapperStructOutput);

    // GUI (comment or use default argument to disable any visual output)
    const op::WrapperStructGui wrapperStructGui{op::flagsToDisplayMode(FLAGS_display,
                                                                       FLAGS_3d),
                                                !FLAGS_no_gui_verbose,
                                                FLAGS_fullscreen};
    opWrapper.configure(wrapperStructGui);
    // clang-format on

    // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
    if (FLAGS_disable_multi_thread)
      opWrapper.disableMultiThreading();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ros_openpose_node");
  ros::NodeHandle nh("~");

  // define the parameters, we are going to read
  std::string colorTopic, frameId, pubTopic;

  // read the parameters from relative nodel handle
  nh.getParam("frame_id", frameId);
  nh.getParam("pub_topic", pubTopic);
  nh.getParam("color_topic", colorTopic);

  if (pubTopic.empty())
  {
    ROS_FATAL("Missing 'pub_topic' info in launch file. Please make sure that you have executed 'run.launch' file.");
    exit(-1);
  }

  // parsing command line flags
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const auto cameraReader = std::make_shared<ros_openpose::CameraReader>(nh, colorTopic);

  // the frame consists of the location of detected body parts of each person
  // const ros::Publisher framePublisher = nh.advertise<ros_openpose::SkeletonArray>(pubTopic, 1);
  std::shared_ptr<realtime_tools::RealtimePublisher<ros_openpose::SkeletonArray>> framePublisher;
  framePublisher = std::make_shared<realtime_tools::RealtimePublisher<ros_openpose::SkeletonArray>>(nh, pubTopic, 1);

  try
  {
    ROS_INFO("Starting ros_openpose...");
    op::Wrapper opWrapper;
    configureOpenPose(opWrapper, cameraReader, framePublisher, frameId);

    // start and run
    opWrapper.start();

    // exit when Ctrl-C is pressed, or the node is shutdown by the master
    ros::spin();

    // return successful message
    ROS_INFO("Exiting ros_openpose...");

    // stop processing
    opWrapper.stop();
    return 0;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
    return -1;
  }
}
