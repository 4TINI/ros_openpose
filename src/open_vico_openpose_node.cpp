#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>
#include <ros_openpose/SkeletonArray.h>

// define a macro for compatibility with older versions
#define OPENPOSE1POINT6_OR_HIGHER OpenPose_VERSION_MAJOR >= 1 && OpenPose_VERSION_MINOR >= 6
#define OPENPOSE1POINT7POINT1_OR_HIGHER OpenPose_VERSION_MAJOR >= 1 && OpenPose_VERSION_MINOR >= 7 && OpenPose_VERSION_PATCH >=1

class OpenVICOOpenPose
{
    public:
        OpenVICOOpenPose(op::Wrapper* op_wrapper) : 
                        nh_("~"), op_wrapper_(op_wrapper)
        {
            ROS_INFO("OpenVICOOpenPose()");
            skeleton_pub_ = nh_.advertise<open_vico_msgs::SkeletonArray>("skeletons", 1);
            color_img_sub_ = nh_.subscribe("rgb/image_raw", 1, &OpenVICOOpenPose::colorImgCallback, this, 
            ros::TransportHints().reliable().tcpNoDelay());
        }

        virtual ~OpenVICOOpenPose() = default;
    private:
        ros::NodeHandle nh_;
        op::Wrapper* op_wrapper_;
        ros::Publisher skeleton_pub_;
        ros::Subscriber color_img_sub_;
        std::string frame_id_;
        cv::Mat color_img_;

        void colorImgCallback(const sensor_msgs::ImageConstPtr& msg)
        {
            color_img_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

            // Fill datum
#if OPENPOSE1POINT6_OR_HIGHER
            auto datum_ptr = op_wrapper_->emplaceAndPop(OP_CV2OPCONSTMAT(color_img_));
#else
            auto datum_ptr = op_wrapper_->emplaceAndPop(color_img_);
#endif

            const auto& pose_kp = datum_ptr->at(0)->poseKeypoints;

            // Get the sizes
            const auto num_skeletons = pose_kp.getSize(0);
            const auto num_body_parts = pose_kp.getSize(1);

            open_vico_msgs::SkeletonArray skeletons_msg;
            skeletons_msg.header.stamp = ros::Time::now();
            skeletons_msg.header.frame_id = msg->header.frame_id;
            skeletons_msg.skeletons.resize(num_skeletons);

            // int i;
            for(auto skeleton_cnt = 0; skeleton_cnt < num_skeletons; skeleton_cnt++)
            {
                auto& current_skeleton = skeletons_msg.skeletons[skeleton_cnt];

                current_skeleton.body_parts.resize(num_body_parts);

                // Fill body parts
                for (auto body_part_cnt = 0; body_part_cnt < num_body_parts; body_part_cnt++) 
                {
                    auto& curr_body_part = current_skeleton.body_parts[body_part_cnt];
                    int body_part_ind = pose_kp.getSize(2) * (skeleton_cnt * num_body_parts + body_part_cnt);

                    curr_body_part.x = pose_kp[body_part_ind];
                    curr_body_part.y = pose_kp[body_part_ind+1];
                }
            }

            skeleton_pub_.publish(skeletons_msg);
            ROS_INFO_ONCE("First skeleton published.");
        }

}; // class OpenVICOOpenPose

void configureOpenPose(op::Wrapper& opWrapper);

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "open_vico_openpose");

    // Parse Openpose Args
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    try 
    {
        ROS_INFO("Starting Open-VICO OpenPose...");

        // Initialize Openpose wrapper
        op::Wrapper op_wrapper{op::ThreadManagerMode::Asynchronous};
        configureOpenPose(op_wrapper);
        op_wrapper.start();

        ROS_INFO("Open-VICO OpenPose started.");

        // Start ROS Openpose wrapper
        OpenVICOOpenPose open_vico_openpose(&op_wrapper);

        ros::spin();

        ROS_INFO("Exiting ros_openpose...");

        op_wrapper.stop();

        return 0;
    }
    catch (const std::exception& e) {
        ROS_ERROR("Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
        return -1;
    }
}

void configureOpenPose(op::Wrapper& opWrapper) 
{
    try
    {
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
