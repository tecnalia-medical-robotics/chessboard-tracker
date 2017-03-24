/**
 * author: Vaibhav Mehta <vaibhavmehta.in@gmail.com>
 */

#include <fast_checkerboard_detector/fast_checkerboard_detector.h>

namespace fast_checkerboard_detector
{


/**
 * @brief Get the transformation matrix from a m3x3 rotation matrix and a m1x3
 * or m3x1 translation matrix.
 * @param r m3x3 rotation matrix
 * @param t m1x3 or m3x1 translation matrix
 * @param m4x4 TRanformation matrix
 */
void RTMatrices2TransformMatrix(const cv::Mat& r, const cv::Mat& t,
                                cv::Mat& m4x4)
{
  m4x4 = cv::Mat::eye(4, 4, CV_64F);
  cv::Mat submatR = cv::Mat(m4x4, cv::Rect(0, 0, 3, 3));
  cv::Mat submatT = cv::Mat(m4x4, cv::Rect(3, 0, 1, 3));
  r.copyTo(submatR);
  if (t.cols == 3)
  {
    cv::Mat Tt = t.t();
    Tt.copyTo(submatT);
  }
  else
    t.copyTo(submatT);
  /*std::cout << "Rotation matrix: " << R << std::endl;
  std::cout << "Translation matrix: " << T << std::endl;
  std::cout << "Transformation matrix: " << m4x4 << std::endl;*/
}

/**
 * @brief Get the rotation matrix and translation matrix from transformation matrix m4x4
 * @param m4x4 Tranformation matrix
 * @param r m3x3 rotation matrix
 * @param t m1x3 or m3x1 translation matrix
 */
void TransformMatrix2RTMatrices(const cv::Mat& m4x4, cv::Mat& r, cv::Mat& t)
{
  r = cv::Mat::eye(3, 3, CV_64F);
  t = cv::Mat::zeros(1, 3, CV_64F);

  r = cv::Mat(m4x4, cv::Rect(0, 0, 3, 3));
  t = cv::Mat(m4x4, cv::Rect(3, 0, 1, 3));

  /*std::cout << "Rotation matrix: " << r << std::endl;
  std::cout << "Translation matrix: " << t << std::endl;
  std::cout << "Transformation matrix: " << m4x4 << std::endl;*/
}

/**
 * @brief rodrigues_to_matrix Converts transformation from rodrigues form to 4x4
 * matrix form
 * @param r input rotation vector in rodrigues form
 * @param t input translation vector
 * @param m4x4 ouput 4x4 matrix transformation
 */
void Rodrigues2Matrix(const cv::Mat& r, const cv::Mat& t, cv::Mat& m4x4)
{

  cv::Mat r64, t64;
  r.convertTo(r64, CV_64F);
  t.convertTo(t64, CV_64F);

  m4x4 = cv::Mat::eye(4, 4, r64.type());

  cv::Mat submat3x3 = cv::Mat(m4x4, cv::Rect(0, 0, 3, 3));
  cv::Rodrigues(r64, submat3x3);

  cv::Mat submatT = cv::Mat(m4x4, cv::Rect(3, 0, 1, 3));
  t64.copyTo(submatT);
}




FastCheckerboardDetector::FastCheckerboardDetector(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
    it_(nh),
    cv_translation(3, 1, cv::DataType<double>::type, translation),
    cv_rotation(3, 1, cv::DataType<double>::type, rotation),
    cv_distortion(cv::Mat::zeros(1, 5, cv::DataType<float>::type))
  {
    if(!nh_private.getParam("grid_size_x", grid_size_x))
    {
      ROS_ERROR("Missing parameter ''!");
    }
    if(!nh_private.getParam("grid_size_y", grid_size_y))
    {
      ROS_ERROR("Missing parameter 'grid_size_y'!");
    }
    if(!nh_private.getParam("rect_size_x", rect_size_x))
    {
      ROS_ERROR("Missing parameter 'rect_size_x'!");
    }
    if(!nh_private.getParam("rect_size_y", rect_size_y))
    {
      ROS_ERROR("Missing parameter 'rect_size_y'!");
    }
    if(!nh_private.getParam("write_trajectory", writePoseToFile))
    {
      ROS_ERROR("Missing parameter 'write_trajectory'!");
    }
    if(!nh_private.getParam("show_image", show_image_))
    {
      ROS_ERROR("Missing parameter 'show_image!");
    }
    corners_.reserve(grid_size_x * grid_size_y);

    double x_offset = rect_size_x * (grid_size_x - 1) / 2.0;
    double y_offset = rect_size_y * (grid_size_y - 1) / 2.0;

    for(int y = 0; y < grid_size_y; y++)
    {
      for(int x = 0; x < grid_size_x; x++)
      {
        object_points_.push_back(cv::Point3f(x * rect_size_x - x_offset, y * rect_size_y - y_offset, 0));
      }
    }
    
    if(writePoseToFile)
    {
      if(nh_private.getParam("trajectory_file", TrajectoryFile))
      {
        trajectory_out_ = new std::ofstream(TrajectoryFile.c_str());
        if(trajectory_out_->fail())
        {
          delete trajectory_out_;

          std::cerr << "Failed to open '" << "Checkerboard_marker_trajectory.txt" << "'!" << std::endl;
        }
      }
      else
      {
        trajectory_out_ = &std::cout;
      }
    }


    camera_pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    camera_info_subscriber_ = nh.subscribe("camera_info", 1, &FastCheckerboardDetector::handleCameraInfo, this);

    resetROI();
  }

void FastCheckerboardDetector::handleCameraInfo(const sensor_msgs::CameraInfoConstPtr &info)
{
  camera_info_subscriber_.shutdown();

  intrinsic_matrix_ = cv::Mat(3, 3, cv::DataType<float>::type);

  const double* info_ptr = info->K.begin();

  for(int row_idx = 0; row_idx < intrinsic_matrix_.rows; ++row_idx)
  {
      float* row_ptr = intrinsic_matrix_.ptr<float>(row_idx);

      for(int col_idx = 0; col_idx < intrinsic_matrix_.cols; ++col_idx, row_ptr++, info_ptr++)
      {
        *row_ptr = (float) *info_ptr;
      }
      // skip 4th column
      //info_ptr++;
  }


  distortion_vector = info->D;

  img_subscriber_ = it_.subscribe("camera_image", 1, &FastCheckerboardDetector::handleImageMessage, this);
}


void FastCheckerboardDetector::handleImageMessage(const sensor_msgs::ImageConstPtr& message)
{
  cv_bridge::CvImageConstPtr image_bridge = cv_bridge::toCvShare(message, "mono8");

  cv_bridge::CvImageConstPtr image_color_bridge;
  if (show_image_)
  {
    image_color_bridge = cv_bridge::toCvShare(message, "bgr8");    
  }

  const cv::Mat image = image_bridge->image;

  limitROI(image);

  //ROS_INFO_STREAM("ROI x: " << roi_.x << " y: " << roi_.y << " width: " << roi_.width << " height: " << roi_.height);
  // todo the original code was using the grey image, and not the color one.
  bool found_chessboard = cv::findChessboardCorners(image_color_bridge->image, cv::Size(grid_size_x, grid_size_y), corners_, 
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

  fixCorners();
  // todo check if we want to reactivate that functionality
  //updateROI();

  if(found_chessboard)
  {
    // todo: in our code, the last flag is 0.1
    cv::cornerSubPix(image, corners_, cv::Size(4, 4), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));
    
    cv::solvePnP(object_points_, corners_, intrinsic_matrix_, distortion_vector, cv_rotation, cv_translation);
    
    // if (show_image_)
    // {
    //   cv::Mat image_color = image_color_bridge->image;
    //   cv::drawChessboardCorners(image_color, cv::Size(grid_size_x,grid_size_y), cv::Mat(corners_), found_chessboard);
    //   drawAxis(image_color, intrinsic_matrix_, distortion_vector, cv_rotation, cv_translation, 0.1);
    //   cv::imshow("Corners", image_color);
    //   cv::waitKey(5);
    // }

    ROS_INFO_STREAM("solvePNP" << std::endl << cv_rotation << " " << cv_translation);


    Eigen::Vector3d rotation_vector(rotation[0], rotation[1], rotation[2]);
    Eigen::Translation3d translation_(translation[0], translation[1], translation[2]);

    cv::Mat camMchess;

    Rodrigues2Matrix(cv_rotation, cv_translation, camMchess);

    ROS_INFO_STREAM("in Matrix: " << std::endl << camMchess);


    cv::Mat chessMchess2 = cv::Mat::eye(4, 4, CV_64F);
    chessMchess2.at<double>(0, 3) = - (grid_size_x + 1) * rect_size_x / 2.0;
    chessMchess2.at<double>(1, 3) = - (grid_size_y + 1) * rect_size_y / 2.0;

    std::cout << "chess transform: " << chessMchess2 << std::endl;
    cv::Mat r2, t2;

    cv::Mat camMchess2 = camMchess * chessMchess2;

    TransformMatrix2RTMatrices(camMchess2, r2,t2);


    ROS_INFO_STREAM("Eigen" << std::endl << r2 << " " << t2);

    Eigen::AngleAxisd rotation_;

    if(!rotation_vector.isZero(1e-6))
    {
      rotation_ = Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized());
    }
    else
    {
      rotation_ = Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1));
    }

    Eigen::Affine3d transform = translation_ * rotation_;
    

    // Eigen::Affine3d camMchess2, chessMchess2;
    // Eigen::AngleAxisd r_id = Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1));
    // Eigen::Vector3d chesstchess2(grid_size_x * rect_size_x /2.0, grid_size_y * rect_size_y /2.0, 0);

    // chesstchess2 = chesstchess2 * r_id;
    // camMchess2 = camMchess * chessMchess2;

    // cv::Mat cv_crc2, cv_ctc2;
    // cv_crc2 = cv::Mat(1, 3, CV_64F);
    // cv_ctc2 = cv::Mat(1, 3, CV_64F);

    // Eigen::Translation3d t =  camMchess2.translation();
    // cv_tc2.at<double>(0,0) = t.x();

    if (show_image_)
    {
      cv::Mat image_color = image_color_bridge->image;
      cv::drawChessboardCorners(image_color, cv::Size(grid_size_x,grid_size_y), cv::Mat(corners_), found_chessboard);
      drawAxis(image_color, intrinsic_matrix_, distortion_vector, cv_rotation, cv_translation, 0.1);
      
      drawAxis(image_color, intrinsic_matrix_, distortion_vector, r2, t2, 0.1);

      cv::imshow("Corners", image_color);
      cv::waitKey(5);
    }


    ROS_INFO_STREAM("Eigen Matrix" << std::endl << transform.matrix());
    // disambiguate by tracking..
    Eigen::Affine3d rotated_transform = transform; //* Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0,0,1));
    Eigen::Affine3d diff = previous_transform_.inverse() * transform;
    Eigen::Affine3d final_transform = diff(0, 0) >= 0 ? transform : rotated_transform;

    previous_transform_ = final_transform;

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = message->header.frame_id;
    msg.header.stamp = message->header.stamp;

    tf::poseEigenToMsg(final_transform, msg.pose);

    camera_pose_publisher_.publish(msg);

    if(writePoseToFile)
      {
        Eigen::Affine3d camera_pose = final_transform;
        Eigen::Quaterniond q(camera_pose.rotation());

        (*trajectory_out_)
            << msg.header.stamp << " "
            << camera_pose.translation()(0) << " "
            << camera_pose.translation()(1) << " "
            << camera_pose.translation()(2) << " "
            << q.x() << " "
            << q.y() << " "
            << q.z() << " "
            << q.w() << " "
            << std::endl;
      }

    // transform broadcast
    // tf::Transform tf_transform;
    // tf::transformEigenToTF(final_transform, tf_transform);

    // broadcast_.sendTransform(tf::StampedTransform(tf_transform,
    //                                               ros::Time::now(),
    //                                               "cam",
    //                                               "chessboard"));

  }
  else
  {
    resetROI();
     if (show_image_)
     {
      cv::Mat image_color = image_color_bridge->image;
      cv::imshow("Corners", image_color);
      cv::waitKey(5);
     }
    ROS_WARN_STREAM("Checkerboard tracking lost!");
  }
  tf::Transform tf_transform;
  Eigen::Affine3d inversed_transform = previous_transform_.inverse();
  tf::transformEigenToTF(inversed_transform, tf_transform);

  broadcast_.sendTransform(tf::StampedTransform(tf_transform,
                                                ros::Time::now(),
                                                "chessboard",
                                                "cam"));
}

/**
 * @brief drawAxis Draw a RGB axis in a image for a given coordinate frame
 * @param image input-output image
 * @param camera_matrix image camera matrix
 * @param dist_coeffs image distortion coefficients vector
 * @param rvec coordinate frame rotation vector in rodrigues format (from world
 * to camera)
 * @param tvec coordinate frame translation vector (from world to camera)
 * @param length axis lenght in same unit than tvec
 */
void FastCheckerboardDetector::drawAxis(cv::Mat& image, cv::Mat camera_matrix,
                                        cv::InputArray dist_coeffs, cv::Mat rvec, cv::Mat tvec,
                                        float length)
{
  // project axis points
  std::vector<cv::Point3f> axis_points;
  axis_points.push_back(cv::Point3f(0, 0, 0));
  axis_points.push_back(cv::Point3f(length, 0, 0));
  axis_points.push_back(cv::Point3f(0, length, 0));
  axis_points.push_back(cv::Point3f(0, 0, length));
  std::vector<cv::Point2f> imagePoints;
  cv::projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs,
                    imagePoints);

  //draw3DCoordinateAxes(image, imagePoints);
  // draw axis lines
  cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3);
  cv::line(image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 3);
  cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);
}


void FastCheckerboardDetector::resetROI()
{
  roi_ = cv::Rect(0, 0, 1280, 960);
}

void FastCheckerboardDetector::limitROI(const cv::Mat& image)
{
  roi_ &= cv::Rect(0, 0, image.cols, image.rows);
}

void FastCheckerboardDetector::fixCorners()
{
  BOOST_FOREACH(cv::Point2f& p, corners_)
  {
    p.x += roi_.x;
    p.y += roi_.y;
  }
}

void FastCheckerboardDetector::updateROI()
{
  // corner indices
  static size_t c1 = 0, c2 = grid_size_x - 1, c3 = grid_size_x * grid_size_y - grid_size_x, c4 = grid_size_x * grid_size_y - 1;

  int min_x = (int) std::min(corners_[c1].x, std::min(corners_[c2].x, std::min(corners_[c3].x, corners_[c4].x)));
  int min_y = (int) std::min(corners_[c1].y, std::min(corners_[c2].y, std::min(corners_[c3].y, corners_[c4].y)));
  int max_x = (int) std::max(corners_[c1].x, std::max(corners_[c2].x, std::max(corners_[c3].x, corners_[c4].x)));
  int max_y = (int) std::max(corners_[c1].y, std::max(corners_[c2].y, std::max(corners_[c3].y, corners_[c4].y)));

  cv::Rect updated_roi(cv::Point(min_x, min_y), cv::Point(max_x, max_y));

  updated_roi.x -= updated_roi.width / 2.0;
  updated_roi.y -= updated_roi.height / 2.0;
  updated_roi.width *= 2.0;
  updated_roi.height *= 2.0;

  roi_ = updated_roi;
}

} /* namespace fast_checkerboard_detector */
