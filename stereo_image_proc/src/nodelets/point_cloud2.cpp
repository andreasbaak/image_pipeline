/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/stereo_camera_model.h>

#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include "../common/parameters.h"

namespace stereo_image_proc_aurum {

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

class PointCloud2Nodelet : public nodelet::Nodelet
{
public:
  PointCloud2Nodelet()
          : nodelet::Nodelet()
          , STEP(16)
  { };

private:
  boost::shared_ptr<image_transport::ImageTransport> it_;

  // Subscriptions
  image_transport::SubscriberFilter sub_l_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
  message_filters::Subscriber<DisparityImage> sub_disparity_;
  typedef ExactTime<Image, CameraInfo, CameraInfo, DisparityImage> ExactPolicy;
  typedef ApproximateTime<Image, CameraInfo, CameraInfo, DisparityImage> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_points2_;

  // Holds the coordinate system for which the point cloud will be generated
  CoordinateSystem target_coordinate_system_;

  virtual void onInit();

  void connectCb();

  void fillColor(
           const ImageConstPtr& l_image_msg,
           PointCloud2Ptr points_msg);

  void fillMetaData(const std_msgs::Header& header,
           const int height,
           const int width,
           PointCloud2Ptr points_msg);

  void unprojectAndFillPoints(
           const CameraInfoConstPtr& l_info_msg,
           const CameraInfoConstPtr& r_info_msg,
           const cv::Mat_<float>& dmat,
           PointCloud2Ptr& points_msg);

  void imageCb(const ImageConstPtr& l_image_msg,
               const CameraInfoConstPtr& l_info_msg,
               const CameraInfoConstPtr& r_info_msg,
               const DisparityImageConstPtr& disp_msg);
  const int STEP;
};

inline cv::Point3f UnprojectPoint(const cv::Mat_<float>& Q, const int x, const int y, const float disparity)
{
    const cv::Point3f XYZ(x + Q(0, 3), y + Q(1, 3), Q(2, 3));
    const float W = Q(3, 2) * disparity + Q(3, 3);
    return XYZ * (1.0 / W);
}

/* The formula is taken from from the Springer Handbook of Robotics, p. 524:
 * However, we only enter the values
 * that we actually use in the UnprojectPoint function.
 *
 * Q = [  1  0  0     -Cx;
 *        0  1  0     -Cy;
 *        0  0  0      Fx;
 *        0  0 -1/Tx   (Cx-Cx')/Tx]
 *
 * The code for copying the values into the matrix is taken from the
 * OpenCV StereoCameraModel::updateQ() function, see the OpenCV source code
 * image_geometry/src/stereo_camera_model.cpp, revision 29354.
 */
inline cv::Mat_<float> ComputeQ(const CameraInfoConstPtr& left_msg,
        const CameraInfoConstPtr& right_msg)
{
    image_geometry::PinholeCameraModel left;
    image_geometry::PinholeCameraModel right;
    left.fromCameraInfo(left_msg);
    right.fromCameraInfo(right_msg);
    cv::Mat_<float> Q(4, 4, 0.0);
    const float baseline = -right.Tx() / right.fx();
    Q(3, 2) = 1.0 / baseline;
    Q(0, 3) = -right.cx();
    Q(1, 3) = -right.cy();
    Q(2, 3) = right.fx();
    Q(3, 3) = (right.cx() - left.cx()) / baseline;
    return Q;
}

void PointCloud2Nodelet::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Synchronize inputs. Topic subscriptions happen on demand in the connection
  // callback. Optionally do approximate synchronization.
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  bool approx;
  private_nh.param("approximate_sync", approx, false);
  if (approx)
  {
    approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                                                 sub_l_image_, sub_l_info_,
                                                 sub_r_info_, sub_disparity_) );
    approximate_sync_->registerCallback(boost::bind(&PointCloud2Nodelet::imageCb,
                                                    this, _1, _2, _3, _4));
  }
  else
  {
    exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                                     sub_l_image_, sub_l_info_,
                                     sub_r_info_, sub_disparity_) );
    exact_sync_->registerCallback(boost::bind(&PointCloud2Nodelet::imageCb,
                                              this, _1, _2, _3, _4));
  }

  // Receive the target_coordinate_system parameter
  int coordinateSystemId;
  private_nh.param("target_coordinate_system", coordinateSystemId, toInt(CS_EAST_UP_SOUTH));
  target_coordinate_system_ = fromInt(coordinateSystemId);

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloud2Nodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_points2_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_points2_  = nh.advertise<PointCloud2>("points2",  1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void PointCloud2Nodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_points2_.getNumSubscribers() == 0)
  {
    sub_l_image_  .unsubscribe();
    sub_l_info_   .unsubscribe();
    sub_r_info_   .unsubscribe();
    sub_disparity_.unsubscribe();
  }
  else if (!sub_l_image_.getSubscriber())
  {
    ros::NodeHandle &nh = getNodeHandle();
    // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_l_image_  .subscribe(*it_, "left/image_rect_color", 1, hints);
    sub_l_info_   .subscribe(nh,   "left/camera_info", 1);
    sub_r_info_   .subscribe(nh,   "right/camera_info", 1);
    sub_disparity_.subscribe(nh,   "disparity", 1);
  }
}

void PointCloud2Nodelet::imageCb(const ImageConstPtr& l_image_msg,
                                 const CameraInfoConstPtr& l_info_msg,
                                 const CameraInfoConstPtr& r_info_msg,
                                 const DisparityImageConstPtr& disp_msg)
{
  // Calculate point cloud
  const Image& dimage = disp_msg->image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);

  PointCloud2Ptr points_msg = boost::make_shared<PointCloud2>();
  fillMetaData(l_image_msg->header, dmat.rows, dmat.cols, points_msg);

  #pragma omp parallel sections
  {
    #pragma omp section
    {
      unprojectAndFillPoints(l_info_msg, r_info_msg, dmat, points_msg);
    }
    #pragma omp section
    {
      fillColor(l_image_msg, points_msg);
    }
  }

  pub_points2_.publish(points_msg);
}

void PointCloud2Nodelet::unprojectAndFillPoints(
        const CameraInfoConstPtr& l_info_msg,
        const CameraInfoConstPtr& r_info_msg,
        const cv::Mat_<float>& dmat,
        PointCloud2Ptr& points_msg)
{
    const float BAD_POINT = std::numeric_limits<float>::quiet_NaN ();
    const cv::Mat_<float> Q = ComputeQ(l_info_msg, r_info_msg);
    int offset = 0;
    switch (target_coordinate_system_)
    {
      case CS_EAST_UP_SOUTH:
      {
        for (int v = 0; v < dmat.rows; ++v)
        {
          for (int u = 0; u < dmat.cols; ++u, offset += STEP)
          {
            const float disparity = dmat.at<float>(v, u);
            if (disparity <= 0)
            {
                memcpy (&points_msg->data[offset + 0], &BAD_POINT, sizeof (float));
                memcpy (&points_msg->data[offset + 4], &BAD_POINT, sizeof (float));
                memcpy (&points_msg->data[offset + 8], &BAD_POINT, sizeof (float));
            }
            else
            {
                const cv::Point3f vec = UnprojectPoint(Q, u, v, disparity);
                memcpy(&points_msg->data[offset + 0], &vec.x, sizeof(float));
                memcpy(&points_msg->data[offset + 4], &vec.y, sizeof(float));
                memcpy(&points_msg->data[offset + 8], &vec.z, sizeof(float));
            }
          }
        }
        break;
      }
      case CS_EAST_NORTH_UP:
      {
        for (int v = 0; v < dmat.rows; ++v)
        {
          for (int u = 0; u < dmat.cols; ++u, offset += STEP)
          {
              const float disparity = dmat.at<float>(v, u);
              if (disparity <= 0)
              {
                  memcpy (&points_msg->data[offset + 0], &BAD_POINT, sizeof (float));
                  memcpy (&points_msg->data[offset + 4], &BAD_POINT, sizeof (float));
                  memcpy (&points_msg->data[offset + 8], &BAD_POINT, sizeof (float));
              }
              else
              {
                  const cv::Point3f vec = UnprojectPoint(Q, u, v, disparity);
                  memcpy(&points_msg->data[offset + 0], &vec.x, sizeof(float));
                  memcpy(&points_msg->data[offset + 4], &vec.z, sizeof(float));
                  const float z = -vec.y;
                  memcpy(&points_msg->data[offset + 8], &z, sizeof(float));
              }
          }
        }
        break;
      }
      default:
      {
          NODELET_ERROR("Conversion to 3D points for given target coordinate system %d not implemented.", toInt(target_coordinate_system_));
      }
    }
}

void PointCloud2Nodelet::fillMetaData(const std_msgs::Header& header,
        const int height,
        const int width,
        PointCloud2Ptr points_msg)
{
    // Fill in metadata for PointCloud2 message (2D image-like layout)
    points_msg->header = header;
    points_msg->height = height;
    points_msg->width  = width;
    points_msg->fields.resize (4);
    points_msg->fields[0].name = "x";
    points_msg->fields[0].offset = 0;
    points_msg->fields[0].count = 1;
    points_msg->fields[0].datatype = PointField::FLOAT32;
    points_msg->fields[1].name = "y";
    points_msg->fields[1].offset = 4;
    points_msg->fields[1].count = 1;
    points_msg->fields[1].datatype = PointField::FLOAT32;
    points_msg->fields[2].name = "z";
    points_msg->fields[2].offset = 8;
    points_msg->fields[2].count = 1;
    points_msg->fields[2].datatype = PointField::FLOAT32;
    points_msg->fields[3].name = "rgb";
    points_msg->fields[3].offset = 12;
    points_msg->fields[3].count = 1;
    points_msg->fields[3].datatype = PointField::FLOAT32;
    //points_msg->is_bigendian = false; ???
    points_msg->point_step = STEP;
    points_msg->row_step = STEP * width;
    points_msg->data.resize (points_msg->row_step * height);
    points_msg->is_dense = false; // there may be invalid points

}

void PointCloud2Nodelet::fillColor(const ImageConstPtr& l_image_msg,
        PointCloud2Ptr points_msg)
{
    // Fill in color
    namespace enc = sensor_msgs::image_encodings;
    const std::string& encoding = l_image_msg->encoding;
    int offset = 0;
    const int rows = l_image_msg->height;
    const int cols = l_image_msg->width;

    if (encoding == enc::MONO8)
    {
      const cv::Mat_<uint8_t> color(rows, cols,
                                    (uint8_t*)&l_image_msg->data[0],
                                    l_image_msg->step);
      for (int v = 0; v < rows; ++v)
      {
        for (int u = 0; u < cols; ++u, offset += STEP)
        {
            uint8_t g = color(v,u);
            int32_t rgb = (g << 16) | (g << 8) | g;
            memcpy (&points_msg->data[offset + 12], &rgb, sizeof (int32_t));
        }
      }
    }
    else if (encoding == enc::RGB8)
    {
      const cv::Mat_<cv::Vec3b> color(l_image_msg->height, l_image_msg->width,
                                      (cv::Vec3b*)&l_image_msg->data[0],
                                      l_image_msg->step);
      for (int v = 0; v < rows; ++v)
      {
        for (int u = 0; u < cols; ++u, offset += STEP)
        {
            const cv::Vec3b& rgb = color(v,u);
            int32_t rgb_packed = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
            memcpy (&points_msg->data[offset + 12], &rgb_packed, sizeof (int32_t));
        }
      }
    }
    else if (encoding == enc::BGR8)
    {
      const cv::Mat_<cv::Vec3b> color(l_image_msg->height, l_image_msg->width,
                                      (cv::Vec3b*)&l_image_msg->data[0],
                                      l_image_msg->step);
      for (int v = 0; v < rows; ++v)
      {
        for (int u = 0; u < cols; ++u, offset += STEP)
        {
            const cv::Vec3b& bgr = color(v,u);
            int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
            memcpy (&points_msg->data[offset + 12], &rgb_packed, sizeof (int32_t));
        }
      }
    }
    else
    {
      NODELET_WARN_THROTTLE(30, "Could not fill color channel of the point cloud, "
                            "unsupported encoding '%s'", encoding.c_str());
    }
}

} // namespace stereo_image_proc_aurum

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(stereo_image_proc_aurum::PointCloud2Nodelet,nodelet::Nodelet)
