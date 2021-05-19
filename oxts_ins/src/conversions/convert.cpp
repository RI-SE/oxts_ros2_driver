#include "oxts_ins/convert.hpp"


namespace oxts_ins
{

void OxtsIns::NCom_callback_regular(const oxts_msgs::msg::Ncom::SharedPtr msg)
{
  // Add data to decoder
  if (NComNewChars(this->nrx, msg->raw_packet.data(), NCOM_PACKET_LENGTH) == COM_NEW_UPDATE)
  {
    double current_time = rclcpp::Time(msg->header.stamp).seconds();
    int sec_idx = round((current_time - floor(current_time)) * this->ncom_rate);
    
    // Publish IMU message if being subscribed to and enabled in config
    if (this->pub_imu_flag) 
      this->imu(msg->header);
    if (this->pub_tf_flag)
      this->tf(msg->header);
    if (this->pubStringInterval && (sec_idx % this->pubStringInterval == 0))
      this->string();
    if (this->pubNavSatRefInterval && (sec_idx % this->pubNavSatRefInterval == 0))
      this->nav_sat_ref(msg->header);
    if (this->pubEcefPosInterval && (sec_idx % this->pubEcefPosInterval == 0))
      this->ecef_pos(msg->header);
    if (this->pubNavSatFixInterval && (sec_idx % this->pubNavSatFixInterval == 0))
      this->nav_sat_fix(msg->header);
    if (this->pubVelocityInterval && (sec_idx % this->pubVelocityInterval == 0))
      this->velocity(msg->header);
    if (this->pubOdometryInterval && (sec_idx % this->pubOdometryInterval == 0))
      this->odometry(msg->header);
    if (this->pubTimeReferenceInterval && (sec_idx % this->pubTimeReferenceInterval == 0))
      this->time_reference(msg->header);
  }
}

void OxtsIns::string()
{
  auto msgString = RosNComWrapper::string(this->nrx);
  pubString_->publish(msgString);

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", 
                                  msgString.data.c_str());
}

void OxtsIns::nav_sat_fix(std_msgs::msg::Header header)
{
  header.frame_id = "navsat_link";
  auto msg    = RosNComWrapper::nav_sat_fix(this->nrx, header);
  pubNavSatFix_->publish(msg);
}

void OxtsIns::nav_sat_ref(std_msgs::msg::Header header)
{
  header.frame_id = "navsat_link";
  auto msg    = RosNComWrapper::nav_sat_ref(this->nrx, header);
  pubNavSatRef_->publish(msg);
}

void OxtsIns::ecef_pos(std_msgs::msg::Header header)
{
  header.frame_id = "oxts_link";
  auto msg    = RosNComWrapper::ecef_pos(this->nrx, header);
  pubEcefPos_->publish(msg);
}

void OxtsIns::imu(std_msgs::msg::Header header)
{
  header.frame_id = "imu_link";
  auto msg    = RosNComWrapper::imu(this->nrx, header);
  pubImu_->publish(msg);
}

void OxtsIns::tf(std_msgs::msg::Header header)
{
  // Set the LRF if - we haven't set it before
  if (!this->lrf_valid)
  {
    this->getLrf();
  } 
  if (this->lrf_valid)
  {
    auto odometry = RosNComWrapper::odometry(this->nrx, header, this->lrf);
    geometry_msgs::msg::TransformStamped tf_oxts;
    tf_oxts.header = header;
    tf_oxts.header.frame_id = this->pub_odometry_frame_id;
    tf_oxts.child_frame_id = "oxts_link";
    tf_oxts.transform.translation.x = odometry.pose.pose.position.x;
    tf_oxts.transform.translation.y = odometry.pose.pose.position.y;
    tf_oxts.transform.translation.z = odometry.pose.pose.position.z;
    tf_oxts.transform.rotation.x = odometry.pose.pose.orientation.x;
    tf_oxts.transform.rotation.y = odometry.pose.pose.orientation.y;
    tf_oxts.transform.rotation.z = odometry.pose.pose.orientation.z;
    tf_oxts.transform.rotation.w = odometry.pose.pose.orientation.w;
    tf_broadcaster_->sendTransform(tf_oxts);

    auto vat    = RosNComWrapper::getVat(this->nrx);
    auto nsp    = RosNComWrapper::getNsp(this->nrx);

    if (this->nrx->mIsNoSlipLeverArmXValid)
    {
      geometry_msgs::msg::TransformStamped tf_vat;
      tf_vat.header = header;
      tf_vat.header.frame_id = "oxts_link";
      tf_vat.child_frame_id = "rear_axle_link";
      tf_vat.transform.translation.x = nsp.x();
      tf_vat.transform.translation.y = nsp.y();
      tf_vat.transform.translation.z = nsp.z();
      tf_vat.transform.rotation.x = vat.x();
      tf_vat.transform.rotation.y = vat.y();
      tf_vat.transform.rotation.z = vat.z();
      tf_vat.transform.rotation.w = vat.w();
      tf_broadcaster_->sendTransform(tf_vat);
      
      /** \todo Make this real */
      if (true) // if vertical slip lever arm is valid
      {
        // auto nvsp    = RosNComWrapper::getNvsp(this->nrx);
        // evil spoof pls fix
        auto nvsp    = nsp;
        nvsp += tf2::quatRotate(vat, tf2::Vector3(2.6, 0, 0));

        geometry_msgs::msg::TransformStamped tf_front_axle;
        tf_front_axle.header = header;
        tf_front_axle.header.frame_id = "oxts_link";
        tf_front_axle.child_frame_id = "front_axle_link";
        tf_front_axle.transform.translation.x = nvsp.x();
        tf_front_axle.transform.translation.y = nvsp.y();
        tf_front_axle.transform.translation.z = nvsp.z();
        tf_front_axle.transform.rotation.x = vat.x();
        tf_front_axle.transform.rotation.y = vat.y();
        tf_front_axle.transform.rotation.z = vat.z();
        tf_front_axle.transform.rotation.w = vat.w();
        tf_broadcaster_->sendTransform(tf_front_axle);
      }
    }
  }
  
}

void OxtsIns::velocity(std_msgs::msg::Header header)
{
  header.frame_id = "oxts_link";
  auto msg    = RosNComWrapper::velocity(this->nrx, header);
  pubVelocity_->publish(msg);
}

void OxtsIns::odometry(std_msgs::msg::Header header)
{
  header.frame_id = this->pub_odometry_frame_id;
  // Set the LRF if - we haven't set it before
  if (!this->lrf_valid)
  {
    this->getLrf();
  } 
  if (this->lrf_valid)
  {
    auto msg    = RosNComWrapper::odometry(this->nrx, header, this->lrf);
    pubOdometry_->publish(msg);
  }
}

void OxtsIns::time_reference(std_msgs::msg::Header header)
{
  header.frame_id = "oxts_link";
  auto msg    = RosNComWrapper::time_reference(this->nrx, header);
  pubTimeReference_->publish(msg);
}

void OxtsIns::getLrf()
{
  // Configured to come from NCom LRF, and the NCom LRF is valid.
  if (this->lrf_source == LRF_SOURCE::NCOM_LRF && nrx->mIsRefLatValid)
    {
      this->lrf = RosNComWrapper::getNcomLrf(nrx); 
      this->lrf_valid = true;
    }
  // Configured to come from the first NCom packet
  else if (this->lrf_source == LRF_SOURCE::NCOM_FIRST)
  {
    this->lrf.origin(nrx->mLat,nrx->mLon,nrx->mAlt);
    // mHeading is in NED. Get angle between LRF and ENU
    this->lrf.heading((90.0+nrx->mHeading) * NAV_CONST::DEG2RADS); 
    this->lrf_valid = true;
  }  
  else if (this->lrf_source == LRF_SOURCE::NCOM_FIRST_ENU)
  {
    this->lrf.origin(nrx->mLat,nrx->mLon,nrx->mAlt);
    this->lrf.heading((0.0) * NAV_CONST::DEG2RADS); // LRF aligned to ENU
    this->lrf_valid = true;
  }  
}

} // namespace oxts_ins