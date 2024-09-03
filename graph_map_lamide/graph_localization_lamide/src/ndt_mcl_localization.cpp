// perception_oru
#include <graph_localization_lamide/ndt_mcl_localization.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_mcl");
    ros::NodeHandle parameters("~");
    localization_node pf(parameters);
}

//
//
//
//
//
// ███████╗██████╗  █████╗ ███╗   ███╗███████╗
// ██╔════╝██╔══██╗██╔══██╗████╗ ████║██╔════╝
// █████╗  ██████╔╝███████║██╔████╔██║█████╗
// ██╔══╝  ██╔══██╗██╔══██║██║╚██╔╝██║██╔══╝
// ██║     ██║  ██║██║  ██║██║ ╚═╝ ██║███████╗
// ╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚══════╝

void localization_node::processFrame(pcl::PointCloud<pcl::PointXYZL>& cloud, const ros::Time& ts)
{
    if (!initLocalization(ts))
    {
        return;
    }

    filterPointsBySemantics(cloud);

    // Get motion
    Eigen::Affine3d Todom;
    bool success = false;
    Eigen::Affine3d Tmotion = getMotion(ts, Todom, success);
    if (!success)
    {
        return;
    }

    // note: update
    transformPointCloudInPlace(Tsens, cloud);
    localisation_type_ptr_->UpdateAndPredict(cloud, Tmotion, Tsens);
    Eigen::Affine3d pose = localisation_type_ptr_->GetPose();

    // get error
    ros::Time gt_time;
    Eigen::Affine3d gt = getGT(ts, gt_time);
    double error, ATE, RPE, mean_rpe;
    Eigen::Vector3d error_vec = getError(pose, gt, error, ATE, RPE, mean_rpe);

    // check if lost
    checkLost(error);

    // comparison for lifelong localization
    if (compare_)
    {
        compare(pose);
    }

    // logging, printing, visualization
    printDebug(Tmotion, pose, gt, error_vec, error, ATE, RPE, mean_rpe);
    publish(ts, pose, cloud, Todom);
    log(pose, gt, gt_time, ts, error_vec, ATE, RPE, mean_rpe);
    visualize(pose, cloud);
    histogramming(cloud);

    // save state
    last_gt_ = gt;
    last_pose_ = pose;
    got_last_ = true;

    cloud.clear();
    frame_count_++;

    if (request_scans_ && request_got_next_)
    {
        got_frame_ = true;
        requestScan();
    }
}

// ███████╗██████╗  █████╗ ███╗   ███╗███████╗
// ██╔════╝██╔══██╗██╔══██╗████╗ ████║██╔════╝
// █████╗  ██████╔╝███████║██╔████╔██║█████╗
// ██╔══╝  ██╔══██╗██╔══██║██║╚██╔╝██║██╔══╝
// ██║     ██║  ██║██║  ██║██║ ╚═╝ ██║███████╗
// ╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚══════╝

// ███████╗██╗   ██╗███╗   ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
// ██╔════╝██║   ██║████╗  ██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
// █████╗  ██║   ██║██╔██╗ ██║██║        ██║   ██║██║   ██║██╔██╗ ██║███████╗
// ██╔══╝  ██║   ██║██║╚██╗██║██║        ██║   ██║██║   ██║██║╚██╗██║╚════██║
// ██║     ╚██████╔╝██║ ╚████║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║███████║
// ╚═╝      ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝

//
//
//
//
//
//------------------------------------------------------------------------------
// MOTION
//------------------------------------------------------------------------------

Eigen::Affine3d localization_node::getMotion(const ros::Time& ts,
                                             Eigen::Affine3d& Todom,
                                             bool& success)
{
    static tf::TransformListener tf_listener;

    if (odom_type_ == "tf")
    {
        tf::StampedTransform transform;
        // double x, y, yaw;

        // note: duration from 0.1 to 1
        tf_listener.waitForTransform(odomTF, baseTF, ts, ros::Duration(1));
        try
        {
            // note: this seems the be the same than in fuser
            tf_listener.lookupTransform(odomTF, baseTF, ts, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            success = false;
            return Eigen::Affine3d::Identity();
        }
        tf::poseTFToEigen(transform, Todom);
    }
    else if (odom_type_ == "ins")
    {
        if (got_odom_)
        {
            Todom = latest_odom_;
        }
        else
        {
            Todom = Eigen::Affine3d::Identity();
        }
    }

    Eigen::Affine3d Tmotion;
    if (relative_movement_)
    {
        Tmotion = Todom;
    }
    else
    {
        if (got_odom_)
        {
            if (firstLoad_)
            {
                Tmotion = localisation_type_ptr_->GetPose().inverse() * Todom;
                // HACK: first odom is jumpy!
                Tmotion = Tmotion.prescale(0.01);
            }
            else
            {
                Tmotion = tOld.inverse() * Todom;
            }
            if (firstLoad_)
            {
                if (Todom.translation().norm() > 0)
                {
                    std::cout << "got initial non-zero odometry" << std::endl;
                    tOld = Todom;
                    firstLoad_ = false;
                }
            }
            else
            {
                tOld = Todom;
            }
        }
        else
        {
            Tmotion = Eigen::Affine3d::Identity();
        }
    }

    // HACK: motion jump prevention
    if (Tmotion.translation().norm() > 10)
    {
        double x = Tmotion.translation()(0);
        double y = Tmotion.translation()(1);
        double z = Tmotion.translation()(2);
        double largest = std::max({x, y, z});
        Tmotion = Tmotion.prescale(1.0 / (2.0 * largest));
        std::cout << "LARGE JUMP WARNING!" << std::endl;
    }

    // note: restrict to 2d solution
    if (restrict_to_2d_solution_)
    {
        Eigen::Vector3d eul = Tmotion.rotation().matrix().eulerAngles(0, 1, 2);
        normalizeEulerAngles(eul);
        std::cout << "motion t: " << Tmotion.translation().transpose() << std::endl;
        std::cout << "motion r: " << eul.transpose() << std::endl;
        Eigen::AngleAxisd rollAngle(eul.x(), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(eul.y(), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(0, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        Eigen::Matrix3d rollpitch = q.matrix();
        Eigen::Matrix3d yaw = rollpitch.inverse() * Tmotion.rotation().matrix();

        Tmotion.translation().z() = 0;

        Eigen::Affine3d modified = Eigen::Affine3d::Identity();
        modified.rotate(yaw).translate(Tmotion.translation());
        Tmotion = modified;
        eul = Tmotion.rotation().matrix().eulerAngles(0, 1, 2);
        std::cout << "modified t: " << Tmotion.translation().transpose() << std::endl;
        std::cout << "modified r: " << eul.transpose() << std::endl;
    }

    success = true;
    return Tmotion;
}

//
//
//
//
//
//------------------------------------------------------------------------------
// FILTER
//------------------------------------------------------------------------------

void localization_node::filterPointsBySemantics(pcl::PointCloud<pcl::PointXYZL>& cloud) const
{
    // note only static mode
    if (only_static_)
    {
        ndt_generic::filterNonStaticPoints(cloud);
    }
    else if (filter_dynamic_)
    {
        ndt_generic::filterDynamicPoints(cloud);
    }
}

//
//
//
//
//
//------------------------------------------------------------------------------
// INIT
//------------------------------------------------------------------------------

bool localization_node::initLocalization(const ros::Time& ts)
{
    if (!initialized_ && !initial_pose_set)
    {
        std::cout << "not initialized" << std::endl;
        return false;
    }
    else if (!initialized_ && initial_pose_set)
    {
        geometry_msgs::Pose pose_init_geom;
        tf::poseEigenToMsg(initial_pose, pose_init_geom);
        std::cout << "initialize from process frame" << std::endl;
        Initialize(pose_init_geom, ts);
        ROS_INFO_STREAM("Initialized");
    }
    return true;
}

//
//
//
//
//
//------------------------------------------------------------------------------
// PRINT DEBUG
//------------------------------------------------------------------------------

void localization_node::printDebug(const Eigen::Affine3d& Tmotion,
                                   const Eigen::Affine3d& pose,
                                   const Eigen::Affine3d& gt,
                                   const Eigen::Vector3d& error_vec,
                                   double error,
                                   double ATE,
                                   double RPE,
                                   double mean_rpe)
{
    std::stringstream ss;
    ss << std::endl;
    ss << "Frame " << frame_count_ << std::endl;
    ss << "**********************************************************" << std::endl;
    ss << std::fixed << std::setprecision(3);
    ss << "Estimated motion: ";
    ss << Tmotion.translation().x() << " ";
    ss << Tmotion.translation().y() << " ";
    ss << Tmotion.translation().z() << " ";
    ss << std::endl;
    ss << std::fixed << std::setprecision(3);
    ss << "  Localized pose: ";
    ss << pose.translation().x() << " ";
    ss << pose.translation().y() << " ";
    ss << pose.translation().z() << " ";
    ss << std::endl;
    ss << "                : ";
    Eigen::Vector3d pEul = pose.rotation().matrix().eulerAngles(0, 1, 2);
    ss << pEul.x() << " ";
    ss << pEul.y() << " ";
    ss << pEul.z() << " ";
    ss << std::endl;
    if (got_gt_)
    {
        ss << "     Groud truth: ";
        ss << gt.translation().x() << " ";
        ss << gt.translation().y() << " ";
        ss << gt.translation().z() << " ";
        ss << std::endl;
        ss << "           Error: ";
        ss << error_vec.x() << " ";
        ss << error_vec.y() << " ";
        ss << error_vec.z() << " ";
        ss << std::endl;
        ss << "      Error norm: " << error << std::endl;

        ss << std::endl;
        ss << "             ATE: " << ATE << std::endl;

        if (got_last_)
        {
            ss << "             RPE: " << sqrt(RPE) << std::endl;
            ss << "        RMSE RPE: " << mean_rpe << std::endl;
        }
    }
    last_state_print_ = ss.str();
    if (verbose_)
    {
        std::cout << last_state_print_;
        std::cout << std::endl;
    }

    ss.clear();
}

//
//
//
//
//
//------------------------------------------------------------------------------
// LOST CHECKING
//------------------------------------------------------------------------------

void localization_node::checkLost(double error)
{
    std::stringstream ss;
    // localization lost detection
    if (error > lost_threshold_ && lost_threshold_ > 0)
    {
        lost_count_++;
        ss << "Localization lost: " << lost_count_ << " / " << max_lost_ << std::endl;
        if (lost_count_ >= max_lost_)
        {
            ss << "LOCALIZATION LOST!!!!" << std::endl;
            ss << "frame: " << frame_count_ << std::endl;
            ss << "error: " << error << std::endl;
            ss << "threshold: " << lost_threshold_ << std::endl;
            lost_print_ = ss.str();
            std::cout << lost_print_;
            exitRoutine();
        }
    }
    else
    {
        lost_count_ = 0;
    }
}

//
//
//
//
//
//------------------------------------------------------------------------------
// ERROR
//------------------------------------------------------------------------------

Eigen::Vector3d localization_node::getError(const Eigen::Affine3d& pose,
                                            const Eigen::Affine3d& gt,
                                            double& error,
                                            double& ATE,
                                            double& RPE,
                                            double& mean_rpe)
{
    Eigen::Vector3d error_vec = (pose.translation() - gt.translation());
    if (error_dim_ == 2)
    {
        error_vec.z() = 0;
    }

    error = error_vec.norm();

    error_sum_ += error * error;
    ATE = sqrt(error_sum_ / (double)frame_count_);

    if (got_last_)
    {
        Eigen::Affine3d te;
        if (error_dim_ == 2)
        {
            last_pose_.translation().z() = 0;
            Eigen::Affine3d zeroz_pose = pose;
            zeroz_pose.translation().z() = 0;
            te = (last_gt_.inverse() * gt).inverse() * (last_pose_.inverse() * zeroz_pose);
        }
        else
        {
            te = (last_gt_.inverse() * gt).inverse() * (last_pose_.inverse() * pose);
        }

        RPE = te.translation().transpose() * te.translation();
        rpe_sum_ += RPE;
        mean_rpe = sqrt(rpe_sum_ / (double)frame_count_);
    }

    return error_vec;
}

//
//
//
//
//
//------------------------------------------------------------------------------
// GT
//------------------------------------------------------------------------------

Eigen::Affine3d localization_node::getGT(const ros::Time& ts, ros::Time& gt_time)
{
    int index = -1;
    double tss = ts.toSec();
    double minDt = -1;
    std::deque<nav_msgs::Odometry>::iterator it;
    for (it = gt_buffer_.begin(); it != gt_buffer_.end(); ++it)
    {
        double timestamp = it->header.stamp.toSec();
        double dt = fabs(tss - timestamp);
        if (minDt == -1 || minDt > dt)
        {
            minDt = dt;
            index = it - gt_buffer_.begin();
            gt_time = it->header.stamp;
        }
    }
    nav_msgs::Odometry minGt = gt_buffer_[index];
    Eigen::Affine3d gt;
    tf::poseMsgToEigen(minGt.pose.pose, gt);

    return gt;
}

//
//
//
//
//
//------------------------------------------------------------------------------
// PUBLISH
//------------------------------------------------------------------------------

void localization_node::publish(const ros::Time& ts,
                                const Eigen::Affine3d& pose,
                                pcl::PointCloud<pcl::PointXYZL>& cloud,
                                const Eigen::Affine3d& Todom)
{
    // rootTF -> OdomTF -> baseTF    Tcorr: rootTF ->OdomTF is unknown before localization, The
    // transformation is defined such that Trobot: rootTF -> baseTF is the robot pose in the
    // world, Tcorr "corrects" the parent to the odoemtry frame such taht Trobot is the robot in
    // the root
    // Todom: OdomTF ->baseTF is given by the odometry estimates
    // Trobot=Tcorr*Todom <=> Trobot*inv(Todom)
    nav_msgs::Odometry mean_pose_odom;
    mean_pose_odom.header.frame_id = rootTF;
    tf::poseEigenToMsg(pose, mean_pose_odom.pose.pose);
    mean_pose_odom.header.stamp = ts;
    mclPosePub.publish(mean_pose_odom);

    // try: publish tf as well
    tf::Transform base;
    tf::poseEigenToTF(pose, base);
    trans_pub.sendTransform(tf::StampedTransform(base, ts, rootTF, baseTF));
    // end

    Eigen::Affine3d Tcorr = pose * Todom.inverse();
    tf::poseEigenToTF(Tcorr, Tcorr_tf);
    trans_pub.sendTransform(tf::StampedTransform(Tcorr_tf, ts, rootTF, odomTF));

    if (publish_sensor_link)
    {
        tf::poseEigenToTF(Tsens, sensor_tf);
        trans_pub.sendTransform(tf::StampedTransform(sensor_tf, ts, baseTF, laser_link_id));
    }

    cloud.header.frame_id = rootTF;
    pcl_conversions::toPCL(ts, cloud.header.stamp);
    Eigen::Affine3d Tloc = pose * Tsens;

    transformPointCloud(Tloc, cloud);
    transformPointCloudInPlace(Tloc, cloud);
}

//
//
//
//
//
//------------------------------------------------------------------------------
// LOG
//------------------------------------------------------------------------------

void localization_node::log(const Eigen::Affine3d& pose,
                            const Eigen::Affine3d& gt,
                            const ros::Time& gt_time,
                            const ros::Time& ts,
                            const Eigen::Vector3d& error_vec,
                            double ATE,
                            double RPE,
                            double mean_rpe) const
{
    static std::string isodate = getISOdate();
    static std::string logPath = map_path + log_prefix_ + "_" + hostname_ + "_pose_" +
                                 localisation_type_name + "_" + isodate + ".log";
    logPose(logPath, pose, ts);

    static std::string logPath_gt = map_path + log_prefix_ + "_" + hostname_ + "_gt_" +
                                    localisation_type_name + "_" + isodate + ".log";
    logPose(logPath_gt, gt, gt_time);

    MCLNDTTypePtr ndt_mcl =
        MCLNDTTypePtr(boost::dynamic_pointer_cast<MCLNDTType>(localisation_type_ptr_));
    Eigen::Matrix3d cov = ndt_mcl->getPFCovariance();

    static std::string logPath_dbg = map_path + log_prefix_ + "_" + hostname_ + "_dbg_" +
                                     localisation_type_name + "_" + isodate + ".log";
    logDebug(logPath_dbg, ts, error_vec, ATE, RPE, mean_rpe, cov);
}

//
//
//
//
//
//------------------------------------------------------------------------------
// HISTOGRAMS
//------------------------------------------------------------------------------

void localization_node::histogramming(const pcl::PointCloud<pcl::PointXYZL>& cloud)
{
    if (match_histogram_)
    {
        std::vector<std::string> string_vector = localisation_type_ptr_->getHistogramStrings();
        histogram_data_.insert(histogram_data_.end(), string_vector.begin(), string_vector.end());
    }

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = now - last_input_time_;
    double sec_count = diff.count();
    rateMeasurements_.push_back(sec_count);
    if (rateMeasurements_.size() > 100)
    {
        rateMeasurements_.pop_front();
    }
    double sum = 0;
    unsigned int rm_size = rateMeasurements_.size();
    for (unsigned int i = 0; i < rm_size; i++)
    {
        sum += 1.0 / rateMeasurements_[i];
    }
    double processRate = 1.0 / ((double)rm_size) * sum;
    estimated_rate_ = processRate * 1.0 / 20.0;

    last_input_time_ = std::chrono::steady_clock::now();
    if (calculate_histogram_ && !match_histogram_)
    {
        if (localisation_type_name == "ndt_mcl" or localisation_type_name == "ndt_mcl_lamide")
        {
            static int hcnt = 0;
            int node_id = graph_map_->GetCurrentNode()->GetId();
            histogram_buffer_.push(HistogramObject(cloud, Tsens, node_id));
            std::cout << getISOdate() << " M >> Histogram buffer size: " << histogram_buffer_.size()
                      << std::endl;

            hcnt++;

            input_time_set_ = true;
        }
    }
}

//
//
//
//
//
//------------------------------------------------------------------------------
// VISUALIZE
//------------------------------------------------------------------------------

void localization_node::visualize(Eigen::Affine3d& pose, pcl::PointCloud<pcl::PointXYZL>& cloud)
{
    if (!doVisualization_)
    {
        return;
    }
    if (!visualizationInitialized_)
    {
        vis_ = new graphVisualization(graph_map_, false, visualization_colors_);
        vis_->SetParentFrameId(rootTF);
        if (simple_vis)
        {
            vis_->SetMarkerType(plotmarker::point);
        }
        visualizationInitialized_ = true;
    }

    bool draw = false;
    int node_id = graph_map_->GetCurrentNode()->GetId();
    if (firstDraw_)
    {
        draw = true;
        firstDraw_ = false;
    }
    else
    {
        draw = node_id != prev_node_id_;
    }
    prev_node_id_ = node_id;

    if (isLifelong_ || draw || true)
    {
        vis_->PlotLinks(0, ros::Time::now());
        vis_->PlotCurrentLinkInfo(ros::Time::now());
        vis_->PlotTrajectory(ros::Time::now());
        vis_->PlotEstGtTrajectory(ros::Time::now());
        vis_->PlotCurrentMapTrajectory(ros::Time::now());
        vis_->PlotCurrentMap(plotmarker::sphere, ros::Time::now());
    }
}

//
//
//
//
//
//------------------------------------------------------------------------------
// GET SCANS
//------------------------------------------------------------------------------

void localization_node::requestScan()
{
    std_srvs::Trigger service;
    ros::service::call(scan_service_, service);
    request_got_next_ = service.response.success;
}

//
//
//
//
//
// ██╗███╗   ██╗██╗████████╗██╗ █████╗ ██╗     ██╗███████╗███████╗
// ██║████╗  ██║██║╚══██╔══╝██║██╔══██╗██║     ██║╚══███╔╝██╔════╝
// ██║██╔██╗ ██║██║   ██║   ██║███████║██║     ██║  ███╔╝ █████╗
// ██║██║╚██╗██║██║   ██║   ██║██╔══██║██║     ██║ ███╔╝  ██╔══╝
// ██║██║ ╚████║██║   ██║   ██║██║  ██║███████╗██║███████╗███████╗
// ╚═╝╚═╝  ╚═══╝╚═╝   ╚═╝   ╚═╝╚═╝  ╚═╝╚══════╝╚═╝╚══════╝╚══════╝

void localization_node::Initialize(const Eigen::Affine3d& pose_init)
{
    geometry_msgs::Pose pose_init_geom;
    tf::poseEigenToMsg(pose_init, pose_init_geom);
    Initialize(pose_init_geom, ros::Time::now());
}

void localization_node::initialposeCallback(geometry_msgs::PoseWithCovarianceStamped input_init)
{
    cout << "Initialize: initialposeCallback" << endl;
    Initialize(input_init.pose.pose, input_init.header.stamp);
    initialized_ = true;
    firstLoad_ = true;
}

void localization_node::Initialize(const geometry_msgs::Pose& pose_init, const ros::Time& t_init)
{
    Vector6d certain;
    certain << init_range_x_, init_range_y_, init_range_z_, init_range_rr_, init_range_rp_,
        init_range_ry_;
    Vector6d uncertain = certain;

    Eigen::Affine3d pose_init_eig;
    tf::poseMsgToEigen(pose_init, pose_init_eig);
    Eigen::Vector3d eul = pose_init_eig.rotation().matrix().eulerAngles(0, 1, 2);
    cout << "Initial position set to " << pose_init_eig.translation().transpose() << endl;
    cout << "Initial rotation set to " << eul.transpose() << endl;

    bool init = false;
    if (localisation_type_name == "ndt_mcl" or localisation_type_name == "ndt_mcl_lamide")
    {
        MCLNDTTypePtr ndt_mcl =
            MCLNDTTypePtr(boost::dynamic_pointer_cast<MCLNDTType>(localisation_type_ptr_));
        if (ndt_mcl != NULL)
        {
            Vector6d var;
            // we actually have a good guess
            if (initialized_)
            {
                var = certain;
            }
            // we can have a good guess if robot pose is saved and accurate
            else
            {
                var = uncertain;
            }
            if (init_particles_ > 0)
            {
                ndt_mcl->InitializeLocalization(pose_init_eig, var, true, init_particles_);
            }
            else
            {
                ndt_mcl->InitializeLocalization(pose_init_eig, var, true);
            }
            init = true;
        }
    }

    // note histogram setter
    localisation_type_ptr_->setCreateHistogram(match_histogram_, histogram_ratio_);
    localisation_type_ptr_->setDisplayErrorMap(display_error_map_, display_local_map_);
    localisation_type_ptr_->setDistributionError(distribution_error_);
    std::cout << "set histogram creation: " << match_histogram_ << std::endl;

    std::cout << "set map swtiching method" << std::endl;
    MapSwitchingMethod swm = GraphMapNavigatorParam::String2SwitchMethod(sw_method_);
    localisation_type_ptr_->SetMapSwitchMethod(swm);
    localisation_type_ptr_->SetMapInterchangeRadius(interchange_radius_);

    if (not init)
    {
        localisation_type_ptr_->InitializeLocalization(pose_init_eig, certain);
    }
    initialized_ = true;
}

void localization_node::InitializeUniform(const geometry_msgs::Pose& pose_init,
                                          const ros::Time& t_init)
{
    initialized_ = true;
}

//
//
//
//
//
//  ██████╗ █████╗ ██╗     ██╗     ██████╗  █████╗  ██████╗██╗  ██╗███████╗
// ██╔════╝██╔══██╗██║     ██║     ██╔══██╗██╔══██╗██╔════╝██║ ██╔╝██╔════╝
// ██║     ███████║██║     ██║     ██████╔╝███████║██║     █████╔╝ ███████╗
// ██║     ██╔══██║██║     ██║     ██╔══██╗██╔══██║██║     ██╔═██╗ ╚════██║
// ╚██████╗██║  ██║███████╗███████╗██████╔╝██║  ██║╚██████╗██║  ██╗███████║
//  ╚═════╝╚═╝  ╚═╝╚══════╝╚══════╝╚═════╝ ╚═╝  ╚═╝ ╚═════╝╚═╝  ╚═╝╚══════╝

void localization_node::VeloCallback(const velodyne_msgs::VelodyneScan::ConstPtr& msg)
{
    velodyne_msgs::VelodyneScan::ConstPtr scan = msg;
    std::cerr << "NOT recomended callback" << std::endl;
    pcl::PointCloud<pcl::PointXYZL> cloud;
    ndt_generic::UnwarpCloudSimple(dataParser, scan, cloud);
    this->processFrame(cloud, msg->header.stamp);
}

void localization_node::PCCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZL> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);
    this->processFrame(pcl_cloud, msg->header.stamp);
}

void localization_node::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_geometry::LaserProjection projector_;
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZL> pcl_cloud_unfiltered, pcl_cloud;
    projector_.projectLaser(*msg, cloud);
    pcl::fromROSMsg(cloud, pcl_cloud_unfiltered);

    pcl::PointXYZL pt;
    // add some variance on z
    for (int i = 0; i < pcl_cloud_unfiltered.points.size(); i++)
    {
        pt = pcl_cloud_unfiltered.points[i];
        if (sqrt(pt.x * pt.x + pt.y * pt.y) > min_range)
        {
            pt.z += 0.1 * ((double)rand()) / (double)INT_MAX;
            pcl_cloud.points.push_back(pt);
        }
    }
    this->processFrame(pcl_cloud, msg->header.stamp);
}

bool localization_node::exit_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    exitRoutine();
    return true;
}

//
//
//
//
//
//  ██████╗████████╗
// ██╔════╝╚══██╔══╝
// ██║  ███╗  ██║
// ██║   ██║  ██║
// ╚██████╔╝  ██║
//  ╚═════╝   ╚═╝

void localization_node::GTCallback(const nav_msgs::Odometry::ConstPtr& msg_in)
{
    if (!initialized_)
    {
        cout << "initialize: gt callback" << endl;
        Initialize(msg_in->pose.pose, msg_in->header.stamp);
        initialized_ = true;
        firstLoad_ = true;
    }

    static std::string isodate = getISOdate();
    static std::string logPath =
        map_path + log_prefix_ + "_gt_" + localisation_type_name + "_" + isodate + ".log";
    ros::Time ts = msg_in->header.stamp;
    Eigen::Affine3d gt;
    tf::poseMsgToEigen(msg_in->pose.pose, gt);

    got_gt_ = true;

    if (gt_buffer_.size() > gt_buffer_max_size_)
    {
        gt_buffer_.pop_front();
    }
    gt_buffer_.push_back(*msg_in);
}

//
//
//
//
//
//  ██████╗ ██████╗  ██████╗ ███╗   ███╗
// ██╔═══██╗██╔══██╗██╔═══██╗████╗ ████║
// ██║   ██║██║  ██║██║   ██║██╔████╔██║
// ██║   ██║██║  ██║██║   ██║██║╚██╔╝██║
// ╚██████╔╝██████╔╝╚██████╔╝██║ ╚═╝ ██║
//  ╚═════╝ ╚═════╝  ╚═════╝ ╚═╝     ╚═╝

void localization_node::odomCallback(const nav_msgs::Odometry::ConstPtr& msg_in)
{
    Eigen::Affine3d odom = Eigen::Affine3d::Identity();
    if (odom_twist_)
    {
        // FIXME: Time difference
        geometry_msgs::Vector3 t = msg_in->twist.twist.linear;
        odom.translation() = Eigen::Vector3d(t.x, t.y, t.z);
    }
    else
    {
        tf::poseMsgToEigen(msg_in->pose.pose, odom);
    }
    latest_odom_ = odom;
    got_odom_ = true;
}

//
//
//
//
//
// ██╗   ██╗████████╗██╗██╗     ███████╗
// ██║   ██║╚══██╔══╝██║██║     ██╔════╝
// ██║   ██║   ██║   ██║██║     ███████╗
// ██║   ██║   ██║   ██║██║     ╚════██║
// ╚██████╔╝   ██║   ██║███████╗███████║
//  ╚═════╝    ╚═╝   ╚═╝╚══════╝╚══════╝

bool localization_node::logPose(std::string& filename,
                                const Eigen::Affine3d& Ts,
                                const ros::Time& ts) const
{
    std::stringstream ss;
    ss << std::setw(9) << std::setfill('0') << ts.nsec;
    std::string ns = ss.str();

    std::stringstream out;
    out << ts.sec << ns << " ";
    out << Ts.translation()(0) << " ";
    out << Ts.translation()(1) << " ";
    out << Ts.translation()(2) << " ";
    out << Ts.rotation().eulerAngles(0, 1, 2)(0) << " ";
    out << Ts.rotation().eulerAngles(0, 1, 2)(1) << " ";
    out << Ts.rotation().eulerAngles(0, 1, 2)(2) << " ";

    return logData(filename, out.str());
}

bool localization_node::logDebug(std::string& filename,
                                 const ros::Time& ts,
                                 const Eigen::Vector3d& e,
                                 const double& ate,
                                 const double& rpe,
                                 const double& rpe_mean,
                                 const Eigen::Matrix3d& cov) const
{
    std::stringstream ss;
    ss << std::setw(9) << std::setfill('0') << ts.nsec;
    std::string ns = ss.str();

    std::stringstream out;
    out << ts.sec << ns << " ";
    out << e.x() << " " << e.y() << " " << e.z() << " ";
    out << ate << " " << rpe << " " << rpe_mean << " ";
    for (unsigned int i = 0; i < 3; i++)
    {
        for (unsigned int j = 0; j < 3; j++)
        {
            out << cov(i, j) << " ";
        }
    }
    return logData(filename, out.str());
}

bool localization_node::logData(std::string& filename, const std::string& data) const
{
    std::ofstream ofs;
    ofs.open(filename.c_str(), std::ofstream::app);
    if (!ofs.is_open())
    {
        return false;
    }
    ofs << data;
    ofs << std::endl;
    ofs.close();
    return true;
}

bool localization_node::LoadAffineToFile(const std::string& fileName, Eigen::Affine3d& ret)
{
    try
    {
        std::fstream myfile(fileName.c_str(), std::ios_base::in);
        if (myfile.is_open())
        {
            double x, y, z, qx, qy, qz, qw;
            myfile >> x >> y >> z >> qx >> qy >> qz >> qw;
            ret = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(qw, qx, qy, qz);
            myfile.close();
            return true;
        }
    }
    catch (std::exception& e)
    {
    }
    return false;
}

void localization_node::Pose2DToTF(Eigen::Vector3d mean, ros::Time ts, Eigen::Affine3d Todometry)
{
    static tf::TransformBroadcaster br, br_mapOdom;
    tf::Transform transform;
    tf::Quaternion q;
    q.setRPY(0, 0, mean[2]);
    transform.setOrigin(tf::Vector3(mean[0], mean[1], 0.0));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ts, rootTF, mclTF));
}

int localization_node::LoadMap()
{
    LoadGraphMap(map_path, map_file, graph_map_);
    if (graph_map_ == NULL)
    {
        std::cerr << "ERROR LOADING NDT MAP FROM FILE" << std::endl;
        exit(0);
    }
}

std::string localization_node::getISOdate() const
{
    time_t now;
    time(&now);
    char buf[sizeof "2011-10-08T07:07:09Z"];
    strftime(buf, sizeof buf, "%FT%TZ", localtime(&now));
    return std::string(buf);
}

//
//
//
//
//
// ███████╗██╗  ██╗██╗████████╗
// ██╔════╝╚██╗██╔╝██║╚══██╔══╝
// █████╗   ╚███╔╝ ██║   ██║
// ██╔══╝   ██╔██╗ ██║   ██║
// ███████╗██╔╝ ██╗██║   ██║
// ╚══════╝╚═╝  ╚═╝╚═╝   ╚═╝

void localization_node::exitData()
{
    std::cout << "print exit data..." << std::endl;
    // Print summary
    std::ofstream ofs;
    ofs.open(exitPath_.c_str(), std::ofstream::app);
    if (!ofs.is_open())
    {
        std::cout << "failed to open file " << exitPath_ << std::endl;
        return;
    }
    ofs << last_state_print_ << std::endl;
    ofs << lost_print_ << std::endl;
    ofs.close();

    // print params
    std::cout << "print params data..." << std::endl;
    ofs.open(exitMetadataPath_.c_str(), std::ofstream::app);
    if (!ofs.is_open())
    {
        std::cout << "failed to open file " << exitMetadataPath_ << std::endl;
        return;
    }
    ofs << localisation_type_ptr_->ToString() << std::endl;
    ofs << "estimated rate: " << estimated_rate_ << std::endl;
    ofs << "processed rate: " << estimated_rate_ * 20.0 << std::endl;
    ofs.close();
    std::cout << "done!" << std::endl;
}

void localization_node::exitRoutine()
{
    std::cout << "exit routine" << std::endl;
    exitData();
    std::cout << "shutdown" << std::endl;
    ros::shutdown();
}

//
//
//
//
//
// ██╗    ██╗██████╗ ██╗████████╗███████╗██████╗ ███████╗
// ██║    ██║██╔══██╗██║╚══██╔══╝██╔════╝██╔══██╗██╔════╝
// ██║ █╗ ██║██████╔╝██║   ██║   █████╗  ██████╔╝███████╗
// ██║███╗██║██╔══██╗██║   ██║   ██╔══╝  ██╔══██╗╚════██║
// ╚███╔███╔╝██║  ██║██║   ██║   ███████╗██║  ██║███████║
//  ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚══════╝

void localization_node::histWriter()
{
    std::cout << getISOdate() << " H >> Start histWriter" << std::endl;
    while (ros::ok())
    {
        HistogramObject data;
        bool run = false;
        int size = 0;
        {
            size = histogram_buffer_.size();
            if (size > 0)
            {
                data = histogram_buffer_.front();
                histogram_buffer_.pop();
                run = true;
            }
        }
        if (run)
        {
            MCLNDTTypePtr ndt_mcl =
                MCLNDTTypePtr(boost::dynamic_pointer_cast<MCLNDTType>(localisation_type_ptr_));

            std::vector<std::string> string_vector;
            if (!match_histogram_)
            {
                string_vector = ndt_mcl->logPointHistogram(histogramPath_, data.cloud_, data.pose_,
                                                           data.node_id_);
            }

            int strSize = 0;
            {
                histogram_data_.insert(histogram_data_.end(), string_vector.begin(),
                                       string_vector.end());
                strSize = histogram_data_.size();
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    std::cout << getISOdate() << " H >> End histWriter!" << std::endl;
}

void localization_node::fileWriter()
{
    std::cout << getISOdate() << " F >> Start fileWriter" << std::endl;
    while (ros::ok())
    {
        bool run = false;
        std::vector<std::string> local_strings;
        int dataSize = 0;
        {
            dataSize = histogram_data_.size();
            if (dataSize > 0)
            {
                local_strings.insert(local_strings.end(), histogram_data_.begin(),
                                     histogram_data_.end());
                histogram_data_.clear();
                run = true;
            }
        }
        if (run)
        {
            int writeSize = local_strings.size();
            // initialize log
            std::ofstream ofs;
            ofs.open(histogramPath_.c_str(), std::ofstream::app);
            if (!ofs.is_open())
                return;
            for (std::string row : local_strings)
            {
                ofs << row;
            }
            ofs.close();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    std::cout << getISOdate() << " F >> End fileWriter!" << std::endl;
}

void localization_node::inputWatchdog()
{
    while (ros::ok())
    {
        if (input_time_set_)
        {
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> diff = now - last_input_time_;
            double sec_count = diff.count();
            if (sec_count > 5)
            {
                input_timeout_ = true;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void localization_node::requestStarter()
{
    while(ros::ok() and not got_frame_)
    {
        requestScan();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

    //
    //
    //
    //
    //
    //  ██████╗ ██████╗ ███╗   ███╗██████╗  █████╗ ██████╗ ██╗███████╗ ██████╗ ███╗   ██╗
    // ██╔════╝██╔═══██╗████╗ ████║██╔══██╗██╔══██╗██╔══██╗██║██╔════╝██╔═══██╗████╗  ██║
    // ██║     ██║   ██║██╔████╔██║██████╔╝███████║██████╔╝██║███████╗██║   ██║██╔██╗ ██║
    // ██║     ██║   ██║██║╚██╔╝██║██╔═══╝ ██╔══██║██╔══██╗██║╚════██║██║   ██║██║╚██╗██║
    // ╚██████╗╚██████╔╝██║ ╚═╝ ██║██║     ██║  ██║██║  ██║██║███████║╚██████╔╝██║ ╚████║
    //  ╚═════╝ ╚═════╝ ╚═╝     ╚═╝╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝╚══════╝ ╚═════╝ ╚═╝  ╚═══╝

    void localization_node::parseTransforms()
{
    std::ifstream file;
    file.open(transforms_file_);
    std::string line;
    const std::string delimiter = ", ";
    if (file.is_open())
    {
        while (getline(file, line))
        {
            std::string orig = line;
            size_t pos = 0;
            std::vector<std::string> tokens;
            while ((pos = line.find(delimiter)) != std::string::npos)
            {
                std::string token = line.substr(0, pos);
                tokens.push_back(token);
                line.erase(0, pos + delimiter.length());
            }
            tokens.push_back(line);
            Eigen::Affine3d tf;
            for (int r = 0; r < 4; r++)
            {
                for (int c = 0; c < 4; c++)
                {
                    std::string str = tokens[r * 4 + c];
                    double val = std::stod(str);
                    tf(r, c) = val;
                }
            }
            tfs_.push_back(tf);
        }
        file.close();
    }
}

void localization_node::compare(const Eigen::Affine3d& pose)
{
    // First (primary) map
    MapTypePtr primary = graph_map_->GetCurrentMapNode();
    MapNodePtr node = graph_map_->GetCurrentNode();
    int node_id = graph_map_->GetNodeId(node);

    // Secondary map
    // FIXME parametrize max map change distance
    comp_map_->SwitchToMapNode(comp_map_->GetNode(node_id));
    MapTypePtr secondary = comp_map_->GetCurrentMapNode();

    if (node_id >= tfs_.size())
    {
        std::cout << "warning! not enough tfs!" << std::endl;
        std::cout << "node id: " << node_id << " # tfs " << tfs_.size() << std::endl;
        return;
    }
    Eigen::Affine3d& tf = tfs_[node_id];

    // Compare
    // a
    NDTMapPtr current_map = boost::dynamic_pointer_cast<NDTMapType>(primary);
    perception_oru::NDTMap* a = current_map->GetNDTMap();

    // b
    NDTMapPtr other_map = boost::dynamic_pointer_cast<NDTMapType>(secondary);
    perception_oru::NDTMap* b = other_map->GetNDTMap();

    if (a and b)
    {
        ComparisonResult res = a->compare(b, tf, false);

        static std::string isodate = getISOdate();
        static std::string logPath = map_path + log_prefix_ + "_" + hostname_ + "_comparison_" +
                                     localisation_type_name + "_" + isodate + ".log";

        std::string print = std::to_string(node_id) + ", " + res.serialize();
        logData(logPath, print);
    }
}

//
//
//
//
//
//  ██████╗████████╗ ██████╗ ██████╗
// ██╔════╝╚══██╔══╝██╔═══██╗██╔══██╗
// ██║        ██║   ██║   ██║██████╔╝
// ██║        ██║   ██║   ██║██╔══██╗
// ╚██████╗   ██║   ╚██████╔╝██║  ██║
//  ╚═════╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝

localization_node::localization_node(ros::NodeHandle param)
{
    param.param<std::string>("map_file", map_file, "");
    param.param<std::string>("map_path", map_path, "");
    param.param<bool>("visualize", doVisualization_, false);
    param.param<int>("visualization_colors", visualization_colors_, 1);

    param.param<std::string>("input_cloud_topic", input_cloud_topic, "/velodyne_packets");
    param.param<std::string>("gt_topic", gt_topic, "");
    param.param<std::string>("pose_estimate_topic", pose_estimate_topic, "/ndt_mcl_pose");
    param.param<bool>("initPoseFromGT", init_pose_gt, true);
    param.param<bool>("initPoseFile", init_pose_file, false);
    param.param<bool>("Velodyne", beVelodyne, false);
    param.param<bool>("PointCloud", bePC, true);
    param.param<bool>("Laser_2d", laser_2d, false);

    param.param<std::string>("pose_init_path", pose_init_path, "init_pose.txt");
    param.param<std::string>("log_prefix", log_prefix_, "");
    param.param<std::string>("hostname", hostname_, "");
    param.param<std::string>("world_tf", worldTF, "/world");
    param.param<std::string>("root_tf", rootTF, "/map");
    param.param<std::string>("odom_tf", odomTF, "/odom");
    param.param<std::string>("base_tf", baseTF, "/base_link");
    param.param<std::string>("mcl_tf", mclTF, "/mcl");
    param.param<std::string>("laser_tf", laser_link_id, "/velodyne");
    param.param<bool>("publish_sensor_link", publish_sensor_link, false);
    param.param<bool>("simple_visualization", simple_vis, "false");
    param.param<bool>("send_map_frame", send_map_frame, "false");

    Eigen::Vector3d sensor_offset_pos, sensor_offset_euler;
    param.param("sensor_pose_x", sensor_offset_pos(0), 0.);
    param.param("sensor_pose_y", sensor_offset_pos(1), 0.);
    param.param("sensor_pose_z", sensor_offset_pos(2), 0.);
    param.param("sensor_pose_r", sensor_offset_euler(0), 0.);
    param.param("sensor_pose_p", sensor_offset_euler(1), 0.);
    param.param("sensor_pose_t", sensor_offset_euler(2), 0.);
    Tsens = ndt_generic::vectorsToAffine3d(sensor_offset_pos, sensor_offset_euler);

    Eigen::Vector3d init_pos = Eigen::Vector3d::Identity();
    Eigen::Vector3d init_euler = Eigen::Vector3d::Identity();
    param.param<double>("pose_init_x", init_pos(0), 0.0);
    param.param<double>("pose_init_y", init_pos(1), 0.0);
    param.param<double>("pose_init_tLas", init_euler(2), 0.0);
    param.param<double>("init_var", initVar, 0.5);

    param.param<double>("var_x", var_x, 0.07);
    param.param<double>("var_y", var_y, 0.07);
    param.param<double>("var_th", var_th, 0.035);
    param.param<string>("initial_pose_topic", initial_pose_topic, "initialpose");

    param.param<double>("r_var_x", r_var_x, 1.0);
    param.param<double>("r_var_y", r_var_y, 1.0);
    param.param<double>("r_var_th", r_var_th, 0.001);
    param.param<std::string>("dataset", dataset, "orkla-velodyne");

    param.param<double>("mm_tx1", mm_tx1_, 0.0);
    param.param<double>("mm_tx2", mm_tx2_, 0.0);
    param.param<double>("mm_ty1", mm_ty1_, 0.0);
    param.param<double>("mm_ty2", mm_ty2_, 0.0);
    param.param<double>("mm_tz1", mm_tz1_, 0.0);
    param.param<double>("mm_tz2", mm_tz2_, 0.0);
    param.param<double>("mm_rr1", mm_rr1_, 0.0);
    param.param<double>("mm_rr2", mm_rr2_, 0.0);
    param.param<double>("mm_rp1", mm_rp1_, 0.0);
    param.param<double>("mm_rp2", mm_rp2_, 0.0);
    param.param<double>("mm_ry1", mm_ry1_, 0.0);
    param.param<double>("mm_ry2", mm_ry2_, 0.0);
    param.param<double>("mm_off_tx", mm_off_tx_, 0.0);
    param.param<double>("mm_off_ty", mm_off_ty_, 0.0);
    param.param<double>("mm_off_tz", mm_off_tz_, 0.0);
    param.param<double>("mm_off_rr", mm_off_rr_, 0.0);
    param.param<double>("mm_off_rp", mm_off_rp_, 0.0);
    param.param<double>("mm_off_ry", mm_off_ry_, 0.0);

    param.param<double>("init_range_x", init_range_x_, 20.0);
    param.param<double>("init_range_y", init_range_y_, 20.0);
    param.param<double>("init_range_z", init_range_z_, 0.0);
    param.param<double>("init_range_rr", init_range_rr_, 0.0);
    param.param<double>("init_range_rp", init_range_rp_, 0.0);
    param.param<double>("init_range_ry", init_range_ry_, 2 * M_PI);
    param.param<int>("init_particles", init_particles_, 5000);

    param.param("min_range", min_range, 1.5);
    param.param("max_range", max_range, 100.0);

    param.param<bool>("relative_movement", relative_movement_, false);

    param.param<bool>("map_unloading", map_unloading_, false);
    param.param<bool>("only_static", only_static_, false);
    param.param<bool>("filter_dynamic", filter_dynamic_, false);

    // note the localization type
    param.param<std::string>("localisation_type_name", localisation_type_name, "ndt_mcl");
    dataParser.setup(param);
    dataParser.setParameters(min_range, max_range, 0, 2 * 3.1415);

    param.param<bool>("display_local_map", display_local_map_, false);
    param.param<bool>("display_error_map", display_error_map_, true);
    param.param<bool>("calculate_histogram", calculate_histogram_, true);
    param.param<bool>("match_histogram", match_histogram_, true);
    param.param<int>("histogram_ratio", histogram_ratio_, -1);
    param.param<double>("lost_threshold", lost_threshold_, -1);
    param.param<int>("max_lost", max_lost_, -1);
    param.param<int>("error_dim", error_dim_, 3);
    param.param<bool>("distribution_error", distribution_error_, false);
    param.param<bool>("odom_twist", odom_twist_, false);
    std::string odom_topic = "";
    param.param<std::string>("odom_topic", odom_topic, "");
    param.param<std::string>("odom_type", odom_type_, "");

    param.param<std::string>("map_switching_method", sw_method_, "");
    param.param<double>("interchange_radius", interchange_radius_, 30);
    param.param<bool>("restrict_to_2d_solution", restrict_to_2d_solution_, false);
    param.param<bool>("verbose", verbose_, true);

    param.param<bool>("compare", compare_, false);
    param.param<std::string>("comp_map_file", comp_map_file_, "");
    param.param<std::string>("comp_map_path", comp_map_path_, "");
    param.param<std::string>("transforms_file", transforms_file_, "");

    param.param<bool>("request_scans", request_scans_, false);
    param.param<std::string>("scan_service", scan_service_, "/oxford_publisher/publish");
    double presleep;
    param.param<double>("presleep", presleep, 5.0);

    if (compare_)
    {
        LoadGraphMap(comp_map_path_, comp_map_file_, comp_map_);
    }
    parseTransforms();

    LoadMap();
    graph_map_->setMapUnloading(map_unloading_);
    if (map_unloading_)
    {
        graph_map_->unloadAllSubmaps();
    }

    // set map parameters
    MapParamPtr mapParams = GraphFactory::CreateMapParam("ndt_map");
    mapParams->GetParametersFromRos();
    graph_map_->setMapParams(mapParams);

    LocalizationType::fix_frame_id = rootTF;

    LocalisationParamPtr loc_ptr =
        LocalisationFactory::CreateLocalisationParam(localisation_type_name);

    loc_ptr->GetParamFromRos();
    loc_ptr->sensor_pose = Tsens;
    loc_ptr->graph_map_ = graph_map_;

    if (MCLNDTParamPtr parPtr = boost::dynamic_pointer_cast<MCLNDTParam>(loc_ptr))
    {
        cout << "Read motion model for MCL" << endl;
        if (dataset == "custom")
        {
            MotionModel3d motionModel;
            MotionModel3d::Params par;
            par.set3DParams(mm_tx1_, mm_tx2_, mm_ty1_, mm_ty2_, mm_tz1_, mm_tz2_, mm_rr1_, mm_rr2_,
                            mm_rp1_, mm_rp2_, mm_ry1_, mm_ry2_);
            Eigen::Matrix<double, 6, 1> offset;
            offset << mm_off_tx_, mm_off_ty_, mm_off_tz_, mm_off_rr_, mm_off_rp_, mm_off_ry_;
            par.SetOffset(offset);
            motionModel.setParams(par);
            parPtr->motion_model = motionModel;
        }
        else
        {
            GetMotionModel(dataset, parPtr->motion_model);
        }
    }

    cout << "----------------Localisation parameters------------------\n"
         << loc_ptr->ToString() << endl;
    localisation_type_ptr_ = LocalisationFactory::CreateLocalisationType(loc_ptr);
    std::cout << localisation_type_ptr_->ToString() << std::endl;
    cout << "---------------------------------------------------------" << endl;

    MCLNDTParamPtr parPtr = boost::dynamic_pointer_cast<MCLNDTParam>(loc_ptr);
    isLifelong_ = parPtr->lifelong;

    if (isLifelong_)
    {
        std::cout << "                                                                 "
                  << std::endl;
        std::cout << "                                                                 "
                  << std::endl;
        std::cout << " ██╗     ██╗███████╗███████╗██╗      ██████╗ ███╗   ██╗ ██████╗  "
                  << std::endl;
        std::cout << " ██║     ██║██╔════╝██╔════╝██║     ██╔═══██╗████╗  ██║██╔════╝  "
                  << std::endl;
        std::cout << " ██║     ██║█████╗  █████╗  ██║     ██║   ██║██╔██╗ ██║██║  ███╗ "
                  << std::endl;
        std::cout << " ██║     ██║██╔══╝  ██╔══╝  ██║     ██║   ██║██║╚██╗██║██║   ██║ "
                  << std::endl;
        std::cout << " ███████╗██║██║     ███████╗███████╗╚██████╔╝██║ ╚████║╚██████╔╝ "
                  << std::endl;
        std::cout << " ╚══════╝╚═╝╚═╝     ╚══════╝╚══════╝ ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝  "
                  << std::endl;
        std::cout << "                                                                 "
                  << std::endl;
        std::cout << "                                                                 "
                  << std::endl;
    }

    firstLoad_ = true;
    initialized_ = false;

    if (beVelodyne)
    {
        PCSub = nh.subscribe(input_cloud_topic, 1, &localization_node::VeloCallback, this);
        cout << "Listen to sensor_msgs::velodyne_msgs::VelodyneScan at topic \""
             << input_cloud_topic << "\"" << endl;
    }
    else if (bePC)
    {
        PCSub = nh.subscribe(input_cloud_topic, 1, &localization_node::PCCallback, this);
        cout << "Listen to sensor_msgs::PointCloud2 at topic \"" << input_cloud_topic << "\""
             << endl;
    }
    else if (laser_2d)
    {
        PCSub = nh.subscribe(input_cloud_topic, 1, &localization_node::LaserCallback, this);
        cout << "Listen to 2d-laser at topic \"" << input_cloud_topic << "\"" << endl;
    }
    else
        std::cerr << "No lidar Type sepected for topic \"" << input_cloud_topic << "\""
                  << std::endl;

    initPoseSub =
        nh.subscribe(initial_pose_topic, 1000, &localization_node::initialposeCallback, this);

    initPoseSub = nh.subscribe(odom_topic, 1000, &localization_node::odomCallback, this);

    if (init_pose_gt)
    {
        std::cout << "initialize pose with gt" << std::endl;
        cout << "Listen to GT at topic :" << gt_topic << endl;
        gtPoseSub =
            nh.subscribe<nav_msgs::Odometry>(gt_topic, 10, &localization_node::GTCallback, this);
    }
    else if (init_pose_file)
    {
        bool status = LoadAffineToFile(pose_init_path, initial_pose);
        cout << std::fixed << std::setprecision(5)
             << "Load pose :" << initial_pose.translation().transpose() << "\n from  file "
             << pose_init_path << endl;
        if (status)
            cout << "Init pose from file: " << initial_pose.translation().transpose() << endl;
        else if (!status)
        {
            initial_pose = ndt_generic::vectorsToAffine3d(init_pos, init_euler);
            cout << "Initial pose set to :" << initial_pose.translation().transpose() << endl;
        }
        initial_pose_set = true;
    }
    else
    {
        initial_pose = ndt_generic::vectorsToAffine3d(init_pos, init_euler);
        cout << "Initial pose set to :" << initial_pose.translation().transpose() << endl;
        initial_pose_set = true;
    }

    mclPosePub = nh.advertise<nav_msgs::Odometry>(pose_estimate_topic, 20);
    cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZL>>("localized_cloud", 20);

    exit_server_ = nh.advertiseService("loc_exit", &localization_node::exit_callback, this);

    std::string isodate = getISOdate();
    exitPath_ = map_path + log_prefix_ + "_" + hostname_ + "_log_" + localisation_type_name + "_" +
                isodate + ".summary";
    exitMetadataPath_ = map_path + log_prefix_ + "_" + hostname_ + "_params_" +
                        localisation_type_name + "_" + isodate + ".meta";

    // histograms
    int numThreads = 5;

    if (calculate_histogram_)
    {
        static std::string hisodate = getISOdate();
        if (match_histogram_)
        {
            histogramPath_ = map_path + log_prefix_ + "_match_histogram_" + localisation_type_name +
                             "_" + hisodate + ".log";
        }
        else
        {
            histogramPath_ = map_path + log_prefix_ + "_histogram_" + localisation_type_name + "_" +
                             hisodate + ".log";
        }
        if (!match_histogram_)
        {
            for (int i = 0; i < numThreads; i++)
            {
                std::thread thread = std::thread(&localization_node::histWriter, this);
                thread.detach();
            }
        }
        std::thread fileWriter(&localization_node::fileWriter, this);
        std::thread dog(&localization_node::inputWatchdog, this);
        fileWriter.detach();
        dog.detach();
    }

    std::thread dog(&localization_node::inputWatchdog, this);

    if (request_scans_)
    {
        long ms = 1000 * presleep;
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        std::thread req(&localization_node::requestStarter, this);
        req.detach();
    }

    ros::spin();
} // ctor