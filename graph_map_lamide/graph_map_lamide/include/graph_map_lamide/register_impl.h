

namespace perception_oru
{
namespace graph_map
{

template <class PointT>
bool Registration::scanmatching(pcl::PointCloud<PointT>& reference,
                                pcl::PointCloud<PointT>& source,
                                Eigen::Affine3d& Tsrc,
                                Eigen::MatrixXd& reg_cov,
                                const Eigen::Affine3d& Ttarget)
{

    pcl::PointCloud<pcl::PointXYZL> target_local, src;
    Eigen::Affine3d Toffset = Ttarget.inverse(); // Eigen::Affine3d::Identity();
                                                 // //Ttarget.inverse();
    ConvertAndTransform(reference, target_local, Toffset); // change to local frame
    pcl::copyPointCloud(source, src);

    Eigen::Affine3d Tinit = Toffset * Tsrc; // change to local frame
    bool success = registrator_->RegisterScan(target_local, Tinit, src, reg_cov);
    score = registrator_->GetScore();
    cout << "score: " << score << endl;
    bool fuse_this_frame = KeyFrameBasedFuse(Tinit); // fuse frame based on distance traveled

    if (success)
        Tsrc = Toffset.inverse() * Tinit; // change back to global frame
    return success && fuse_this_frame;    //&& fuse_this_frame;
}

template <class PointT>
bool Registration::scanmatching(
    std::vector<std::pair<Eigen::Affine3d, pcl::PointCloud<PointT>>>& reference,
    pcl::PointCloud<PointT>& source,
    Eigen::Affine3d& Tsrc,
    Eigen::MatrixXd& reg_cov)
{
    if (reference.empty())
        return true;
    pcl::PointCloud<PointT> target;
    Aggregate(reference, target);
    return scanmatching(target, source, Tsrc, reg_cov, reference.back().first);
}

template <class PointT>
void Registration::ConvertAndTransform(pcl::PointCloud<PointT>& cld_in,
                                       pcl::PointCloud<pcl::PointXYZL>& cld_out,
                                       Eigen::Affine3d& Toffset)
{
    pcl::copyPointCloud(cld_in, cld_out);
    transformPointCloudInPlace(Toffset, cld_out);
}

template <class PointT>
void Registration::Aggregate(
    std::vector<std::pair<Eigen::Affine3d, pcl::PointCloud<PointT>>>& reference,
    pcl::PointCloud<PointT>& aggregated_ref)
{
    for (auto r : reference)
        aggregated_ref += r.second;
}

} // namespace graph_map
} // namespace perception_oru
