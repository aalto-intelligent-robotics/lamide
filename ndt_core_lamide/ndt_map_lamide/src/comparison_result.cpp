#include <ndt_map_lamide/comparison_result.h>
#include <iostream>
#include <sstream>

namespace perception_oru
{

void ComparisonResult::print() const
{
    std::cout << std::endl;
    std::cout << "Map comparison complete" << std::endl;
    std::cout << "--------------------------------------------------------------" << std::endl;
    std::cout << "other_missing_count:         " << other_missing_count_ << std::endl;
    std::cout << "this_missing_count:          " << this_missing_count_ << std::endl;
    std::cout << "both_missing_count:          " << both_missing_count_ << std::endl;
    std::cout << "other_no_distribution_count: " << other_no_distribution_count_ << std::endl;
    std::cout << "this_no_distribution_count:  " << this_no_distribution_count_ << std::endl;
    std::cout << "both_no_distribution_count:  " << both_no_distribution_count_ << std::endl;
    std::cout << "match_count:                 " << match_count_ << std::endl;
    std::cout << "total_distance:              " << total_distance_ << std::endl;
    std::cout << "min_distance:                " << min_distance_ << std::endl;
    std::cout << "max_distance:                " << max_distance_ << std::endl;
    std::cout << "avg_distance:                " << avg_distance_ << std::endl;
    std::cout << "--------------------------------------------------------------" << std::endl;
    std::cout << std::endl;
}

std::string ComparisonResult::serialize() const
{
    std::stringstream ss;
    ss << other_missing_count_ << ", ";
    ss << this_missing_count_ << ", ";
    ss << both_missing_count_ << ", ";
    ss << other_no_distribution_count_ << ", ";
    ss << this_no_distribution_count_ << ", ";
    ss << both_no_distribution_count_ << ", ";
    ss << match_count_ << ", ";
    ss << total_distance_ << ", ";
    ss << min_distance_ << ", ";
    ss << max_distance_ << ", ";
    ss << avg_distance_;
    return ss.str();
}

}