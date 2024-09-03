#pragma once
#include <limits>
#include <string>

namespace perception_oru
{

struct ComparisonResult
{
    int other_missing_count_ = 0;
    int this_missing_count_ = 0;
    int both_missing_count_ = 0;
    int other_no_distribution_count_ = 0;
    int this_no_distribution_count_ = 0;
    int both_no_distribution_count_ = 0;
    int match_count_ = 0;
    double total_distance_ = 0;
    double min_distance_ = std::numeric_limits<double>::max();
    double max_distance_ = std::numeric_limits<double>::min();
    double avg_distance_ = 0;

    void print() const;
    std::string serialize() const;
};

} // namespace perception_oru