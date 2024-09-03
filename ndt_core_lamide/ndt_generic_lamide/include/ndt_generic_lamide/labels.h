#pragma once

#include <vector>
#include <map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ██╗      █████╗ ██████╗ ███████╗██╗     ███████╗
// ██║     ██╔══██╗██╔══██╗██╔════╝██║     ██╔════╝
// ██║     ███████║██████╔╝█████╗  ██║     ███████╗
// ██║     ██╔══██║██╔══██╗██╔══╝  ██║     ╚════██║
// ███████╗██║  ██║██████╔╝███████╗███████╗███████║
// ╚══════╝╚═╝  ╚═╝╚═════╝ ╚══════╝╚══════╝╚══════╝


namespace ndt_generic
{

const std::map<int, std::vector<int>> color_map_ = {
    {0, {0, 0, 0}},        {1, {0, 0, 255}},      {10, {245, 150, 100}},  {11, {245, 230, 100}},
    {13, {250, 80, 100}},  {15, {150, 60, 30}},   {16, {255, 0, 0}},      {18, {180, 30, 80}},
    {20, {255, 0, 0}},     {30, {30, 30, 255}},   {31, {200, 40, 255}},   {32, {90, 30, 150}},
    {40, {255, 0, 255}},   {44, {255, 150, 255}}, {48, {75, 0, 75}},      {49, {75, 0, 175}},
    {50, {0, 200, 255}},   {51, {50, 120, 255}},  {52, {0, 150, 255}},    {60, {170, 255, 150}},
    {70, {0, 175, 0}},     {71, {0, 60, 135}},    {72, {80, 240, 150}},   {80, {150, 240, 255}},
    {81, {0, 0, 255}},     {99, {255, 255, 50}},  {252, {245, 150, 100}}, {256, {255, 0, 0}},
    {253, {200, 40, 255}}, {254, {30, 30, 255}},  {255, {90, 30, 150}},   {257, {250, 80, 100}},
    {258, {180, 30, 80}},  {259, {255, 0, 0}}};
const std::vector<int> all_labels_ = {0,  1,  10,  11,  13,  15,  16,  18,  20,  30, 31, 32,
                                      40, 44, 48,  49,  50,  51,  52,  60,  70,  71, 72, 80,
                                      81, 99, 252, 253, 254, 255, 256, 257, 258, 259};
const std::vector<int> static_labels_ = {40, 44, 48, 49, 50, 51, 52, 60, 70, 71, 72, 80, 81, 99};
const std::vector<int> semistatic_labels_ = {
    10, 11, 13, 15, 16, 18, 20, 30, 31, 32,
};
const std::vector<int> dynamic_labels_ = {252, 253, 254, 255, 256, 257, 258, 259};
const std::vector<int> nonstatic_labels_ = {0,  1,  10,  11,  13,  15,  16,  18,  20,  30,
                                            31, 32, 252, 253, 254, 255, 256, 257, 258, 259};
const std::vector<int> unknown_labels_ = {0, 1};

bool isStatic(int label);

bool isDynamic(int label);

bool isSemiStatic(int label);

void filterNonStaticPoints(pcl::PointCloud<pcl::PointXYZL>& cloud);

void filterDynamicPoints(pcl::PointCloud<pcl::PointXYZL>& cloud);

} // namespace ndt_generic