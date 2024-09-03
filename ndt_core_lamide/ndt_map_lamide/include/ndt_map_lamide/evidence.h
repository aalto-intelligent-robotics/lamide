#pragma once

#include <ndt_map_lamide/ndt_cell.h>
#include <vector>
#include <map>

// ███████╗██╗   ██╗██╗██████╗ ███████╗███╗   ██╗ ██████╗███████╗
// ██╔════╝██║   ██║██║██╔══██╗██╔════╝████╗  ██║██╔════╝██╔════╝
// █████╗  ██║   ██║██║██║  ██║█████╗  ██╔██╗ ██║██║     █████╗
// ██╔══╝  ╚██╗ ██╔╝██║██║  ██║██╔══╝  ██║╚██╗██║██║     ██╔══╝
// ███████╗ ╚████╔╝ ██║██████╔╝███████╗██║ ╚████║╚██████╗███████╗
// ╚══════╝  ╚═══╝  ╚═╝╚═════╝ ╚══════╝╚═╝  ╚═══╝ ╚═════╝╚══════╝

namespace perception_oru
{

class Evidence
{
public:
    Evidence();
    Evidence(NDTCell* cell, const double evidence, const int numPoints);
    NDTCell* cell_;
    int clusterId_;
    double evidence_;
    double logOddsLikelihood_;
    double totalLikelihood_;
    int numPoints_;
};

class EvidenceContainer
{
public:
    EvidenceContainer();
    void add(NDTCell* cell, const double evidence, const int numPoints);
    const bool hasBeenUpdated(const NDTCell* cell) const;
    const Evidence& get(const NDTCell* cell, bool& found) const;
    const std::vector<int> getIndices(const int id) const;
    const unsigned int size() const;
    const Evidence& operator[](const int index) const;

public:
    std::vector<int> allIndices_;
private:
    std::vector<Evidence> evidence_;
    std::vector<int> indices_;
};

} // namespace perception_oru