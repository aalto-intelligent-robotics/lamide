#pragma once

#include <algorithm>
#include <cmath>
#include <limits.h>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/time.h>
#include <time.h>
#include <vector>
#include <chrono>
#include <map>
#include <iostream>
namespace ndt_generic
{

void getVectorMeanStdev(const std::vector<double>& v, double& mean, double& stdev);

void getVectorMinMax(const std::vector<double>& v, double& min, double& max);

void getVectorQuartiles(const std::vector<double>& vec, double& q1, double& median, double& q3);

void normalizeVector(std::vector<double>& v);

template <class T> std::string toString(const T& x)
{
    std::ostringstream o;

    if (!(o << x))
        throw std::runtime_error("::toString()");

    return o.str();
}

template <class T> T fromString(const std::string& s)
{
    T t;
    std::istringstream iss(s);
    iss >> t;
    return t;
}

std::string getVectorStatisticStr(const std::vector<double>& data);

std::string getVectorStatisticGnuplotStr(const std::vector<double>& data);

double getDoubleTime();

const std::string currentDateTimeString();

std::vector<std::string> splitLine(const std::string& line, const std::string& delimiter);

std::string getISOdate();

class NullBuffer : public std::streambuf
{
public:
    int overflow(int c)
    {
        return c;
    }
};

class NullStream : public std::ostream
{
public:
    NullStream()
        : std::ostream(&m_sb)
    {
    }

private:
    NullBuffer m_sb;
};

class PrintInhibitor
{
public:
    PrintInhibitor()
    {
        ndt_generic::NullStream ns;
        old_ = std::cout.rdbuf();
        std::cout.rdbuf(ns.rdbuf());
    };
    ~PrintInhibitor()
    {
        std::cout.rdbuf(old_);
    };

private:
    std::streambuf* old_;
};

class Stopwatch
{
public:
    Stopwatch(const std::string& name = "default", bool inloop = false);
    void start();
    void stop();
    void lap(const std::string& name = "");
    void print();
    void loopStart();

private:
    void normalLap(const std::chrono::steady_clock::time_point l, const std::string& name);
    void loopLap(const std::chrono::steady_clock::time_point l, const std::string& name);

    void printNormal();
    void printLoop();

    double getDiff(const std::chrono::steady_clock::time_point& start,
                   const std::chrono::steady_clock::time_point& end) const;

    std::chrono::steady_clock::time_point begin_;
    std::chrono::steady_clock::time_point end_;
    std::vector<std::chrono::steady_clock::time_point> laps_;
    std::vector<std::string> lap_names_;
    std::map<std::string, double> durations_;
    std::string watchname_;
    std::chrono::steady_clock::time_point prevTime_;
    bool prevTimeSet_;
    bool inloop_;
    unsigned int loopcount_;
};

} // namespace ndt_generic
