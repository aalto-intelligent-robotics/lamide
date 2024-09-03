#include <ndt_generic_lamide/utils.h>
#include <iostream>
#include <iomanip>

namespace ndt_generic
{

void getVectorMeanStdev(const std::vector<double>& v, double& mean, double& stdev)
{

    mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();

    std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(), std::bind2nd(std::minus<double>(), mean));
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    stdev = std::sqrt(sq_sum / v.size());
}

void getVectorMinMax(const std::vector<double>& v, double& min, double& max)
{

    min = *std::min_element(v.begin(), v.end());
    max = *std::max_element(v.begin(), v.end());
}

void getVectorQuartiles(const std::vector<double>& vec, double& q1, double& median, double& q3)
{
    std::vector<double> v(vec);
    std::sort(v.begin(), v.end());
    q1 = v[v.size() * 1 / 4];
    median = v[v.size() * 2 / 4];
    q3 = v[v.size() * 3 / 4];
}

void normalizeVector(std::vector<double>& v)
{
    double weight_factor = std::accumulate(v.begin(), v.end(), 0.);
    std::vector<double>::iterator it;
    for (it = v.begin(); it != v.end(); it++)
        *it = *it * weight_factor;
}

std::string getVectorStatisticStr(const std::vector<double>& data)
{
    double mean, stdev, min, max, q1, median, q3;
    ndt_generic::getVectorMeanStdev(data, mean, stdev);
    ndt_generic::getVectorMinMax(data, min, max);
    ndt_generic::getVectorQuartiles(data, q1, median, q3);

    std::string ret("[mean]: " + toString(mean) + "\n[stdev]:" + toString(stdev) + "\n[min]: " +
                    toString(min) + "\n[max] :" + toString(max) + "\n[q1]: " + toString(q1) +
                    "\n[median]: " + toString(median) + "\n[q3]: " + toString(q3));
    return ret;
}

std::string getVectorStatisticGnuplotStr(const std::vector<double>& data)
{
    double mean, stdev, min, max, q1, median, q3;
    ndt_generic::getVectorMeanStdev(data, mean, stdev);
    ndt_generic::getVectorMinMax(data, min, max);
    ndt_generic::getVectorQuartiles(data, q1, median, q3);

    std::string ret(toString(mean) + "\t" + toString(stdev) + "\t" + toString(min) + "\t" +
                    toString(max) + "\t" + toString(q1) + "\t" + toString(median) + "\t" +
                    toString(q3));
    return ret;
}

double getDoubleTime()
{
    struct timeval time;
    gettimeofday(&time, NULL);
    return time.tv_sec + time.tv_usec * 1e-6;
}
//!
//! \brief currentDateTimeString return the current time formated as a file-name
//! compatible string
//! \return not used
//!
const std::string currentDateTimeString()
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "d%Y_%m_%d_time%H_%M_%S", &tstruct);

    return buf;
}

std::vector<std::string> splitLine(const std::string& line, const std::string& delimiter)
{
    std::string orig = line;
    size_t pos = 0;
    std::vector<std::string> tokens;
    while ((pos = orig.find(delimiter)) != std::string::npos)
    {
        std::string token = orig.substr(0, pos);
        tokens.push_back(token);
        orig.erase(0, pos + delimiter.length());
    }
    tokens.push_back(orig);
    return tokens;
}

std::string getISOdate()
{
    time_t now;
    time(&now);
    char buf[sizeof "2011-10-08T07:07:09Z"];
    strftime(buf, sizeof buf, "%FT%TZ", localtime(&now));
    return std::string(buf);
}

// ███████╗████████╗ ██████╗ ██████╗ ██╗    ██╗ █████╗ ████████╗ ██████╗██╗  ██╗
// ██╔════╝╚══██╔══╝██╔═══██╗██╔══██╗██║    ██║██╔══██╗╚══██╔══╝██╔════╝██║  ██║
// ███████╗   ██║   ██║   ██║██████╔╝██║ █╗ ██║███████║   ██║   ██║     ███████║
// ╚════██║   ██║   ██║   ██║██╔═══╝ ██║███╗██║██╔══██║   ██║   ██║     ██╔══██║
// ███████║   ██║   ╚██████╔╝██║     ╚███╔███╔╝██║  ██║   ██║   ╚██████╗██║  ██║
// ╚══════╝   ╚═╝    ╚═════╝ ╚═╝      ╚══╝╚══╝ ╚═╝  ╚═╝   ╚═╝    ╚═════╝╚═╝  ╚═╝


Stopwatch::Stopwatch(const std::string& name, bool inloop)
    : watchname_(name)
    , inloop_(inloop)
    , loopcount_(0)
    , prevTimeSet_(false)
{
    start();
}

void Stopwatch::start()
{
    begin_ = std::chrono::steady_clock::now();
    laps_.clear();
    lap_names_.clear();
}

void Stopwatch::stop()
{
    end_ = std::chrono::steady_clock::now();
}

void Stopwatch::lap(const std::string& name)
{
    // note: good for debugging
    // std::cout << "lap " << name << std::endl;
    std::chrono::steady_clock::time_point l = std::chrono::steady_clock::now();
    std::string usedname = name;
    if (usedname.empty())
    {
        usedname = "" + (laps_.size() + 1);
    }

    if (inloop_)
    {
        loopLap(l, usedname);
    }
    else
    {
        normalLap(l, usedname);
    }

    prevTime_ = l;
    prevTimeSet_ = true;
}

void Stopwatch::normalLap(const std::chrono::steady_clock::time_point l, const std::string& name)
{
    laps_.push_back(l);
    lap_names_.push_back(name);
}

void Stopwatch::loopLap(const std::chrono::steady_clock::time_point l, const std::string& name)
{
    double dur;
    if (prevTimeSet_)
    {
        dur = getDiff(prevTime_, l);
    }
    else
    {
        dur = getDiff(begin_, l);
    }

    double existing = durations_[name];

    double recursiveAverage = ((dur) + (existing * loopcount_)) / (loopcount_ + 1);

    durations_[name] = recursiveAverage;
}

void Stopwatch::loopStart()
{
    loopcount_++;
}

void Stopwatch::print()
{
    if (inloop_)
    {
        printLoop();
    }
    else
    {
        printNormal();
    }
}

void Stopwatch::printLoop()
{
    double totalDur = getDiff(begin_, end_);
    std::cout << "**************" << std::endl;
    std::cout << std::fixed << std::setprecision(10);
    std::cout << "Total: " << totalDur << std::endl;
    std::cout << "Loops: " << loopcount_ << std::endl;
    std::cout << "Laps:" << std::endl;
    int width = 15;

    std::map<std::string, double>::iterator it;

    for (it = durations_.begin(); it != durations_.end(); ++it)
    {
        std::string printname = it->first;
        double dur = it->second;
        double p = (dur * loopcount_) / totalDur * 100.0;
        std::cout << std::setw(width) << printname << "    " << std::setw(width) << dur
                  << " s    " << std::setw(width) << p << " %" << std::endl;
    }
}

void Stopwatch::printNormal()
{
    double totalDur = getDiff(begin_, end_);
    std::cout << watchname_ << std::endl;
    std::cout << "**************" << std::endl;
    std::cout << std::fixed << std::setprecision(10);
    std::cout << "Total: " << totalDur << std::endl;
    std::cout << "Laps:" << std::endl;
    int width = 15;
    for (unsigned int i = 0; i < laps_.size(); i++)
    {
        std::chrono::steady_clock::time_point prev;
        if (i == 0)
        {
            prev = begin_;
        }
        else
        {
            prev = laps_[i - 1];
        }
        std::string printname = lap_names_[i];
        double dur = getDiff(prev, laps_[i]);
        double p = dur / totalDur * 100.0;
        std::cout << std::setw(width) << printname << "    " << std::setw(width) << dur << " s    "
                  << std::setw(width) << p << " %" << std::endl;
    }
}

double Stopwatch::getDiff(const std::chrono::steady_clock::time_point& start,
                          const std::chrono::steady_clock::time_point& end) const
{
    std::chrono::duration<double> diff = end - start;
    return diff.count();
}
} // namespace ndt_generic
