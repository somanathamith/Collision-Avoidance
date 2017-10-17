#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <iostream>
#include <ros/ros.h>

// ---------------------------------------------------------------------
// Operator overloading to enable printing directly using cout
// ---------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const std::vector<double>& v) 
{
    os << "[";
    for (std::vector<double>::const_iterator it = v.begin() ; it != v.end(); ++it) {
        os << ' ' << *it;
    }
    os << "]";
    return os;
}
std::ostream& operator<<(std::ostream& os, const std::vector<std::string>& v) 
{
    os << "[";
    for (std::vector<std::string>::const_iterator it = v.begin() ; it != v.end(); ++it) {
        os << ' ' << *it;
    }
    os << "]";
    return os;
}
std::ostream& operator<<(std::ostream& os, const std::vector<ros::Time>& v) 
{
    os << "[";
    for (std::vector<ros::Time>::const_iterator it = v.begin() ; it != v.end(); ++it) {
        os << ' ' << *it;
    }
    os << "]";
    return os;
}

// ---------------------------------------------------------------------
// Operator overloading to enable vector addition, subtraction and dot product
// ---------------------------------------------------------------------

std::vector<double> operator+(const std::vector<double>& v1, const std::vector<double>& v2)
{
    std::vector<double> v;
    for(int i=0; i<v1.size(); i++)
    {
        v.push_back(v1[i]+v2[i]);
    }
    return v;
}
std::vector<double> operator-(const std::vector<double>& v1, const std::vector<double>& v2)
{
    std::vector<double> v;
    for(int i=0; i<v1.size(); i++)
    {
        v.push_back(v1[i]-v2[i]);
    }
    return v;
}
std::vector<double> operator*(const std::vector<double>& v1, const std::vector<double>& v2)
{
    std::vector<double> v;
    for(int i=0; i<v1.size(); i++)
    {
        v.push_back(v1[i]*v2[i]);
    }
    return v;
}
std::vector<double> operator*(std::vector<double>& v, double alpha)
{
    for(int i=0; i<v.size(); i++)
    {
        v[i] = v[i]*alpha;
    }
    return v;
}
std::vector<double> operator*(double alpha, std::vector<double>& v)
{
    for(int i=0; i<v.size(); i++)
    {
        v[i] = v[i]*alpha;
    }
    return v;
}

double distance(const std::vector<double>& a, const std::vector<double>& b)
{
    double sum = 0;
    for (int i = 0; i < a.size(); i++)
    {
        double d = b[i] - a[i];
        sum += d * d;
    }
    return sqrt(sum);
}
    
#endif /* COMMON_H */
