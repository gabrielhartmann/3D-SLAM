#include "Measurement.hpp"
#include "Utilities.h"

Measurement::Measurement()
{
    m.clear();
}

void Measurement::clear()
{
    m.clear();
}

bool Measurement::add(int lmTag, double u, double v)
{
    if (!contains(lmTag))
    {
        std::vector<double> pixel;
        pixel.push_back(u);
        pixel.push_back(v);
        m[lmTag] = pixel;
        return true;
    }
    
    return false;
}

bool Measurement::remove(int lmTag)
{
    if (contains(lmTag))
    {
        m.erase(lmTag);
        return true;
    }
    
    return false;
}

bool Measurement::contains(int lmTag)
{
    std::map<int, std::vector<double> >::iterator iter;
    iter = m.find(lmTag);
    if(iter == m.end())
    {
        return false;
    }
    
    return true;
}

int Measurement::size()
{
    return m.size();
}

std::vector<int> Measurement::getTags()
{
    std::vector<int> tags;
    for (std::map<int, std::vector<double> >::iterator iter=m.begin(); iter != m.end(); iter++)
    {
        tags.push_back(iter->first);
    }
    
    return tags;
}

std::vector<double> Measurement::getObservation(int tag)
{
    return m.find(tag)->second;
}

Eigen::VectorXd Measurement::toVector()
{
    Eigen::VectorXd v;
    v.resize(size() * 2);
    
    int i=0;
    for(std::map<int, std::vector<double> >::iterator iter = m.begin(); iter != m.end(); iter++)
    {
        v[i] = iter->second[0];
        v[i+1] = iter->second[1];
        i += 2;
    }
    
    return v;
}

void Measurement::print(std::string s)
{
    std::cout << s << " " << size() << std::endl;
    for (std::map<int, std::vector<double> >::iterator iter = m.begin(); iter != m.end(); iter++)
    {
        printf("%d: (%f, %f)\n", iter->first, iter->second[0], iter->second[1]);
    }
}