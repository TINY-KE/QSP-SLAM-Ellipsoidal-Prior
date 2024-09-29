#include "include/utils/Logger.h"

using namespace std;



Logger* Logger::m_instance = NULL;


Logger::Logger()
{}

Logger::~Logger()
{}

Logger* Logger::instance()
{
    if (m_instance == NULL){
        m_instance = new Logger();
    }
    return m_instance;
}


void Logger::open(const string& filename)
{
    m_fout.open(filename, ios::app);
    if(m_fout.fail())
    {
        throw std::logic_error("open file failed " + filename);
    }
}

void Logger::close()
{
    
}
