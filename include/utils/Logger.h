#pragma once

#include <iostream>
#include <fstream>

using namespace std;

class Logger
{
public:
    enum Level
    {
        DEBUG = 0,
        INFO,
        WARN,
        ERROR,
        FATAL,
        LEVEL_COUNT
    };

    Logger* instance();

    void open(const string& filename);
    void close();
    
    void log(Level level, const char *file, int line, const char *format, ...);

private:
    Logger();
    ~Logger();

private:
    std::string m_filename;
    std::ofstream m_fout;

    static const char *s_level[LEVEL_COUNT];

    static Logger *m_instance;


};