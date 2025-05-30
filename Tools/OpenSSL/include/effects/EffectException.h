
#pragma once
#include <exception>
#include <string>
namespace RS21::direct_input {

class EffectException : public std::exception
{
public:
    EffectException(const std::string& errorMessage);
    EffectException(const char* errorMessage);
#ifdef COMPILER_MSVC
    virtual const char* what() const override{return m_errorStr.c_str();}
#else
//    virtual const char*
//    what() const _GLIBCXX_TXN_SAFE_DYN _GLIBCXX_NOTHROW {return m_errorStr.c_str();}
#endif
private:
    std::string m_errorStr;
};

}//namespace

