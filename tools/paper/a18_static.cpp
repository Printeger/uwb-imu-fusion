#include "a18_certificate.h"
#include <filesystem>
#include <chrono>
#include <iostream>
int main(int argc,char**argv){try{if(argc!=3)throw std::runtime_error("MANIFEST OUTPUT required");auto start=std::chrono::steady_clock::now();std::filesystem::create_directory(argv[2]);std::ifstream f(argv[1]);std::string path;size_t n=0;while(std::getline(f,path)){auto data=a18::load(path);if(data.factors.size()!=371)throw std::runtime_error("FACTOR_COUNT");auto c=a18::certify(data);auto dir=std::filesystem::path(argv[2])/std::filesystem::path(path).filename();std::filesystem::create_directory(dir);a18::write((dir/"certificate.json").string(),c);a18::writeDetails(dir,c);++n;}std::cout<<"pairs="<<n<<" elapsed_s="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<" mpfr="<<mpfr_get_version()<<" ABI_size="<<sizeof(A18Mpfr)<<'\n';return n==43?0:2;}catch(const std::exception&e){std::cerr<<a18::exceptionStatus(e)<<":"<<e.what()<<'\n';return 2;}}
