//
// Created by luyifan on 19-6-10.
//

#ifndef HDMAP_COMMON_H
#define HDMAP_COMMON_H

#include <string>
#include <vector>
#include <iostream>
#include "log.h"
#include <memory>
#include <unordered_map>
#include <utility>
#include <unordered_set>
#include <algorithm>

using std::cout;
using std::endl;

inline size_t GetCurrentExcutableFilePathName( char* processdir,char* processname, size_t len)
{
    char* path_end;
    if(readlink("/proc/self/exe", processdir,len) <=0)
        return -1;
    path_end = strrchr(processdir,  '/');
    if(path_end == NULL)
        return -1;
    ++path_end;
    strcpy(processname, path_end);
    *path_end = '\0';
    return (size_t)(path_end - processdir);
}

inline std::string expand_catkin_ws(std::string path) {
    char buf1[80];
    char buf2[80];

    GetCurrentExcutableFilePathName( buf1, buf2, 80);
    std::string string_buf = buf1;

    return string_buf + "../../../" + path;
}


#endif //HDMAP_COMMON_H
