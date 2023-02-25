//
// Created by huyao on 10/31/17.
//
#include "path_modify.h"

#include <unistd.h>
#include <cassert>
#include <string>
#include <string.h>

std::string expand_user(std::string path) {
    if (not path.empty() and path[0] == '~') {
        assert(path.size() == 1 or path[1] == '/');  // or other error handling
        char const* home = getenv("HOME");
        if (home or (home = getenv("USERPROFILE"))){
            path.replace(0, 1, home);
        } else {
            char const *hdrive = getenv("HOMEDRIVE"),
                    *hpath = getenv("HOMEPATH");
            assert(hdrive);  // or other error handling
            assert(hpath);
            path.replace(0, 1, std::string(hdrive) + hpath);
        }
    }
    return path;
}

size_t GetCurrentExcutableFilePathName( char* processdir,char* processname, size_t len)
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

std::string expand_catkin_ws(std::string path) {
    char buf1[80];
    char buf2[80];

    GetCurrentExcutableFilePathName( buf1, buf2, 80);
    std::string string_buf = buf1;

    return string_buf + "../../../" + path;
}

std::string expand_catkin_ws(char *path) {
    char buf1[80];
    char buf2[80];

    GetCurrentExcutableFilePathName( buf1, buf2, 80);
    std::string string_buf = buf1;

    return string_buf + "../../../" + path;
}




