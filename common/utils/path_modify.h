//
// Created by huyao on 10/31/17.
//

#ifndef PROJECT_EXPAND_HOME_DIR_H
#define PROJECT_EXPAND_HOME_DIR_H

#include <stdlib.h>
#include <string>
#include <cassert>

std::string expand_user(std::string path);

std::string expand_catkin_ws(std::string path);
std::string expand_catkin_ws(char *);

#endif //PROJECT_EXPAND_HOME_DIR_H
