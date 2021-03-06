#ifndef PATH_H_IN
#define PATH_H_IN

#include <string>
// 在CMakeLists.txt中使用config设置:
// 当使用cmake构建的时候,带有.in的文件将会被宏替换然后生成新的文件,通过这个就能实现绝对路径的
// 更换, 提高了可移植性

const std::string NODE_NAME = "path_plan";
const std::string WORK_SPACE_PATH = "/home/jinzedong/PathPlan/DTW_c";
const std::string CONFIG_FILE_PATH = WORK_SPACE_PATH + "/config";

#endif
