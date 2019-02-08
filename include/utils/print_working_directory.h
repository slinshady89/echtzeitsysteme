//
// Created by frederic on 07.02.19.
//

#ifndef PROJECT_PRINT_WORKING_DIRECTORY_H
#define PROJECT_PRINT_WORKING_DIRECTORY_H

void printWorkingDirectory() {
    char dir_name[100];
    getcwd(dir_name, 100);
    ROS_INFO("Current directory is: %s", dir_name);
}

#endif //PROJECT_PRINT_WORKING_DIRECTORY_H
