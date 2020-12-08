#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include <fcntl.h>
#include <string>
#include <map>
#include <vector>
#include <iostream>
#include <unistd.h>
#include "compile.h"

#include "node.h"

using namespace std;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto n = std::make_shared<DrivekitNode>();
    if(n->run)
    {
        rclcpp::spin(n);
        n->shutdown();
        rclcpp::shutdown();
    }

    return 0;
}