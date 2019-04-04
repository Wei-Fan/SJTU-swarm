//
// Created by zm on 18-12-1.
//

#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <math.h>
#include "string"
#include <time.h>
#include <queue>
#include <vector>
#include "Parameter.h"

bool Parameter::readParam(const char *addr) {

    std::ifstream fs;

    std::string name = "";
    float value[3];

    fs.open(addr);

    if (!fs)
    {
        std::cout << "parameter file err" << std::endl;
        return false;
    }

    while (!fs.eof())
    {
        fs >> name >> value[0] >> value[1] >> value[2];

        if (name == "P")
        {
            x_p = value[0];
            y_p = value[1];
            z_p = value[2];
        }

        else if (name == "I")
        {
            x_i = value[0];
            y_i = value[1];
            z_i = value[2];
        }

        else if (name == "D")
        {
            x_d = value[0];
            y_d = value[1];
            z_d = value[2];
        }
        
        else if (name == "vP")
        {
            vx_p = value[0];
            vy_p = value[1];
            vz_p = value[2];

        }
        else if(name == "vI")
        {
            vx_i = value[0];
            vy_i = value[1];
            vz_i = value[2];

        }
        else if (name == "vD")
        {
            vx_d = value[0];
            vy_d = value[1];
            vz_d = value[2];

        }
        else if (name == "rpy_trim")
        {
            rpy_trim[0] = value[0];
            rpy_trim[1] = value[1];
            rpy_trim[2] = value[2];
        }

    }
    std::cout << "read config file successfully!"<<std::endl;

    fs.close();

    return true;
}
