//
// Created by zm on 18-12-1.
//

// #ifndef OFFB_POSCTL_PARAMETER_H
// #define OFFB_POSCTL_PARAMETER_H


class Parameter {

public:
    float x_p;
    float y_p;
    float z_p;

    float x_i;
    float y_i;
    float z_i;

    float x_d;
    float y_d;
    float z_d;

    float vx_p;
    float vy_p;
    float vz_p;

    float vx_i;
    float vy_i;
    float vz_i;

    float vx_d;
    float vy_d;
    float vz_d;

    float rpy_trim[3];


    bool readParam(const char* addr);

};


// #endif //OFFB_POSCTL_PARAMATER_H
