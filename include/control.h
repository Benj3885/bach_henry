#ifndef CONTROL_H
#define CONTROL_H
#include "control.h"
#include <sstream>
#include <math.h>
#include "motion.h"
#include <mutex>

struct hex_command{
    int i = -1;

    int motor[18] = {0};
    short int pos[18] = {0};
};

struct move_para{
    bool W, A, S, D;
    uint8_t speed;
    uint8_t gait;
};

struct kinematics{
    #define RESOLUTION 10 // Make this a multiple of 10

    float pointPosf[3][5] = {{153.1, 103.5, 103.5, 128.3, 153.1}, {37.7, 87.2, 87.2, 62.5, 37.7}, {-120, -120, -40, 0, -40}};
    float pointPosc[3][5] = {{135, 135, 135, 135, 135}, {-35, 35, 35, 0, -35}, {-120, -120, -40, 0, -40}};
    float pointPosb[3][5] = {{103.5, 153.1, 153.1, 128.3, 103.5}, {0, 49.5, 49.5, 24.7, 0}, {-120, -120, -40, 0, -40}};

    float pointPosfc[3][5] = {{180.5, 130.3, 130.3, 155.1, 180.5}, {-34.9, 14.7, 14.7, -10.1, -34.9}, {-120, -120, -40, 0, -40}};

    float l1 = 53.97, l2 = 80.94, l3 = 120.24;
    float c1, c2, c3, c4;

    float part1rf[3][RESOLUTION], part2rf[3][RESOLUTION];
    float part1lf[3][RESOLUTION], part2lf[3][RESOLUTION];

    float part1rfc[3][RESOLUTION], part2rfc[3][RESOLUTION];
    float part1lfc[3][RESOLUTION], part2lfc[3][RESOLUTION];

    float part1rc[3][RESOLUTION], part2rc[3][RESOLUTION];
    float part1lc[3][RESOLUTION], part2lc[3][RESOLUTION];

    float part1rb[3][RESOLUTION], part2rb[3][RESOLUTION];
    float part1lb[3][RESOLUTION], part2lb[3][RESOLUTION];

    void create_gait(float pointPos[3][5], float part1r[3][RESOLUTION], float part2r[3][RESOLUTION],
                                           float part1l[3][RESOLUTION], float part2l[3][RESOLUTION]);

    float part1rrotc[3][RESOLUTION], part2rrotc[3][RESOLUTION];
    float part1lrotc[3][RESOLUTION], part2lrotc[3][RESOLUTION];

    float part1rrotf[3][RESOLUTION], part2rrotf[3][RESOLUTION];
    float part1lrotf[3][RESOLUTION], part2lrotf[3][RESOLUTION];

    float part1rrotb[3][RESOLUTION], part2rrotb[3][RESOLUTION];
    float part1lrotb[3][RESOLUTION], part2lrotb[3][RESOLUTION];

    float turnPos[2][5] = {{0, M_PI/12, M_PI/12, M_PI/24, 0}, {-120, -120, -40, 0, -40}};

    float off_set_coordc[2] = {-71.4502,     0};
    float off_set_coordf[2] = {-111.1396, -10.0938};
    float off_set_coordb[2] = {-111.1396,  10.0938};

    float rfb = 246.3, rc = 206.4;
    

    kinematics();

    void getIK(float x, float y, float z, float* out);

    void create_turn_gait(float pointPos[2][5], float r, float zero_angle, float frame_angle, float off_set[2],
    float part1r[3][RESOLUTION], float part2r[3][RESOLUTION], float part1l[3][RESOLUTION], float part2l[3][RESOLUTION]);
    float make_range(float in);
};


struct controller{
    std::stringstream ss;
    hex_command hc;
    move_para mp;

    std::mutex *mtx;

    int fd;
    int time_move;
    int speed_max;

    kinematics k;

    //ids and offset
    //RIght side first from front to back
    char servo_id[6][3] = {{10, 9, 8}, {6, 5, 4}, {1, 2, 3}, {26, 25, 24}, {22, 21, 20}, {18, 17, 16}};
    short int off_set[6][3] = {{1455, 1614, 1463}, {1538, 1500, 1518}, {1549, 1558, 1461}, {1538, 1469, 1479}, {1529, 1500, 1566}, {1562, 1517, 1471}};

    short int off_set_climbing[6][3] = {{300, 0,0}, {220, 0, 0}, {200, 0, 0}, {-300, 0, 0}, {-220, 0, 0}, {-200, 0, 0}};
    float climb_modifier = 0.4;

    imu *im;

    controller(imu *im_in);

    void main();

    void tri_gait(bool dir);
    void climb_gait(bool dir);
    void go_to_zero();
    void turn(bool p);
    void control_gait();

    void save_command(char id, float angle, short int off_set);
    void send_command();
    short int pwm_convert(float angle, short int off_set);
    void write_mp(move_para in_mp);
};

#endif