#ifndef LASER_H
#define LASER_H
#include "laser.h"
#include <arpa/inet.h> //inet_addr
#include <mutex>

#define LIDAR_RANGE 61
#define MAP_W 11 // Each coordinate is 5x5 cm
#define MAP_H 31

struct pos_lidar{
    float x = 0, y = 0;
    float rz = 0;
};

struct point{
    bool set = 0;
    float x0 = 0, x1 = 0, y0 = 0, y1 = 0, z0 = 0, z1 = 0;
    float dz = 0;

    void write(float x, float y, float z);
};

struct laser{
    int sock;
    struct sockaddr_in server;
    int buffIdx = 0;

    bool mapIdx = 0;
    unsigned short int dist[LIDAR_RANGE];
    float dist_cos_angles[LIDAR_RANGE];
    float dist_sin_angles[LIDAR_RANGE];
    float dist_cos_tilt;
    point map[2][MAP_W][MAP_H];
    unsigned short int dist_discard[LIDAR_RANGE];
    uint8_t tcp_data[233];

    short int obsDist = 0;

    std::mutex *mtx;

    pos_lidar pl;

    laser();

    void main();

    void start_scan();
    void stop_scan();
    void read_data();
    void write_pos(pos_lidar pos);
    
    pos_lidar read_pos();

    void shiftMap();
    void calc_init_values(int width);
    void transformPoint(int w, int h, float cz, float sz, float x, float y);
    void add_data_to_map();
};

#endif