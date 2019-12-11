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
    bool set;
    float x0, x1, y0, y1, z0, z1;
    float dz;

    void write(float x, float y, float z);
};

struct laser{
    int sock;
    struct sockaddr_in server;
    int buffIdx = 0;

    bool mapIdx = 0;
    unsigned short int dist[LIDAR_RANGE];
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
    void calc_discard_values(int width);
    void transformPoint(int w, int h, float cz, float sz, float x, float y);
};

#endif