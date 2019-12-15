#include "control.h"
#include <iostream>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <string>
#include <sstream>
#include <math.h>
#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <mutex>
#include "motion.h"


controller::controller(imu *im_in){
    time_move = 100;
    speed_max = 1000;

    mtx = new std::mutex;

    im = im_in;

    std::thread contthread(&controller::main, this);
    contthread.detach();
}

void controller::main(){
    if((fd = serialOpen("/dev/ttyS0", 115200)) < 0){
        printf("fd: %d\n", fd);
    }

    for(int i = 0; i < 6; i++)
        for(int j = 0; j < 3; j++)
            save_command(servo_id[i][j], 0, off_set[i][j]);

    send_command();

    sleep(1);

    int interval = 500;
    auto time_interval = std::chrono::milliseconds(interval);
    auto time = std::chrono::steady_clock::now();
    

    while(1){
        control_gait();
        time = std::chrono::steady_clock::now() + time_interval;
        std::this_thread::sleep_until(time);
    }
}

void controller::control_gait(){
    switch(mp.gait){
        case 1:
            if(mp.W){
                tri_gait(1);
                go_to_zero();
            }else if(mp.S){
                tri_gait(0);
                go_to_zero();
            }else if(mp.A){
                turn(1);
            }else if(mp.D){
                turn(0);
            }
            break;
        case 2:
            if(mp.W){
                climb_gait(1);
                go_to_zero();
            }else if(mp.S){
                climb_gait(0);
                go_to_zero();
            }else if(mp.A){
                turn(1);
            }else if(mp.D){
                turn(0);
            }
            break;
    }

    //im->reset_vel();
}

void controller::tri_gait(bool dir){
    int ang = (dir ? -1 : RESOLUTION);
    bool first_state = 1;

    float speed;

    auto time = std::chrono::steady_clock::now();
    
    while(1){
        if(!(dir ? mp.W : mp.S)){
            return;
        }

        mtx->lock();

        speed = mp.speed / 100.0;

        mtx->unlock();

        time += std::chrono::milliseconds((int)(time_move / speed));

        ang = (dir ? ang + 1 : ang - 1);

        if(ang == (dir ? RESOLUTION : -1)){
            ang = (dir ? 0 : RESOLUTION - 1);
            first_state = !first_state;
        }

        if(first_state){
            for(int serv = 0; serv < 3; serv++){
                //First part up
                save_command(servo_id[0][serv], k.part1rf[serv][ang], off_set[0][serv]);
                save_command(servo_id[2][serv], k.part1rb[serv][ang], off_set[2][serv]);
                save_command(servo_id[4][serv], k.part1lc[serv][ang], off_set[4][serv]);

                //Second part pushes body
                save_command(servo_id[1][serv], k.part2rc[serv][ang], off_set[1][serv]); //RightCenterLeg
                save_command(servo_id[3][serv], k.part2lf[serv][ang], off_set[3][serv]); //LeftFrontLeg
                save_command(servo_id[5][serv], k.part2lb[serv][ang], off_set[5][serv]); //LeftBackLeg
            }
        } else { 
            for(int serv = 0; serv < 3; serv++){
                //First part goes down
                save_command(servo_id[0][serv], k.part2rf[serv][ang], off_set[0][serv]); //RightFrontLeg
                save_command(servo_id[2][serv], k.part2rb[serv][ang], off_set[2][serv]); //RightBackLeg
                save_command(servo_id[4][serv], k.part2lc[serv][ang], off_set[4][serv]); //LeftCenterLeg

                //Second part starts to move
                save_command(servo_id[1][serv], k.part1rc[serv][ang], off_set[1][serv]); //RightCenterLeg
                save_command(servo_id[3][serv], k.part1lf[serv][ang], off_set[3][serv]); //LeftFrontLeg
                save_command(servo_id[5][serv], k.part1lb[serv][ang], off_set[5][serv]); //LeftBackLeg
            }
        }

        send_command();
        std::this_thread::sleep_until(time);
    }
}


void controller::climb_gait(bool dir){
    
    int ang = (dir ? -1 : RESOLUTION);
    char state = 0;

    float speed;

    auto time = std::chrono::steady_clock::now();
    
    while(1){
        if(!(dir ? mp.W : mp.S)){
            return;
        }

        mtx->lock();

        speed = mp.speed / 100.0;

        mtx->unlock();

        

        time += std::chrono::milliseconds((int)(time_move / speed));

        ang = (dir ? ang + 1 : ang - 1);

        if(ang == (dir ? RESOLUTION : -1)){
            ang = (dir ? 0 : RESOLUTION - 1);
            state = (state < 7 - 1 ? state+1 : 0);
        }

        switch(state){
            case 0:
                for(int serv = 0; serv < 3; serv++){
                    save_command(servo_id[0][serv], k.part2rfc[serv][ang], off_set[0][serv]);
                }
                break;
            case 1:
                for(int serv = 0; serv < 3; serv++){
                    save_command(servo_id[3][serv], k.part2lfc[serv][ang], off_set[3][serv]);
                }
                break;
            case 2:
                for(int serv = 0; serv < 3; serv++){
                    save_command(servo_id[2][serv], k.part2rb[serv][ang], off_set[2][serv]);
                }
                break;
            case 3:
                for(int serv = 0; serv < 3; serv++){
                    save_command(servo_id[5][serv], k.part2lb[serv][ang], off_set[5][serv]);
                }
                break;
            case 4:
                for(int serv = 0; serv < 3; serv++){
                    save_command(servo_id[1][serv], k.part2rc[serv][ang], off_set[1][serv]);
                }
                break;
            case 5:
                for(int serv = 0; serv < 3; serv++){
                    save_command(servo_id[4][serv], k.part2lc[serv][ang], off_set[4][serv]);
                }
                break;
            case 6:
                for(int serv = 0; serv < 3; serv++){
                    save_command(servo_id[0][serv], k.part1rfc[serv][ang], off_set[0][serv]);
                    save_command(servo_id[1][serv], k.part1rc[serv][ang], off_set[1][serv]);
                    save_command(servo_id[2][serv], k.part1rb[serv][ang], off_set[2][serv]);
                    save_command(servo_id[3][serv], k.part1lfc[serv][ang], off_set[3][serv]);
                    save_command(servo_id[4][serv], k.part1lc[serv][ang], off_set[4][serv]);
                    save_command(servo_id[5][serv], k.part1lb[serv][ang], off_set[5][serv]);
                }
                break;
        }

        send_command();
        std::this_thread::sleep_until(time);
    }
}


void controller::save_command(char id, float angle, short int off_set){
    hc.i++;
    hc.motor[hc.i] = id;
    hc.pos[hc.i] = pwm_convert(angle, off_set);
}

void controller::send_command(){
    for(; hc.i > -1; hc.i--){
        ss << "#" << hc.motor[hc.i];
        ss << "P" << hc.pos[hc.i];
        ss << "S" << speed_max;
    }
    
    ss << "T" << time_move;
    ss << '\r' << '\n';
    
    std::string s = ss.str();
    const char *c = &s[0];
    serialPuts(fd, c);
    ss.str("");
}

short int controller::pwm_convert(float angle, short int off_set){
    float angle_to_pwm = 10; // 10 us per degree

    int angle_no_offset = (int) round(angle * angle_to_pwm);

    return angle_no_offset + off_set;
}

void controller::go_to_zero(){
    hc.i = -1;

    for(int i = 0; i < 2; i++){
        save_command(servo_id[1][1+i], 0, off_set[1][1+i]);
        save_command(servo_id[3][1+i], 0, off_set[3][1+i]);
        save_command(servo_id[5][1+i], 0, off_set[5][1+i]);
    }


    send_command();
    usleep(200000);

    for(int i = 0; i < 2; i++){
        save_command(servo_id[0+i][1], 45,          off_set[0+i][1]);
        save_command(servo_id[2+i][1], 45 - 90 * i, off_set[2+i][1]);
        save_command(servo_id[4+i][1], -45,         off_set[4+i][1]);

        send_command();
        usleep(200000);

        for(int j = 0; j < 3; j++)
            save_command(servo_id[j*2+i][0], 0, off_set[j*2+i][0]);

        send_command();
        usleep(200000);

        for(int j = 1; j < 3; j++)
            for(int k = 0; k < 3; k++)
                save_command(servo_id[k*2+i][j], 0, off_set[k*2+i][j]);

        send_command();
        usleep(200000);
    }
}

void controller::turn(bool ccw){
    auto time_interval = std::chrono::milliseconds(time_move);
    auto time = std::chrono::steady_clock::now() + time_interval;

    newRound: for(int ang = 0; ang < RESOLUTION; ang++){
        for(int serv = 0; serv < 3; serv++){
            save_command(servo_id[0][serv], (!ccw && serv == 0 ? -k.part1rrotf[serv][ang] : k.part1rrotf[serv][ang]), off_set[0][serv]);
            save_command(servo_id[1][serv], (!ccw && serv == 0 ? -k.part1rrotc[serv][ang] : k.part1rrotc[serv][ang]), off_set[1][serv]);
            save_command(servo_id[2][serv], (!ccw && serv == 0 ? -k.part1rrotb[serv][ang] : k.part1rrotb[serv][ang]), off_set[2][serv]);
            save_command(servo_id[3][serv], (!ccw && serv == 0 ? -k.part1lrotf[serv][ang] : k.part1lrotf[serv][ang]), off_set[3][serv]);
            save_command(servo_id[4][serv], (!ccw && serv == 0 ? -k.part1lrotc[serv][ang] : k.part1lrotc[serv][ang]), off_set[4][serv]);
            save_command(servo_id[5][serv], (!ccw && serv == 0 ? -k.part1lrotb[serv][ang] : k.part1lrotb[serv][ang]), off_set[5][serv]);
        }

        send_command();
        std::this_thread::sleep_until(time);
        time += time_interval;
    }

    for(int ang = 0; ang < RESOLUTION; ang++){
        for(int serv = 0; serv < 3; serv++){
            save_command(servo_id[0][serv], (!ccw && serv == 0 ? -k.part2rrotf[serv][ang] : k.part2rrotf[serv][ang]), off_set[0][serv]);
            save_command(servo_id[2][serv], (!ccw && serv == 0 ? -k.part2rrotb[serv][ang] : k.part2rrotb[serv][ang]), off_set[2][serv]);
            save_command(servo_id[4][serv], (!ccw && serv == 0 ? -k.part2lrotc[serv][ang] : k.part2lrotc[serv][ang]), off_set[4][serv]);
        }
        

        send_command();
        std::this_thread::sleep_until(time);
        time += time_interval;
    }

    for(int ang = 0; ang < RESOLUTION; ang++){
        for(int serv = 0; serv < 3; serv++){
            save_command(servo_id[1][serv], (!ccw && serv == 0 ? -k.part2rrotc[serv][ang] : k.part2rrotc[serv][ang]), off_set[1][serv]);
            save_command(servo_id[3][serv], (!ccw && serv == 0 ? -k.part2lrotf[serv][ang] : k.part2lrotf[serv][ang]), off_set[3][serv]);
            save_command(servo_id[5][serv], (!ccw && serv == 0 ? -k.part2lrotb[serv][ang] : k.part2lrotb[serv][ang]), off_set[5][serv]);
        }

        send_command();
        std::this_thread::sleep_until(time);
        time += time_interval;
    }

    if((ccw ? mp.A : mp.D))
        goto newRound;

}

void controller::write_mp(move_para in_mp){
    std::lock_guard<std::mutex> lock(*mtx);
    mp = in_mp;
}



kinematics::kinematics(){
    create_gait(pointPosf, part1rf, part2rf, part1lf, part2lf);
    create_gait(pointPosfc, part1rfc, part2rfc, part1lfc, part2lfc);
    create_gait(pointPosc, part1rc, part2rc, part1lc, part2lc);
    create_gait(pointPosb, part1rb, part2rb, part1lb, part2lb);

    create_turn_gait(turnPos, rc, -M_PI/2, M_PI/2, off_set_coordc, part1rrotc, part2rrotc, part1lrotc, part2lrotc);
    create_turn_gait(turnPos, rfb, -0.7444, M_PI/4, off_set_coordf, part1rrotf, part2rrotf, part1lrotf, part2lrotf);
    create_turn_gait(turnPos, rfb, -M_PI+0.7444, M_PI*3/4, off_set_coordb, part1rrotb, part2rrotb, part1lrotb, part2lrotb);
}

void kinematics::create_gait(float pointPos[3][5], float part1r[3][RESOLUTION], float part2r[3][RESOLUTION],
                                                   float part1l[3][RESOLUTION], float part2l[3][RESOLUTION]){
    float dpos[3];
    float stepPos[3];

    float angleTemp[3];

    int stage = 0;

    // Calculating push stage
    for(int i = 0; i < 3; i++)
        dpos[i] = (pointPos[i][stage + 1] - pointPos[i][stage]) / (RESOLUTION - 1);

    for(int step = 0; step < RESOLUTION; step++){
        for(int i = 0; i < 3; i++)
            stepPos[i] = pointPos[i][stage] + dpos[i] * step;
        
        getIK(stepPos[0], stepPos[1], stepPos[2], angleTemp);

        for(int motor = 0; motor < 3; motor++){
            part1r[motor][step] = angleTemp[motor];
            part1l[motor][step] = -part1r[motor][step];
        }
    }

    int step = 0;
    stage++;
    
    // Calculating ascension stage
    for(int i = 0; i < 3; i++)
        dpos[i] = (pointPos[i][stage + 1] - pointPos[i][stage]) / (RESOLUTION * 0.4);

    for(; step < RESOLUTION * 0.4; step++){
        for(int i = 0; i < 3; i++)
            stepPos[i] = pointPos[i][stage] + dpos[i] * (step+1);
        
        getIK(stepPos[0], stepPos[1], stepPos[2], angleTemp);

        for(int motor = 0; motor < 3; motor++){
            part2r[motor][step] = angleTemp[motor];
            part2l[motor][step] = -part2r[motor][step];
        }
    }

    stage++;
    
    // Calculating arch ascension stage
    for(int i = 0; i < 3; i++)
        dpos[i] = (pointPos[i][stage + 1] - pointPos[i][stage]) / (RESOLUTION * 0.1);

    for(; step < RESOLUTION * 0.5; step++){
        for(int i = 0; i < 3; i++)
            stepPos[i] = pointPos[i][stage] + dpos[i] * (step - RESOLUTION * 0.4 + 1);
        
        getIK(stepPos[0], stepPos[1], stepPos[2], angleTemp);

        for(int motor = 0; motor < 3; motor++){
            part2r[motor][step] = angleTemp[motor];
            part2l[motor][step] = -part2r[motor][step];
        }
    }

    stage++;

    // Calculating arc descension stage
    for(int i = 0; i < 3; i++)
        dpos[i] = (pointPos[i][stage+1] - pointPos[i][stage]) / (RESOLUTION * 0.1);

    for(; step < RESOLUTION * 0.6; step++){
        for(int i = 0; i < 3; i++)
            stepPos[i] = pointPos[i][stage] + dpos[i] * (step - RESOLUTION * 0.5 + 1);
        
        getIK(stepPos[0], stepPos[1], stepPos[2], angleTemp);

        for(int motor = 0; motor < 3; motor++){
            part2r[motor][step] = angleTemp[motor];
            part2l[motor][step] = -part2r[motor][step];
        }
    }

    stage++;

    // Calculating descension stage
    for(int i = 0; i < 3; i++)
        dpos[i] = (pointPos[i][0] - pointPos[i][stage]) / (RESOLUTION * 0.4);

    for(; step < RESOLUTION * 1; step++){
        for(int i = 0; i < 3; i++)
            stepPos[i] = pointPos[i][stage] + dpos[i] * (step - RESOLUTION * 0.6 + 1);
        
        getIK(stepPos[0], stepPos[1], stepPos[2], angleTemp);

        for(int motor = 0; motor < 3; motor++){
            part2r[motor][step] = angleTemp[motor];
            part2l[motor][step] = -part2r[motor][step];
        }
    }
}


void kinematics::create_turn_gait(float pointPos[2][5], float r, float zero_angle, float frame_angle, float off_set[2],
float part1r[3][RESOLUTION], float part2r[3][RESOLUTION], float part1l[3][RESOLUTION], float part2l[3][RESOLUTION]){

    // RF_angle: -47.3553 or -0.8137

    // RC_ee:      0, -206.4
    // RF_ee: 181.11, -166.8

    // RC_r :      0,  -71.4
    // RF_r :   85.7,  -71.4
    
    float dpos[2];
    float xr, yr;
    float x, y, z;

    float angleTemp[3];

    int stage = 0;

    // Calculating push stage
    for(int i = 0; i < 2; i++)
        dpos[i] = (pointPos[i][stage + 1] - pointPos[i][stage]) / (RESOLUTION - 1);

    for(int step = 0; step < RESOLUTION; step++){
        xr = r * cos(pointPos[0][stage] + dpos[0] * step + zero_angle);
        yr = r * sin(pointPos[0][stage] + dpos[0] * step + zero_angle);

        x = xr * cos(frame_angle) - yr * sin(frame_angle) + off_set[0];
        y = xr * sin(frame_angle) + yr * cos(frame_angle) + off_set[1];
        z = pointPos[1][stage] + dpos[1] * step;

        getIK(x, y, z, angleTemp);

        for(int motor = 0; motor < 3; motor++){
            part1r[motor][step] = angleTemp[motor];
            part1l[motor][step] = (motor ? -part1r[motor][step] : part1r[motor][step]);
        }
    }

    int step = 0;
    stage++;
    
    // Calculating ascension stage
    for(int i = 0; i < 2; i++)
        dpos[i] = (pointPos[i][stage + 1] - pointPos[i][stage]) / (RESOLUTION * 0.4);

    for(; step < RESOLUTION * 0.4; step++){
        xr = r * cos(pointPos[0][stage] + dpos[0] * (step+1) + zero_angle);
        yr = r * sin(pointPos[0][stage] + dpos[0] * (step+1) + zero_angle);

        x = xr * cos(frame_angle) - yr * sin(frame_angle) + off_set[0];
        y = xr * sin(frame_angle) + yr * cos(frame_angle) + off_set[1];
        z = pointPos[1][stage] + dpos[1] * (step+1);
        
        getIK(x, y, z, angleTemp);

        for(int motor = 0; motor < 3; motor++){
            part2r[motor][step] = angleTemp[motor];
            part2l[motor][step] = (motor ? -part2r[motor][step] : part2r[motor][step]);
        }
    }

    stage++;
    
    // Calculating arch ascension stage
    for(int i = 0; i < 2; i++)
        dpos[i] = (pointPos[i][stage + 1] - pointPos[i][stage]) / (RESOLUTION * 0.1);

    for(; step < RESOLUTION * 0.5; step++){
        xr = r * cos(pointPos[0][stage] + dpos[0] * (step - RESOLUTION * 0.4 + 1) + zero_angle);
        yr = r * sin(pointPos[0][stage] + dpos[0] * (step - RESOLUTION * 0.4 + 1) + zero_angle);

        x = xr * cos(frame_angle) - yr * sin(frame_angle) + off_set[0];
        y = xr * sin(frame_angle) + yr * cos(frame_angle) + off_set[1];
        z = pointPos[1][stage] + dpos[1] * (step - RESOLUTION * 0.4 + 1);
        
        getIK(x, y, z, angleTemp);

        for(int motor = 0; motor < 3; motor++){
            part2r[motor][step] = angleTemp[motor];
            part2l[motor][step] = (motor ? -part2r[motor][step] : part2r[motor][step]);
        }
    }

    stage++;

    // Calculating arc descension stage
    for(int i = 0; i < 2; i++)
        dpos[i] = (pointPos[i][stage+1] - pointPos[i][stage]) / (RESOLUTION * 0.1);

    for(; step < RESOLUTION * 0.6; step++){
        xr = r * cos(pointPos[0][stage] + dpos[0] * (step - RESOLUTION * 0.5 + 1) + zero_angle);
        yr = r * sin(pointPos[0][stage] + dpos[0] * (step - RESOLUTION * 0.5 + 1) + zero_angle);

        x = xr * cos(frame_angle) - yr * sin(frame_angle) + off_set[0];
        y = xr * sin(frame_angle) + yr * cos(frame_angle) + off_set[1];
        z = pointPos[1][stage] + dpos[1] * (step - RESOLUTION * 0.5 + 1);
        
        getIK(x, y, z, angleTemp);

        for(int motor = 0; motor < 3; motor++){
            part2r[motor][step] = angleTemp[motor];
            part2l[motor][step] = (motor ? -part2r[motor][step] : part2r[motor][step]);
        }
    }

    stage++;

    // Calculating descension stage
    for(int i = 0; i < 2; i++)
        dpos[i] = (pointPos[i][0] - pointPos[i][stage]) / (RESOLUTION * 0.4);

    for(; step < RESOLUTION * 1; step++){
        xr = r * cos(pointPos[0][stage] + dpos[0] * (step - RESOLUTION * 0.6 + 1) + zero_angle);
        yr = r * sin(pointPos[0][stage] + dpos[0] * (step - RESOLUTION * 0.6 + 1) + zero_angle);

        x = xr * cos(frame_angle) - yr * sin(frame_angle) + off_set[0];
        y = xr * sin(frame_angle) + yr * cos(frame_angle) + off_set[1];
        z = pointPos[1][stage] + dpos[1] * (step - RESOLUTION * 0.6 + 1);
        
        getIK(x, y, z, angleTemp);

        for(int motor = 0; motor < 3; motor++){
            part2r[motor][step] = angleTemp[motor];
            part2l[motor][step] = (motor ? -part2r[motor][step] : part2r[motor][step]);
        }
    }
}

void kinematics::getIK(float x, float y, float z, float* out){
    float x_2D = sqrt(pow(x, 2) + pow(y, 2)) - l1;

    out[0] = atan2(y, x);
    out[2] = -acos(make_range((pow(x_2D, 2) + pow(z, 2) - pow(l2, 2) - pow(l3, 2)) / (2 * l2 * l3))) + M_PI/2;

    c1 = l2 + l3 * cos(out[2] - M_PI/2);
    c2 = l3 * sin(out[2] - M_PI/2);
    c3 = -x_2D;
    c4 = z;

    out[1] = atan2(-c1*c4-c2*c3, c2*c4-c1*c3);

    for(int i = 0; i < 3; i++)
        out[i] = -out[i] * 180 / M_PI; // Converts to degreees
}

float kinematics::make_range(float in){
    if(in > 1)
        return 1;
    else if(in < -1)
        return -1;
    else
        return in;

}