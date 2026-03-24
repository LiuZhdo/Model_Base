//
// Created by admin on 2025/05/21.
//

#ifndef IMP_OPT_CAR_RATIO_H
#define IMP_OPT_CAR_RATIO_H

#include <iostream>
#include <string>
#include "Eigen/Dense"
#include <vector>
#include "android_log.h"
#include <cmath>
#include <cstdio>
#include <ctime>
#define GPS_UTC_OFFSET 315964800        // GPS时间与UTC时间的固定偏移量(秒)

// 定义公历时间结构体
typedef struct {
    int year;    // 年
    int month;   // 月 (1-12)
    int day;     // 日 (1-31)
    int hour;    // 时 (0-23)
    int minute;  // 分 (0-59)
    int second;  // 秒 (0-59)
} GregorianTime;

// 闰秒表 (截至2023年)
static const struct {
    time_t gps_time;
    int leap_seconds;
} leap_seconds[] = {
        {46828800, 1},    // 1981-07-01
        {78364801, 2},    // 1982-07-01
        {109900802, 3},   // 1983-07-01
        {173059203, 4},   // 1985-07-01
        {252028804, 5},   // 1988-01-01
        {315187205, 6},   // 1990-01-01
        {346723206, 7},   // 1991-01-01
        {393984007, 8},   // 1992-07-01
        {425520008, 9},   // 1993-07-01
        {457056009, 10},  // 1994-07-01
        {504489610, 11},  // 1996-01-01
        {551750411, 12},  // 1997-07-01
        {599184012, 13},  // 1999-01-01
        {820108813, 14},  // 2006-01-01
        {914803214, 15},  // 2009-01-01
        {1025136015, 16}, // 2012-07-01
        {1119744016, 17}, // 2015-07-01
        {1167264017, 18}, // 2017-01-01
        {0, 0}           // 结束标记
};

using Matrix = Eigen::MatrixXd;
typedef struct Car_State{
    double x;
    double y;
    double psi;
    double v;
    double rol;
    double dpsi;
    double err;
}Car_State;

/*typedef struct ref_Point{
    double x;
    double y;
    double psi;
    double v;
}Ref_Point;*/

typedef struct Car_data{
    double car_K0;
    double car_K1;
    double car_U0;
}Car_data;

class LQR_car {
public:
    Car_data CarData{};
    LQR_car(uint a1, double a2, double a3);
    void Update_Q_R_matrix(double q11, double q22, double r00, double r11,int heading);
    double CALC(Eigen::Vector2d *dif_state, Eigen::MatrixXd *ptr_K);
    void update_car_state(double x,double y,double psi,double v);
    void Update_A_B_matrix(double l1);
    static int get_leap_seconds(time_t gps_time);
    static GregorianTime gps_to_gregorian(uint gps_week, double gps_seconds);

private:
    double tolerance;
    double dt;
    int index{};
    uint max_num_iteration;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd M;
    Eigen::MatrixXd P;

    // std::vector<Ref_Point> path{};
    Car_State cur_state{};

    void Solve(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R,
               const Matrix &M,Matrix *ptr_K);
    void Solve(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R,Matrix *ptr_K);
    void Solve(Matrix *ptr_K);

};

#endif //IMP_OPT_CAR_RATIO_H
