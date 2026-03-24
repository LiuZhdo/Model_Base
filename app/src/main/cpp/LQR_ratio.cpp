//
// Created by admin on 2025/05/21.
//

#include "LQR_ratio.h"

void LQR_car::Solve(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R,
                    const Matrix &M, Matrix *ptr_K) {
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
        Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols() ||
        M.rows() != Q.rows() || M.cols() != R.cols()) {
        // LOGE( "LQR solver: one or more matrices have incompatible dimensions.");
        return;
    }
    Matrix AT = A.transpose();
    Matrix BT = B.transpose();
    Matrix MT = M.transpose();
    P = Q;
    uint num_iteration = 0;
    double diff = std::numeric_limits<double>::max();
    while (num_iteration++ < max_num_iteration && diff > tolerance) {
        Matrix P_next =
                AT * P * A -
                (AT * P * B + M) * (R + BT * P * B).inverse() * (BT * P * A + MT) + Q;
        // check the difference between P and P_next
        diff = fabs((P_next - P).maxCoeff());
        P = P_next;
    }
/*     LOGI("Riccati Matrix P=%s", toString(P).data());
     if (num_iteration >= max_num_iteration) {
         LOGI( "LQR solver cannot converge to a solution,"
               "last consecutive result diff is: %lf",diff);
     } else {
         LOGI("LQR solver converged at iteration:%d "
              "max consecutive result diff%lf.: ",num_iteration,diff);
     }*/
    *ptr_K = (R + BT * P * B).inverse() * (BT * P * A + MT);
}

void LQR_car::Solve(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R,
                    Matrix *ptr_K) {
    Matrix M = Matrix::Zero(Q.rows(), R.cols());
    this->Solve(A, B, Q, R, M, ptr_K);
}

void LQR_car::Update_Q_R_matrix(double q11, double q22, double r00, double r11,int heading){
    double q1,q2,r;
    q1 = q11, q2 = q22, r = r00;
    if(heading == 1){
        q1 = q11;
        // 参数自适应车速 车速过快调节力度需要小
        if(abs(cur_state.v)<0.75 && abs(cur_state.v)>0.2){      // 0.036~2.5km/h
            q2 = 135;
            r = 70;
        }
        if(abs(cur_state.v)<1.2 && abs(cur_state.v)>0.75){       // 2.5~4.32km/h
            q2 = 115;
            r = 80;
        }
        if(abs(cur_state.v)<1.75 && abs(cur_state.v)>1.2){       // 4.32~6.3km/h
            q2 = 125;
            r = 80;
        }
        if(abs(cur_state.v)< 2 && abs(cur_state.v)>1.75){       // 6.3~7.2km/h
            q2 = 120;
            r = 95;
        }
        if(abs(cur_state.v)< 2.5 && abs(cur_state.v)>2){        // 7.2~9km/h
            q2 = 110;
            r = 120;
        }
        if(abs(cur_state.v)< 3 && abs(cur_state.v)>2.5){       // 9~10.8km/h
            q2 = 95;
            r = 145;
        }
        if(abs(cur_state.v)< 3.5 && abs(cur_state.v)>3){       // 10.8~11.6km/h
            q2 = 75;
            r = 175;
        }
        if(abs(cur_state.v)>3.5){                               // >11.6km/h
            q2 = 60;
            r = 200;
        }
    }
    Q(0,0)=q1;
    Q(1,1)=q2;
    R(0,0)=r;
    R(1,1)=r11;
}

void LQR_car::Solve(Matrix *ptr_K) {
    this->Solve(A, B, Q, R, ptr_K);
}

void LQR_car::Update_A_B_matrix(double L11)  {
//void LQR::Update_A_B_matrix(double L11, double L22, double L33)  {
// 倒车时车速符号取反了。(倒车时车速为负值)
    A(0,0)=1.0;
    A(1,0)=cur_state.v*dt;
    A(1,1)=1.0;
    B(0,0)=cur_state.v*dt/L11;
}

double LQR_car::CALC(Eigen::Vector2d *dif_state, Eigen::MatrixXd *ptr_K) {
    this->Solve(A,B,Q,R,ptr_K);
    Matrix U=-(*ptr_K) * (*dif_state);
    Matrix KK=*ptr_K;
    LOGI("KK1=%lf,KK2=%lf",KK(0,0),KK(0,1));
    LOGI("U1=%lf,U2=%lf",U(0,0),U(1,0));
    CarData.car_K0 = KK(0,0);
    CarData.car_K1 = KK(0,1);
    CarData.car_U0 = U(0,0);
    return U(0,0);
}

LQR_car::LQR_car(uint a1, double a2, double a3){
    tolerance=a2;
    max_num_iteration=a1;
    dt=a3;

    A = Matrix::Zero(2, 2);
    B = Matrix::Zero(2, 2);
    Q = Matrix::Zero(2, 2);
    R = Matrix::Zero(2, 2);
    P = Matrix::Zero(2, 2);
}

void LQR_car::update_car_state(double x, double y, double psi, double v) {
    cur_state={x,y,psi,v};
}

// 获取给定GPS时间的闰秒数
int LQR_car::get_leap_seconds(time_t gps_time) {
    int i;
    int leap_sec = 0;

    for (i = 0; leap_seconds[i].gps_time != 0; i++) {
        if (gps_time >= leap_seconds[i].gps_time) {
            leap_sec = leap_seconds[i].leap_seconds;
        } else {
            break;
        }
    }

    return leap_sec;
}

// 将GPS周内秒数转换为公历时间
// 参数: gps_week - GPS周数
//       gps_seconds - GPS周内秒数
// 返回: GregorianTime结构体，包含年月日时分秒
GregorianTime LQR_car::gps_to_gregorian(uint gps_week, double gps_seconds) {
    time_t gps_time;
    time_t unix_time;
    int leap_sec;
    struct tm tm_time{};
    GregorianTime result;

    // 计算GPS时间(从1980-01-06开始的秒数)
    gps_time = (time_t)gps_week * 7 * 24 * 3600 + (time_t)floor(gps_seconds);

    // 转换为UNIX时间(从1970-01-01开始的秒数)
    unix_time = gps_time + GPS_UTC_OFFSET;

    // 获取闰秒数
    leap_sec = get_leap_seconds(gps_time);

    // 减去闰秒得到UTC时间
    unix_time -= leap_sec;

    // 转换为tm结构
    gmtime_r(&unix_time, &tm_time);

    // 填充返回结构体
    result.year = tm_time.tm_year + 1900;
    result.month = tm_time.tm_mon + 1;
    result.day = tm_time.tm_mday;
    result.hour = tm_time.tm_hour;
    result.minute = tm_time.tm_min;
    result.second = tm_time.tm_sec;

    return result;
}