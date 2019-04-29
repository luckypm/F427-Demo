#include "alt_ukf.h"
#include "nav_ukf.h"
#include "imu.h"
#include <string.h>

altUkfStruct_t altUkfData;

void altUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, int n) {
    float acc;
    int i;

    // assume out == in
    out = in;

    for (i = 0; i < n; i++) {
        acc = u[0] + in[ALT_STATE_BIAS*n + i];

        out[ALT_STATE_BIAS*n + i] = in[ALT_STATE_BIAS*n + i] + (noise[ALT_NOISE_BIAS*n + i] * dt);
        out[ALT_STATE_VEL*n + i] = in[ALT_STATE_VEL*n + i] + (acc * dt) + (noise[ALT_NOISE_VEL*n + i] * dt);
        out[ALT_STATE_POS*n + i] = in[ALT_STATE_POS*n + i] - (in[ALT_STATE_VEL*n + i] * dt) - (acc * dt * dt * 0.5f);//高度上向上为正，而速度与ACC都是向下为正，故这里有负号
    }
}

void altUkfPresUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[ALT_STATE_POS] + noise[0];     // return altitude
}

static void altDoPresUpdate(float measuredPres) {
    float noise;        // measurement variance
    float y;            // measurment

    noise = ALT_PRES_NOISE;
    y = navUkfPresToAlt(measuredPres);

    srcdkfMeasurementUpdate(altUkfData.kf, 0, &y, 1, 1, &noise, altUkfPresUpdate);//这个源文件中只有高度才有测量更新，垂直方向的速度是没有测量更新，只有预测更新的
}

void altUkfProcess(float measuredPres) {//run.c中是从这个函数进入的
    float accIn[3];
    float acc[3];

    accIn[0] = IMU_ACCX;// + UKF_ACC_BIAS_X;
    accIn[1] = IMU_ACCY;// + UKF_ACC_BIAS_Y;
    accIn[2] = IMU_ACCZ;// + UKF_ACC_BIAS_Z;

    // rotate acc to world frame//将加速度的值旋转到地理坐标系，用acc[]表示
    navUkfRotateVectorByQuat(acc, accIn, &UKF_Q1);
    acc[2] += GRAVITY;

    srcdkfTimeUpdate(altUkfData.kf, &acc[2], AQ_OUTER_TIMESTEP);

    altDoPresUpdate(measuredPres);
}

void altUkfInit(void) {
    float Q[ALT_S];		// state variance  3
    float V[ALT_V];		// process variance 2

    memset((void *)&altUkfData, 0, sizeof(altUkfData));

    altUkfData.kf = srcdkfInit(ALT_S, ALT_M, ALT_V, ALT_N, altUkfTimeUpdate);

    altUkfData.x = srcdkfGetState(altUkfData.kf);

    Q[ALT_STATE_POS] = 5.0f;
    Q[ALT_STATE_VEL] = 1e-6f;
    Q[ALT_STATE_BIAS] = 0.05f;

    V[ALT_NOISE_BIAS] = ALT_BIAS_NOISE;
    V[ALT_NOISE_VEL] = ALT_VEL_NOISE;

    srcdkfSetVariance(altUkfData.kf, Q, V, 0, 0);

    ALT_POS = navUkfPresToAlt(AQ_PRESSURE);//由气压值计算出来的海拔高度
    ALT_VEL = 0.0f;
    ALT_BIAS = 0.0f;
}
