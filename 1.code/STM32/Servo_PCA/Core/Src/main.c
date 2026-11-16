/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : STM32 Drone Flight Controller (Final Stable Version)
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "string.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include <stdatomic.h>
#include <stdbool.h>

float vz_pid_integ = 0.0f;
float vz_pid_prev_err = 0.0f;

// 硬體定義
#define PCA9685_ADDR 0x80
#define BMX160_ADDR  (0x68u << 1)
#define MODE1 0x00
#define PRESCALE 0xFE
#define LED0_ON_L 0x06
#define PWM_MIN 1000
#define PWM_MAX 2000

// 全域旗標
volatile uint8_t i2c2_busy = 0;
volatile uint8_t data_ready_flag = 0;
volatile uint8_t cmd_received_flag = 0;
static uint32_t last_cmd_time = 0;

// 緩衝區
#define RX_BUF_SIZE 64
#define ALT_BUF_SIZE 128
uint8_t rx_buffer[RX_BUF_SIZE];
uint8_t rx_alt_buffer[ALT_BUF_SIZE];
char cmd_string[RX_BUF_SIZE];
char alt_data_str[ALT_BUF_SIZE];
static uint8_t imu_buf[20];

// 結構體
typedef struct {
	uint32_t t_ms;
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	int16_t mx, my, mz;
} imu_sample_t;

typedef struct {
	float angle;
	float bias;
	float P[2][2];
	float Q_angle;
	float Q_bias;
	float R_meas;
} Kalman_t;

typedef struct {
	float roll;
	float pitch;
	float yaw;
	bool biasLocked;
} AHRS_State_t;

typedef struct {
	float xm_1[3];
	float xm_2[3];
	float e_integral[3];
	float prev_error[3];
	float Kp[3];
	float Ki[3];
	float Kd[3];
} MRAC_State_t;

// 全域變數
static volatile imu_sample_t g_latest;
static Kalman_t KF_Roll, KF_Pitch, KF_Yaw;
static AHRS_State_t current_att;
static MRAC_State_t mrac_state;

// 物理參數
#define DRONE_MASS_KG   2.0f
#define GRAVITY_ACCEL   9.81f
#define MAX_THRUST_N    29.42f
#define MIN_THRUST_N    0.0f
#define ACC_LSB_PER_G   16384.0f
#define GYR_LSB_PER_DPS 16.4f

const float IX = 0.0095f;
const float IY = 0.0095f;
const float IZ = 0.0190f;

// 馬達參數
const float MOTOR_M[4] = { 964.308f, 958.89f, 986.08f, 959.01f };
const float MOTOR_C[4] = { -21.41f, -212.51f, -217.69f, -210.12f };

// 混合矩陣參數
const float MIX_A = 0.25f;
const float MIX_B = 0.25f / 0.13505f;
const float MIX_C = 0.25f / 0.08257f;
const float MIX_D = 0.25f / 0.019677f;

// 控制變數
char modetype[15];
float g_rc_roll_stick = 0.0f;
float g_rc_pitch_stick = 0.0f;
float g_rc_throttle_stick = 0.0f;

float g_est_vx = 0.0f, g_est_vy = 0.0f; // 速度估測

volatile int32_t distance_mm = 0;
float estimated_z = 0.0f, estimated_vz = 0.0f;
float target_vz = 0.0f, current_lock_height = 0.0f;

float g_cmd_phi_rad = 0.0f, g_cmd_theta_rad = 0.0f;
float target_roll = 0.0f, target_pitch = 0.0f, target_yaw = 0.0f;
float g_target_rate[3];
float g_u_vec[3];

// 濾波變數
static float last_gx = 0, last_gy = 0, last_gz = 0;
static float gx_sum = 0, gy_sum = 0, gz_sum = 0;
static float b_gx = 0, b_gy = 0, b_gz = 0;
static int bias_counter = 0;
static float roll_offset = 0, pitch_offset = 0, yaw_offset = 0;

// MRAC 參數
const float MRAC_WN[3] = { 10.0f, 5.0f, 5.0f };
const float MRAC_ZETA[3] = { 0.707f, 0.707f, 0.707f };
const float GAM_P[3] = { 0.01f, 0.01f, 0.01f };
const float GAM_I[3] = { 0.01f, 0.01f, 0.01f };
const float GAM_D[3] = { 0.001f, 0.001f, 0.001f };
const float KP_INIT[3] = {80.0f, 80.0f, 10.0f};
const float KI_INIT[3] = {150.0f, 150.0f, 0.1f};
const float KD_INIT[3] = {0.2f, 0.2f, 0.05f};
const float KP_MAX[3] = { 10000.0f, 10000.0f, 10000.0f };
const float KP_MIN[3] = { 0.01f, 0.01f, 0.01f };
const float KI_MAX[3] = { 10000.0f, 10000.0f, 10000.0f };
const float KI_MIN[3] = { 0.0f, 0.0f, 0.0f };
const float KD_MAX[3] = { 1000.0f, 1000.0f, 500.0f };
const float KD_MIN[3] = { 0.0f, 0.0f, 0.0f };

// 高度參數
#define ALT_DEADZONE 0.02f
#define MAX_CLIMB_RATE 2.0f
#define KP_Z_OUTER 1.0f
#define VZ_KP 2.0f
#define VZ_KI 1.0f
#define VZ_KD 0.0f

// Handles
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static float wrapTo180(float angle) {
	while (angle > 180.0f)
		angle -= 360.0f;
	while (angle < -180.0f)
		angle += 360.0f;
	return angle;
}
float fconstrain(float val, float min, float max) {
	if (val > max)
		return max;
	if (val < min)
		return min;
	return val;
}
float LowPassFilter(float new_val, float prev_val, float dt, float cutoff_freq) {
	float rc = 1.0f / (6.2831853f * cutoff_freq);
	float alpha = dt / (rc + dt);
	return prev_val + alpha * (new_val - prev_val);
}

// 初始化函式
void PCA9685_Init(I2C_HandleTypeDef *hi2c) {
	uint8_t d[2];
	d[0] = MODE1;
	d[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c, PCA9685_ADDR, d, 2, 100);
	HAL_Delay(10);
	d[0] = MODE1;
	d[1] = 0x10;
	HAL_I2C_Master_Transmit(hi2c, PCA9685_ADDR, d, 2, 100);
	d[0] = PRESCALE;
	d[1] = 121;
	HAL_I2C_Master_Transmit(hi2c, PCA9685_ADDR, d, 2, 100);
	d[0] = MODE1;
	d[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c, PCA9685_ADDR, d, 2, 100);
	HAL_Delay(5);
	d[0] = MODE1;
	d[1] = 0x20;
	HAL_I2C_Master_Transmit(hi2c, PCA9685_ADDR, d, 2, 100);
}
void PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t on,
		uint16_t off) {
	uint8_t data[5];
	data[0] = LED0_ON_L + 4 * channel;
	data[1] = on & 0xFF;
	data[2] = on >> 8;
	data[3] = off & 0xFF;
	data[4] = off >> 8;
	HAL_I2C_Master_Transmit(hi2c, PCA9685_ADDR, data, 5, HAL_MAX_DELAY);
}
void PCA9685_SetServoAngle(I2C_HandleTypeDef *hi2c, uint8_t channel,
		float angle) {
	uint16_t pulse = 205 + (uint16_t) (angle / 180.0f * 205.0f);
	PCA9685_SetPWM(hi2c, channel, 0, pulse);
}
void ESC_SetPWM(uint32_t Channel, uint16_t us) {
	__HAL_TIM_SET_COMPARE(&htim3, Channel, us);
}
void ESC_Calibrate(void) {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	ESC_SetPWM(TIM_CHANNEL_1, 2000);
	ESC_SetPWM(TIM_CHANNEL_2, 2000);
	ESC_SetPWM(TIM_CHANNEL_3, 2000);
	ESC_SetPWM(TIM_CHANNEL_4, 2000);
	HAL_Delay(3000);
	ESC_SetPWM(TIM_CHANNEL_1, 1000);
	ESC_SetPWM(TIM_CHANNEL_2, 1000);
	ESC_SetPWM(TIM_CHANNEL_3, 1000);
	ESC_SetPWM(TIM_CHANNEL_4, 1000);
	HAL_Delay(3000);
}
void BMX160_EnableSensors(I2C_HandleTypeDef *hi2c) {
	uint8_t d[2];
	d[0] = 0x7E;
	d[1] = 0xB6;
	HAL_I2C_Master_Transmit(hi2c, BMX160_ADDR, d, 2, 100);
	HAL_Delay(100);
	d[0] = 0x7E;
	d[1] = 0x11;
	HAL_I2C_Master_Transmit(hi2c, BMX160_ADDR, d, 2, 100);
	HAL_Delay(50);
	d[0] = 0x7E;
	d[1] = 0x15;
	HAL_I2C_Master_Transmit(hi2c, BMX160_ADDR, d, 2, 100);
	HAL_Delay(50);
	d[0] = 0x7E;
	d[1] = 0x19;
	HAL_I2C_Master_Transmit(hi2c, BMX160_ADDR, d, 2, 100);
	HAL_Delay(10);
	d[0] = 0x44;
	d[1] = 0x05;
	HAL_I2C_Master_Transmit(hi2c, BMX160_ADDR, d, 2, 100);
	d[0] = 0x4C;
	d[1] = 0x80;
	HAL_I2C_Master_Transmit(hi2c, BMX160_ADDR, d, 2, 100);
	d[0] = 0x4F;
	d[1] = 0x01;
	HAL_I2C_Master_Transmit(hi2c, BMX160_ADDR, d, 2, 100);
	d[0] = 0x4E;
	d[1] = 0x4B;
	HAL_I2C_Master_Transmit(hi2c, BMX160_ADDR, d, 2, 100);
	d[0] = 0x4F;
	d[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c, BMX160_ADDR, d, 2, 100);
	d[0] = 0x4E;
	d[1] = 0x4C;
	HAL_I2C_Master_Transmit(hi2c, BMX160_ADDR, d, 2, 100);
	d[0] = 0x4E;
	d[1] = 0x42;
	HAL_I2C_Master_Transmit(hi2c, BMX160_ADDR, d, 2, 100);
	d[0] = 0x4C;
	d[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c, BMX160_ADDR, d, 2, 100);
	HAL_Delay(50);
}
void MotorLeft_SetSpeed(uint8_t speed_percent) {
	if (speed_percent > 100)
		speed_percent = 100;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed_percent * 10);
}
void MotorRight_SetSpeed(uint8_t speed_percent) {
	if (speed_percent > 100)
		speed_percent = 100;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed_percent * 10);
}
void Motor_SetDirection(uint8_t dir, int velocity) {
	MotorLeft_SetSpeed(velocity);
	MotorRight_SetSpeed(velocity);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (dir == 3 || dir == 4));
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (dir == 1 || dir == 2));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, (dir == 1 || dir == 2));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, (dir == 3 || dir == 4));
	if (dir == 0) { // Stop
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 0);
	}
}

// 控制邏輯
void Kalman_Init(Kalman_t *kf, float q_ang, float q_bias, float r) {
	kf->angle = 0;
	kf->bias = 0;
	kf->Q_angle = q_ang;
	kf->Q_bias = q_bias;
	kf->R_meas = r;
	kf->P[0][0] = 0.1f;
	kf->P[0][1] = 0;
	kf->P[1][0] = 0;
	kf->P[1][1] = 0.1f;
}
void MRAC_Init(void) {
	for (int i = 0; i < 3; i++) {
		mrac_state.xm_1[i] = 0;
		mrac_state.xm_2[i] = 0;
		mrac_state.e_integral[i] = 0;
		mrac_state.prev_error[i] = 0;
		mrac_state.Kp[i] = KP_INIT[i];
		mrac_state.Ki[i] = KI_INIT[i];
		mrac_state.Kd[i] = KD_INIT[i];
	}
}
void Estimate_Horizontal_Velocity(int16_t ax_raw, int16_t ay_raw,
		float roll_deg, float pitch_deg, float dt) {
	float ax_g = ax_raw / ACC_LSB_PER_G;
	float ay_g = ay_raw / ACC_LSB_PER_G;
	float rad_roll = roll_deg * 0.0174533f;
	float rad_pitch = pitch_deg * 0.0174533f;
	float acc_motion_x = (ax_g - (-sinf(rad_pitch))) * 9.81f;
	float acc_motion_y = (ay_g - (sinf(rad_roll))) * 9.81f;
	if (fabsf(acc_motion_x) < 0.2f)
		acc_motion_x = 0;
	if (fabsf(acc_motion_y) < 0.2f)
		acc_motion_y = 0;
	g_est_vx += acc_motion_x * dt;
	g_est_vy += acc_motion_y * dt;
	g_est_vx *= 0.99f;
	g_est_vy *= 0.99f;
}
void Kalman_Step(Kalman_t *kf, float gyro_rate, float meas_angle, float dt,
bool useMeas) {
	float rate = gyro_rate - kf->bias;
	kf->angle += rate * dt;
	kf->angle = wrapTo180(kf->angle);
	float P00 = kf->P[0][0], P01 = kf->P[0][1], P10 = kf->P[1][0], P11 =
			kf->P[1][1];
	kf->P[0][0] = P00 - dt * (P10 + P01) + dt * dt * P11 + kf->Q_angle;
	kf->P[0][1] = P01 - dt * P11;
	kf->P[1][0] = P10 - dt * P11;
	kf->P[1][1] = P11 + kf->Q_bias;
	if (useMeas) {
		float y = wrapTo180(meas_angle - kf->angle);
		float S = kf->P[0][0] + kf->R_meas;
		float K0 = kf->P[0][0] / S, K1 = kf->P[1][0] / S;
		kf->angle += K0 * y;
		kf->bias += K1 * y;
		kf->angle = wrapTo180(kf->angle);
		float P00_t = kf->P[0][0], P01_t = kf->P[0][1];
		kf->P[0][0] -= K0 * P00_t;
		kf->P[0][1] -= K0 * P01_t;
		kf->P[1][0] -= K1 * P00_t;
		kf->P[1][1] -= K1 * P01_t;
	}
}
void AHRS_Update(float ax_raw, float ay_raw, float az_raw, float gx_raw,
		float gy_raw, float gz_raw, float mx_raw, float my_raw, float mz_raw,
		float dt) {
	float ax = ax_raw / ACC_LSB_PER_G, ay = ay_raw / ACC_LSB_PER_G, az = az_raw
			/ ACC_LSB_PER_G;
	float gx = gx_raw / GYR_LSB_PER_DPS, gy = gy_raw / GYR_LSB_PER_DPS, gz =
			gz_raw / GYR_LSB_PER_DPS;
	if (!current_att.biasLocked) {
		gx_sum += gx;
		gy_sum += gy;
		gz_sum += gz;
		bias_counter++;
		if (bias_counter >= 300) {
			b_gx = gx_sum / bias_counter;
			b_gy = gy_sum / bias_counter;
			b_gz = gz_sum / bias_counter;
			current_att.biasLocked = true;
			KF_Roll.bias = b_gx;
			KF_Pitch.bias = b_gy;
			KF_Yaw.bias = b_gz;
		}
		return;
	}
	float gx_c = gx - b_gx, gy_c = gy - b_gy, gz_c = gz - b_gz;
	float roll_acc = atan2f(ay, az) * 57.29578f;
	float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
	bool useAcc = fabsf(sqrtf(ax * ax + ay * ay + az * az) - 1.0f) < 0.5f;
	Kalman_Step(&KF_Roll, gx_c, roll_acc, dt, useAcc);
	Kalman_Step(&KF_Pitch, gy_c, pitch_acc, dt, useAcc);
	KF_Yaw.angle += gz_c * dt;
	current_att.roll = wrapTo180(KF_Roll.angle - roll_offset);
	current_att.pitch = wrapTo180(KF_Pitch.angle - pitch_offset);
	current_att.yaw = wrapTo180(KF_Yaw.angle - yaw_offset);
}
void AHRS_SetOrigin(void) {
	roll_offset = KF_Roll.angle;
	pitch_offset = KF_Pitch.angle;
	yaw_offset = KF_Yaw.angle;
}

void XY_Logic_Controller_Update(float roll_stick, float pitch_stick, float vx,
		float vy) {
	float theta_cmd = 0, phi_cmd = 0;
	if (fabsf(pitch_stick) > 0.02f)
		theta_cmd = pitch_stick * 0.5f;
	else
		theta_cmd = (0.0f - vx) * 0.2f;

	if (fabsf(roll_stick) > 0.02f)
		phi_cmd = -1.0f * roll_stick * 0.5f;
	else
		phi_cmd = (0.0f - vy) * 0.2f * -1.0f;
	g_cmd_theta_rad = fconstrain(theta_cmd, -0.5f, 0.5f);
	g_cmd_phi_rad = fconstrain(phi_cmd, -0.5f, 0.5f);
}

void MRAC_Compute(float cmd_roll, float cmd_pitch, float cmd_yaw, float gx,
		float gy, float gz, float dt) {
	float cmd[3] = { cmd_roll, cmd_pitch, cmd_yaw };
	float act[3] = { gx, gy, gz };
	for (int i = 0; i < 3; i++) {
		float u_model = MRAC_WN[i] * MRAC_WN[i] * cmd[i];
		float dxm2 = -2.0f * MRAC_ZETA[i] * MRAC_WN[i] * mrac_state.xm_2[i]
				- MRAC_WN[i] * MRAC_WN[i] * mrac_state.xm_1[i] + u_model;
		mrac_state.xm_2[i] += dxm2 * dt;
		mrac_state.xm_1[i] += mrac_state.xm_2[i] * dt;
		float e_track = act[i] - mrac_state.xm_1[i];
		float e_ctrl = cmd[i] - act[i];
		mrac_state.e_integral[i] = fconstrain(
				mrac_state.e_integral[i] + e_ctrl * dt, -10, 10);
		float phi_d = (e_ctrl - mrac_state.prev_error[i]) / dt;
		mrac_state.prev_error[i] = e_ctrl;
		mrac_state.Kp[i] = fconstrain(
				mrac_state.Kp[i] - GAM_P[i] * e_track * e_ctrl * dt, KP_MIN[i],
				KP_MAX[i]);
		mrac_state.Ki[i] = fconstrain(
				mrac_state.Ki[i]
						- GAM_I[i] * e_track * mrac_state.e_integral[i] * dt,
				KI_MIN[i], KI_MAX[i]);
		mrac_state.Kd[i] = fconstrain(
				mrac_state.Kd[i] - GAM_D[i] * e_track * phi_d * dt, KD_MIN[i],
				KD_MAX[i]);
		g_u_vec[i] = mrac_state.Kp[i] * e_ctrl
				+ mrac_state.Ki[i] * mrac_state.e_integral[i]
				+ mrac_state.Kd[i] * phi_d;
	}
}

void Height_estimator(int16_t acc_z_raw, int32_t baro_mm, float phi,
		float theta, float dt) {
	float acc_z = ((float) acc_z_raw / ACC_LSB_PER_G) * 9.81f;
	float alt_m = (float) baro_mm / 1000.0f;
	float acc_earth = (acc_z * cosf(phi * 0.017f) * cosf(theta * 0.017f))
			- 9.81f;
	float z_pred = estimated_z + estimated_vz * dt;
	float vz_pred = estimated_vz + acc_earth * dt;
	float z_err = alt_m - z_pred;
	estimated_z = z_pred + 2.0f * z_err * dt;
	estimated_vz = vz_pred + 0.2f * z_err * dt;
}

void Alt_Logic(float stick, float dt) {
	if (fabsf(stick) > ALT_DEADZONE) {
		target_vz = stick * MAX_CLIMB_RATE;
		current_lock_height = estimated_z;
	} else {
		float err = current_lock_height - estimated_z;
		target_vz = fconstrain(err * KP_Z_OUTER, -MAX_CLIMB_RATE,
		MAX_CLIMB_RATE);
	}
}

float Vel_PID(float target, float current, float dt) {
	float err = target - current;
	vz_pid_integ = fconstrain(vz_pid_integ + VZ_KI * err * dt, -10, 10);
	float d = (err - vz_pid_prev_err) / dt;
	vz_pid_prev_err = err;
	float acc_cmd = VZ_KP * err + vz_pid_integ + VZ_KD * d;
	float force = acc_cmd * DRONE_MASS_KG + DRONE_MASS_KG * 9.81f;
	return fconstrain(force, MIN_THRUST_N, MAX_THRUST_N);
}

void Control_Mixer_Advanced(float U1, float U2, float U3, float U4) {
	float T[4];
	T[0] = MIX_A * U1 + MIX_B * U2 - MIX_C * U3 + MIX_D * U4;
	T[1] = MIX_A * U1 - MIX_B * U2 + MIX_C * U3 + MIX_D * U4;
	T[2] = MIX_A * U1 - MIX_B * U2 - MIX_C * U3 - MIX_D * U4;
	T[3] = MIX_A * U1 + MIX_B * U2 + MIX_C * U3 - MIX_D * U4;

	float max_t = T[0];
	for (int i = 1; i < 4; i++)
		if (T[i] > max_t)
			max_t = T[i];
	float excess = max_t - MAX_THRUST_N;
	if (excess > 0)
		for (int i = 0; i < 4; i++)
			T[i] -= excess;
	for (int i = 0; i < 4; i++)
		T[i] = fconstrain(T[i], 0, MAX_THRUST_N);

	uint16_t pwm[4];
	for (int i = 0; i < 4; i++) {
		float g_force = (T[i] / 9.81f) * 1000.0f;
		float ratio =
				(MOTOR_M[i] != 0) ? (g_force - MOTOR_C[i]) / MOTOR_M[i] : 0;
		pwm[i] = (uint16_t) (1000 + fconstrain(ratio, 0, 1) * 1000);
	}
	ESC_SetPWM(TIM_CHANNEL_1, pwm[0]);
	ESC_SetPWM(TIM_CHANNEL_2, pwm[1]);
	ESC_SetPWM(TIM_CHANNEL_3, pwm[2]);
	ESC_SetPWM(TIM_CHANNEL_4, pwm[3]);
	char dbg[64];
	    int len = snprintf(dbg, sizeof(dbg),
	                       "PWM: %u %u %u %u\r\n",
	                       (unsigned)pwm[0],
	                       (unsigned)pwm[1],
	                       (unsigned)pwm[2],
	                       (unsigned)pwm[3]);
	    HAL_UART_Transmit(&huart1, (uint8_t*)dbg, len, 50);
}
/* USER CODE END 0 */

int main(void) {
	/* USER CODE BEGIN 1 */
	Kalman_Init(&KF_Roll, 0.001f, 0.003f, 0.25f);
	Kalman_Init(&KF_Pitch, 0.001f, 0.003f, 0.25f);
	Kalman_Init(&KF_Yaw, 0.001f, 0.003f, 4.0f);
	current_att.biasLocked = false;
	/* USER CODE END 1 */

	/* MCU Configuration */
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM4_Init();
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	MX_I2C2_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();

	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	PCA9685_Init(&hi2c1);
	MRAC_Init();
	HAL_Delay(100);
	BMX160_EnableSensors(&hi2c2);
	ESC_Calibrate();

	// 啟動 DMA 接收
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUF_SIZE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_alt_buffer, ALT_BUF_SIZE);

	HAL_TIM_Base_Start_IT(&htim4); // 開始 IMU 採樣
	strcpy(modetype, "stop");
	/* USER CODE END 2 */

	/* USER CODE BEGIN WHILE */
	uint32_t last_led_time = 0;
	uint32_t last_telemetry_time = 0;
	uint32_t last_ahrs_tick = 0;

	while (1) {
		// 指令處理
		if (cmd_received_flag) {
			cmd_received_flag = 0;
			last_cmd_time = HAL_GetTick();


			// 先處理STOP 指令
			if (strstr(cmd_string, "fly_stop")
					|| strcmp(cmd_string, "stop") == 0) {

				// 離開飛行模式
				strcpy(modetype, "stop");
				ESC_SetPWM(TIM_CHANNEL_1, 1000);
				ESC_SetPWM(TIM_CHANNEL_2, 1000);
				ESC_SetPWM(TIM_CHANNEL_3, 1000);
				ESC_SetPWM(TIM_CHANNEL_4, 1000);

				// 清搖桿
				g_rc_roll_stick = 0.0f;
				g_rc_pitch_stick = 0.0f;
				g_rc_throttle_stick = 0.0f;

				// 重置 MRAC 狀態
				MRAC_Init();

				// 高度估測重抓基準
				vz_pid_integ = 0.0f;
				vz_pid_prev_err = 0.0f;
				estimated_vz = 0.0f;
				estimated_z = (float) distance_mm / 1000.0f;
				current_lock_height = estimated_z;

			}

			// AHRS 歸零
			else if (cmd_string[0] == 'z') {
				AHRS_SetOrigin();
			}

			// 飛行指令
			else if (strncmp(cmd_string, "fly", 3) == 0) {

				// 展開機構
				if (strstr(cmd_string, "fly_mode")) {
					PCA9685_SetServoAngle(&hi2c1, 0, 0);
					PCA9685_SetServoAngle(&hi2c1, 1, 33);
					HAL_Delay(1000);
					PCA9685_SetServoAngle(&hi2c1, 2, 173);
					PCA9685_SetServoAngle(&hi2c1, 4, 30);
					PCA9685_SetServoAngle(&hi2c1, 3, 160);
					PCA9685_SetServoAngle(&hi2c1, 5, 10);
				}

				// 水平移動
				if (strstr(cmd_string, "fly_forward")) {
					g_rc_pitch_stick = 0.4f;
					strcpy(modetype, "fly");
				} else if (strstr(cmd_string, "fly_backward")) {
					g_rc_pitch_stick = -0.4f;
					strcpy(modetype, "fly");
				} else if (strstr(cmd_string, "fly_right")) {
					g_rc_roll_stick = -0.4f;
					strcpy(modetype, "fly");
				} else if (strstr(cmd_string, "fly_left")) {
					g_rc_roll_stick = 0.4f;
					strcpy(modetype, "fly");
				}

				// 上下
				else if (strstr(cmd_string, "fly_up")) {
					g_rc_throttle_stick = 0.3f;
					strcpy(modetype, "fly");
				} else if (strstr(cmd_string, "fly_down")) {
					g_rc_throttle_stick = -0.3f;
					strcpy(modetype, "fly");
				}

				// 懸停 / 緩降
				else if (strstr(cmd_string, "fly_stay")) {
					g_rc_pitch_stick = 0.0f;
					g_rc_roll_stick = 0.0f;
					g_rc_throttle_stick = 0.0f;
					strcpy(modetype, "fly");
				} else if (strstr(cmd_string, "fly_ground")) {
					g_rc_pitch_stick = 0.0f;
					g_rc_roll_stick = 0.0f;
					g_rc_throttle_stick = -0.1f;
					strcpy(modetype, "fly");
				}
			}

			// 清空緩衝
			memset(cmd_string, 0, RX_BUF_SIZE);
		}

		// 失控保護
		if (HAL_GetTick() - last_cmd_time > 500) {
			if (strncmp(modetype, "fly", 3) == 0) {
				g_rc_roll_stick = 0;
				g_rc_pitch_stick = 0;
				g_rc_throttle_stick = 0;
				strcpy(modetype, "fly");
			}
		}

		// 飛控主迴路
		if (data_ready_flag) {
			data_ready_flag = 0;
			imu_sample_t s;
			__disable_irq();
			s = g_latest;
			__enable_irq();

			uint32_t now = HAL_GetTick();
			float dt = (now - last_ahrs_tick) / 1000.0f;
			last_ahrs_tick = now;
			if (dt <= 0)
				dt = 0.001f;

			AHRS_Update((float) s.ax, (float) s.ay, (float) s.az, (float) s.gx,
					(float) s.gy, (float) s.gz, (float) s.mx, (float) s.my,
					(float) s.mz, dt);

			if (strncmp(modetype, "fly", 3) == 0 && current_att.biasLocked) {
				// 速度估測
				Estimate_Horizontal_Velocity(s.ax, s.ay, current_att.roll,
						current_att.pitch, dt);

				// 邏輯控制
				XY_Logic_Controller_Update(g_rc_roll_stick, g_rc_pitch_stick,
						g_est_vx, g_est_vy);
				target_roll = g_cmd_phi_rad * 57.29578f;
				float pitch_trim = 3.0f;
				target_pitch = (g_cmd_theta_rad * 57.29578f) + pitch_trim;
				target_yaw = 0.0f;

				// 姿態控制
				float err_roll = target_roll - current_att.roll;
				float err_pitch = target_pitch - current_att.pitch;
				float err_yaw = target_yaw - current_att.yaw;
				g_target_rate[0] = err_roll * 0.01745f;
				g_target_rate[1] = err_pitch * 0.01745f;
				g_target_rate[2] = err_yaw * 0.01745f;

				// MRAC
				float gyr_x = LowPassFilter((s.gx/GYR_LSB_PER_DPS)*0.01745f, last_gx, dt, 8.0f);
				float gyr_y = LowPassFilter((s.gy/GYR_LSB_PER_DPS)*0.01745f, last_gy, dt, 8.0f);
				float gyr_z = LowPassFilter((s.gz/GYR_LSB_PER_DPS)*0.01745f, last_gz, dt, 8.0f);
				last_gx = gyr_x;
				last_gy = gyr_y;
				last_gz = gyr_z;

				MRAC_Compute(g_target_rate[0], g_target_rate[1],
						g_target_rate[2], gyr_x, gyr_y, gyr_z, dt);

				// 高度控制
				Height_estimator(s.az, distance_mm, current_att.roll,
						current_att.pitch, dt);
				Alt_Logic(g_rc_throttle_stick, dt);
				float U1 = Vel_PID(target_vz, estimated_vz, dt);

				// 翻機保護
				if (fabsf(current_att.roll) > 60
						|| fabsf(current_att.pitch) > 60) {
					U1 = 0;
					MRAC_Init();
				}

				// 輸出
				Control_Mixer_Advanced(U1, g_u_vec[0] * IX, g_u_vec[1] * IY,
						g_u_vec[2] * IZ);
			} else {
				// 非 Fly 模式強制停轉
				if (strncmp(modetype, "fly", 3) != 0) {
					ESC_SetPWM(TIM_CHANNEL_1, 1000);
					ESC_SetPWM(TIM_CHANNEL_2, 1000);
					ESC_SetPWM(TIM_CHANNEL_3, 1000);
					ESC_SetPWM(TIM_CHANNEL_4, 1000);
				}
			}

			// Telemetry
			if (now - last_telemetry_time > 200) {
				last_telemetry_time = now;
			}
		}

		if (HAL_GetTick() - last_led_time > 100) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			last_led_time = HAL_GetTick();
		}
	}
	/* USER CODE END WHILE */
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		Error_Handler();
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
		Error_Handler();
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {
		if (!i2c2_busy) {
			i2c2_busy = 1;
			HAL_I2C_Mem_Read_IT(&hi2c2,
			BMX160_ADDR, 0x04,
					I2C_MEMADD_SIZE_8BIT, imu_buf, 20);
		}
	}
}

// 中斷 Callback
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	if (huart->Instance == USART1) { // 藍牙
		if (Size > RX_BUF_SIZE - 1)
			Size = RX_BUF_SIZE - 1;
		memcpy(cmd_string, rx_buffer, Size);
		cmd_string[Size] = '\0';
		memset(rx_buffer, 0, RX_BUF_SIZE);
		cmd_received_flag = 1;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUF_SIZE);
	} else if (huart->Instance == USART2) { // 高度計
		if (Size > ALT_BUF_SIZE - 1)
			Size = ALT_BUF_SIZE - 1;
		memcpy(alt_data_str, rx_alt_buffer, Size);
		alt_data_str[Size] = '\0';
		memset(rx_alt_buffer, 0, ALT_BUF_SIZE);
		char *ptr = strchr(alt_data_str, 'd');
		if (ptr && strlen(ptr) > 2) {
			int val = 0;
			if (sscanf(ptr, "d:%d", &val) == 1
					|| sscanf(ptr, "d: %d", &val) == 1)
				distance_mm = val;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_alt_buffer, ALT_BUF_SIZE);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUF_SIZE);
	} else if (huart->Instance == USART2) {
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_alt_buffer, ALT_BUF_SIZE);
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c2) {
		imu_sample_t s;
		s.t_ms = HAL_GetTick();
		s.mx = (int16_t) ((imu_buf[1] << 8) | imu_buf[0]);
		s.my = (int16_t) ((imu_buf[3] << 8) | imu_buf[2]);
		s.mz = (int16_t) ((imu_buf[5] << 8) | imu_buf[4]);
		s.gx = (int16_t) ((imu_buf[9] << 8) | imu_buf[8]);
		s.gy = (int16_t) ((imu_buf[11] << 8) | imu_buf[10]);
		s.gz = (int16_t) ((imu_buf[13] << 8) | imu_buf[12]);
		s.ax = (int16_t) ((imu_buf[15] << 8) | imu_buf[14]);
		s.ay = (int16_t) ((imu_buf[17] << 8) | imu_buf[16]);
		s.az = (int16_t) ((imu_buf[19] << 8) | imu_buf[18]);
		g_latest = s;
		data_ready_flag = 1;
		i2c2_busy = 0;
	}
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c2)
		i2c2_busy = 0;
}
void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
}
/* USER CODE END 4 */
