//{{{ include
#include "allan.h"
#include "log.h"
#include "workdata.h"
#include "file_id.h"

#define this_file_id file_id_allan
//}}}

// 计算角速度输出均值间隔，非采样时间，单位s
#define TAU_M     1
// 采样时间，10ms
#define T0      0.01
/*#define m       (TAU/T0)*/
// 采样的样本总数，1s采样100次
#define K       50
#define M       (TAU_M*GYR_READ_FREQ)
#define N       (K*M)

/*#define N       51175*/

// 陀螺仪旋转角速率, mdps/digit
/*int16_imu_t omega_tau[N];*/
/*int16_imu_t omega_M[K];*/
void output_acc_gyr(int16_imu_t *acc, int16_imu_t *gyr)
{
        static uint32_t ix = 0;
        if (ix == N) {
                log_info("insert angular velocity done");
                ix++;
                return; 
        }
        else if (ix > N) {
                return;
        }
        /*omega_tau[ix].x = gyr->x;*/
        /*omega_tau[ix].y = gyr->y;*/
        /*omega_tau[ix].z = gyr->z;*/
        ix++;
        //300是3s静置
#ifdef CALCU_ALLAN_VARI
        log_none("%d, %d, %d,\r\n", gyr->x, gyr->y, gyr->z);
#else
        log_none("%f, %d, %d, %d, %d, %d, %d,\r\n", (os_time_ticks-MAX_FILTER_CNT-STATIC_TIME*100)*0.01, acc->x, acc->y, acc->z, gyr->x, gyr->y, gyr->z);
#endif
}
void output_gyr(int16_imu_t *gyr)
{
        static uint32_t ix = 0;
        if (ix == N) {
                log_info("insert angular velocity done");
                ix++;
                return; 
        }
        else if (ix > N) {
                return;
        }
        /*omega_tau[ix].x = gyr->x;*/
        /*omega_tau[ix].y = gyr->y;*/
        /*omega_tau[ix].z = gyr->z;*/
        ix++;
        //300是3s静置
        log_none("%d, %d, %d,\r\n", gyr->x, gyr->y, gyr->z);
}
#if 0
void calculate_allan_vari(void)
{
        int32_t sum_x = 0;   
        int32_t sum_y = 0;   
        int32_t sum_z = 0;   

        int32_t tmp_x = 0;   
        int32_t tmp_y = 0;   
        int32_t tmp_z = 0;   

        for (int i = 0; i < K; i++) {
                sum_x = 0;
                sum_y = 0;
                sum_z = 0;
                for (int j = i * M; j < M; j++) {
                       sum_x += omega_tau[j].x; 
                       sum_y += omega_tau[j].y; 
                       sum_z += omega_tau[j].z; 
                }
                omega_M[i].x = sum_x / M;
                omega_M[i].y = sum_y / M;
                omega_M[i].z = sum_z / M;
        }

        sum_x = 0;
        sum_y = 0;
        sum_z = 0;

        for (int i = 0; i < K-1; i++) {
                tmp_x = omega_M[i+1].x - omega_M[i].x ;
                tmp_x *= tmp_x;
                sum_x += tmp_x;

                tmp_y = omega_M[i+1].y - omega_M[i].y ;
                tmp_y *= tmp_y;
                sum_y += tmp_y;

                tmp_z = omega_M[i+1].z - omega_M[i].z ;
                tmp_z *= tmp_z;
                sum_z += tmp_z;
        }
        int32_t allan_vari1_x = sum_x / (2 * (K-1));
        int32_t allan_vari1_y = sum_y / (2 * (K-1));
        int32_t allan_vari1_z = sum_z / (2 * (K-1));

        double allan_vari2_x = allan_vari1_x * 0.00875 * 0.00875;
        double allan_vari2_y = allan_vari1_y * 0.00875 * 0.00875;
        double allan_vari2_z = allan_vari1_z * 0.00875 * 0.00875;
        log_info("vari_x = %.12lf", allan_vari2_x);
        log_info("vari_y = %.12lf", allan_vari2_y);
        log_info("vari_z = %.12lf", allan_vari2_z);
}
#endif
