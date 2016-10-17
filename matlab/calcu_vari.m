%nSָ����S����ľ�ֵ����1s,2s,3s
%��allan����ķ���
function  allan_vari=calcu_vari(nS, data_matrix) 
GYR_READ_FREQ = 100;
%������ٶ������ֵ������ǲ���ʱ�䣬��λs
%�޸ĸ�ֵ
TAU_M = str2num(nS);
%����ʱ�䣬10ms
T0 = 0.01;
%/*#define m       (TAU/T0)*/
K = 50;
%����������������1s����100��
M = (TAU_M*GYR_READ_FREQ);
N = (K*M);
%��������K����Ϊ����
%K = (N/M);
%��������ת������, mdps/digit
omega_tau = zeros(N, 3);
omega_M = zeros(K, 3);

for i = 1:1:N
        omega_tau(i, 1) = data_matrix(i, 1);
        omega_tau(i, 2) = data_matrix(i, 2);
        omega_tau(i, 3) = data_matrix(i, 3);
end
%omega_tau
for i = 1:1:K
        sum_x = 0;
        sum_y = 0;
        sum_z = 0;
        for j = 1 + (i - 1)*M : 1 : M + (i - 1)*M 
                sum_x = sum_x + omega_tau(j,1);
                sum_y = sum_y + omega_tau(j,2);
                sum_z = sum_z + omega_tau(j,3);
        end
        omega_M(i,1) = sum_x / M;
        omega_M(i,2) = sum_y / M;
        omega_M(i,3) = sum_z / M;
end
%omega_M
sum_x = 0;
sum_y = 0;
sum_z = 0;
for i = 1:1:K-1
        tmp_x = omega_M(i+1,1) - omega_M(i,1);
        tmp_x = tmp_x * tmp_x;
        sum_x = sum_x + tmp_x;

        tmp_y = omega_M(i+1,2) - omega_M(i,2);
        tmp_y = tmp_y * tmp_y;
        sum_y = sum_y + tmp_y;

        tmp_z = omega_M(i+1,3) - omega_M(i,3);
        tmp_z = tmp_z * tmp_z;
        sum_z = sum_z + tmp_z;
end

allan_vari1_x = sum_x / (2 * (K-1));
allan_vari1_y = sum_y / (2 * (K-1));
allan_vari1_z = sum_z / (2 * (K-1));
%rad/s
scale = 0.00875*3.14/180;
%(rad/s)^2
scale = scale * scale;
allan_vari = zeros(1,3);
%x
allan_vari(1) = allan_vari1_x * scale;
%y
allan_vari(2) = allan_vari1_y * scale;
%z
allan_vari(3) = allan_vari1_z * scale;

return 
