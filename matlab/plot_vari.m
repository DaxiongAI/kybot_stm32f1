function  plot_vari()
%�������
max_tau = 60;
matrix_tau=1:max_tau;
average_allan_vari=zeros(max_tau,3);

%��Ҫ�����е�nS��������
for tau = 1:1:max_tau
        average_allan_vari(tau,1:3)=calcu_average_vari(int2str(tau));
        if isempty(average_allan_vari) == 0
                disp([int2str(tau),'s average allan vari is:']);
                disp(average_allan_vari(tau,:));
        end
end

%average_allan_vari
%��ƽ��ֵ�����߻�����
%average_allan_vari
plot(matrix_tau, average_allan_vari(1:max_tau,1),'-r',matrix_tau, average_allan_vari(1:max_tau,2),'-g',matrix_tau, average_allan_vari(1:max_tau,3),'-b');
axis([0 max_tau 0.0 2*10^-7]);
title('Allan�������ֵ����ʱ���ϵͼ');
xlabel('t [s]');  %x��
ylabel('Allan���� [(rad/s)^2]');%y��
return 
