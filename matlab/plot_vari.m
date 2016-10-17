function  plot_vari()
%配置这个
max_tau = 60;
matrix_tau=1:max_tau;
average_allan_vari=zeros(max_tau,3);

%不要求所有的nS都有数据
for tau = 1:1:max_tau
        average_allan_vari(tau,1:3)=calcu_average_vari(int2str(tau));
        if isempty(average_allan_vari) == 0
                disp([int2str(tau),'s average allan vari is:']);
                disp(average_allan_vari(tau,:));
        end
end

%average_allan_vari
%将平均值的曲线画出来
%average_allan_vari
plot(matrix_tau, average_allan_vari(1:max_tau,1),'-r',matrix_tau, average_allan_vari(1:max_tau,2),'-g',matrix_tau, average_allan_vari(1:max_tau,3),'-b');
axis([0 max_tau 0.0 2*10^-7]);
title('Allan方差与均值采样时间关系图');
xlabel('t [s]');  %x轴
ylabel('Allan方差 [(rad/s)^2]');%y轴
return 
