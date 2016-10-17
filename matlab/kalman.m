function  kalman()

clear all;
close all;
clc;

% Importing data 
gyr = importdata('gyr_kalman3_3.txt');
row=length(gyr(:,1))
column=length(gyr(1,:))

start_row=1
end_row=row

%plot([1:end_row], gyr(start_row:end_row,1),'-r', [1:end_row], gyr(start_row:end_row,2),'-g', [1:end_row], gyr(start_row:end_row,3),'-b');
plot([1:end_row], gyr(start_row:end_row,1),'-r', [1:end_row], gyr(start_row:end_row,2),'-g');
axis([0 end_row -3 3]);
%axis([0 end_row -300 300]);
title('Õ”¬›“«kalman¬À≤®');
xlabel('t [10ms]');  %x÷·
ylabel('Ω«ÀŸ∂» [rad/s]');%y÷·
return 
