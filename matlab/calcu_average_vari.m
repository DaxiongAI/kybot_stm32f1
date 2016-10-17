%nS指多少S间隔的均值，如1s,2s,3s
%将allan方差的均值返回
function  average_allan_vari=calcu_average_vari(nS) 
TAU_M = str2num(nS);
%gyr_data = '';
files=dir(strcat(nS, 's/'));
[file_num unused]=size(files);
dataset=0;
for i = 1:1:file_num
        if files(i).isdir==0
                %files(i).name
                %只有txt文件才是陀螺仪的数据文件
                is_data=strfind(files(i).name, '.txt');
                if is_data>0
                        dataset = dataset+1;
                end
        end

end

%dataset
allan_vari=zeros(dataset,3);
output_file=strcat(nS, 's/', nS, 's_allan_vari.xls');
%清空
%xlswrite(output_file, []);
is_exist=exist(output_file, 'file');
if is_exist>0 
        %定义一个空矩阵
        %matrix=[];
        delete(output_file);
%else
        %[matrix]= xlsread(output_file);
end
%[row column]= size(matrix);
for i = 1:1:dataset
        file=strcat(nS, 's/', int2str(i) , '.txt');
        %gyr_data=strcat ('gyr_', int2str(i) , 's')
        data_matrix = load(file);
        allan_vari(i,1:3)=calcu_vari(nS, data_matrix);
        if isempty(allan_vari) == 0
                disp([int2str(i),').',int2str(TAU_M),'s allan vari is:']);
                disp(allan_vari(i,:));
        end
        %t=importdata('gyr_data.xls');
        %[row column]= size(t.Sheet1);

        %dst_row=sprintf('A%d', row+1)
        %[row1 column1]=size(allan_vari);
        %data=nan(max([aa row])+1,max([bb column]));
        %data=zeros(row+1, 3);
        %if row>0
        %data(1:row, 1:3)=matrix;
        %end
        %data(row+1:row+1, 1:3)=allan_vari(i,1:3);
        %同名文件再写会先删除再写入
        %xlswrite('gyr_data.xls', allan_vari, dst_row);
end
average_allan_vari=zeros(1,3);
if isempty(allan_vari) == 1
        return
end
xlswrite(output_file, allan_vari);

sum_x = 0.0;
sum_y = 0.0;
sum_z = 0.0;
for i = 1:1:dataset
        sum_x = sum_x + allan_vari(i, 1);
        sum_y = sum_y + allan_vari(i, 2);
        sum_z = sum_z + allan_vari(i, 3);
end
average_allan_vari(1,1) = sum_x / dataset;
average_allan_vari(1,2) = sum_y / dataset;
average_allan_vari(1,3) = sum_z / dataset;
%average_allan_vari
return
%将所有方差的曲线画出来
plot([1:dataset], allan_vari(1:dataset,1),'-r',[1:dataset], allan_vari(1:dataset,2),'-g',[1:dataset], allan_vari(1:dataset,3),'-b');
axis([0 50 0.0 2*10^-7]);
title(strcat(nS, 's Allan方差数据图'));
xlabel('n');  %x轴
ylabel('Allan方差 [(rad/s)^2]');%y轴
%保留图形不被覆盖
figure;
return 
