function result()
    clear;clc;
    close all;
    warning('off');
    tic;
    global vehi_speed beacon_period branchset A Linmap1m T ima2 locationjunction w_mini_slot  retrans_max  t_DFIS t_SIFS t_switch t_slot v_signal delay_max s_EM s_RTB s_CTB s_ACK rate_bit CW_max_DIFS CW_min_DIFS  n_cont_mini n_cont_CTB  density_EM CW_CTB isbeacon n_fail;
    %初始化一些必要参数
    R = 200;
    vehi_speed = 60*1000/3600;%vehi_speed表示为车速，60km/h，将km/h换成m/s的形式
    beacon_period = 100e-3;%beacon_period表示为信标更新周期，我们设为100ms
    CW_CTB = 2;%CTB争用初始窗口大小
    density_EM = 2;%竞争消息数目
    t_DFIS = 58e-6;
    t_SIFS = 32e-6;
    t_switch = 1e-6;
    t_slot = 13e-6;
    v_signal = 3e8;
    delay_max = 2e-6;%R/v_signal;最大传输延时
    s_EM = 500*8; %EM包的大小，单位Bytes
    s_RTB = 20*8;
    s_CTB = 14*8;
    s_ACK = 14*8;
    rate_bit = 18e6; %数据传输率，单位为bps
    w_mini_slot = 2*delay_max+t_switch;%这里可以看出mini_slot明显比slot短，mini_slot大概是5e-6，slot是13e-6
    CW_max_DIFS = 2^10;%mini_DFIS竞争窗最大cw数（详见contention），遵循载波侦听多路访问机制（CSMA/CD）的二进制指数退避算法原则，最大为2的10次方减去1，减去1的操作在jyc_contention函数中的t_backoff段做了
    CW_min_DIFS = 2;%初始化mini_DFIS竞争窗的大小，遵循载波侦听多路访问机制（CSMA/CD）的二进制指数退避算法原则
    n_cont_mini = 16; %mini_DFIS竞争冲突次数的上界，遵循载波侦听多路访问机制（CSMA/CD）的二进制指数退避算法原则，最大为16
    n_cont_CTB = 5; %mini_CTB竞争冲突次数的上界
    T = 10;%广播覆盖时延上限
    % bar_symbol = 1;障碍物标记，0代表不考虑障碍物情况，1代表考虑障碍物情况
    N_iter = [3;2];%两次迭代
    N_part = [3;4];%分区数，四分区为Log算法，三分区为3P3B算法
    A = 2;
    retrans_max = 5;%初始化最大重传次数为3次   
    n_fail = 0;%初始化争用失败次数为0（contention fail times are zeros）
    isbeacon_set = [0;1];%isbeacon参数集
    %初始化必要参数完毕    
    simu_num = 2000;%仿真次数设置为5000次
    density = [0.01:0.03:0.25];%车辆密度设置为0.005（即单车道两百米/一辆车）往0.25（即单车道四米/一辆车）以每次0.049（刚好递进五次）递进，这里的节点密度是单车道节点密度，单位车/米 
    code_time = zeros(1,simu_num);
    PDR = zeros(1,simu_num);
    average_time_exponent24 = zeros(1,length(density));
    average_time_exponent24_SE = zeros(1,length(density));
    average_PDR_exponent24 = zeros(1,length(density));
    average_PDR_exponent24_SE = zeros(1,length(density));
    average_time_3P3B = zeros(1,length(density));
    average_time_3P3B_SE = zeros(1,length(density));
    average_PDR_3P3B = zeros(1,length(density));
    average_PDR_3P3B_SE = zeros(1,length(density));
    average_time_beacon = zeros(1,length(density));
    average_time_beacon_SE = zeros(1,length(density));
    average_PDR_beacon = zeros(1,length(density));
    average_PDR_beacon_SE = zeros(1,length(density));
    average_maxhop_exponent24 = zeros(1,length(density));
    average_maxhop_exponent24_SE = zeros(1,length(density));
    average_maxhop_3P3B = zeros(1,length(density));
    average_maxhop_3P3B_SE = zeros(1,length(density));
    average_maxhop_beacon = zeros(1,length(density));
    average_maxhop_beacon_SE = zeros(1,length(density));
    
    %生成障碍物灰度二值化矩阵
    ima = imread('system_map_bar.jpg');%ima为图像矩阵（彩色）
    ima1 = rgb2gray(ima);%ima1为图像的灰度化矩阵
    ima2 = ima1;%初始化ima2和ima1同行同列同值
    %之后将ima2数组变成只有障碍物像素值（0）和非障碍物像素值（242）的二值化矩阵
    for i = 1:length(ima2(:,1))
        for j = 1:length(ima2(1,:))
            if ima2(i,j)<=220%找出ima2中的障碍物像素(障碍物像素值为149)坐标,我们设定像素值在145-155范围内皆是障碍物
                ima2(i,j) = 0;%对找出的障碍物像素坐标对应的像素值置0，即置黑色
            else
                ima2(i,j) = 242; %对非障碍物的坐标像素值置242，242即白色
            end
        end
    end
%     imshow(ima2)
%     hold on
    %障碍物灰度二值化矩阵生成完毕
    
    %将路口点信息存入locationjucntion数组中
    fdata1 = fopen('system_junction.txt','r');%fdata1是文件代号，fdata1=+N(N是正整数)：表示文件打开成功，文件代号是N. fdata1=-1 : 表示文件打开不成功。fileID在此次文件关闭前总是有效的。
    temp = fscanf(fdata1,'%e %e');%从打开的fdata1文件中以科学记数法的方式读取一行三个数（因为打开的文件是以一行三个数一共两行的方式存取数值的），一共读取两行六个数排成一列。
    locationjunction=ones(length(temp)/2,3);
    locationjunction(:,1)=1:length(temp)/2 ;%支路信息。1:序号；2，3：（x,y)
    locationjunction(:,2)=temp(1:2:length(temp)) ;
    locationjunction(:,3)=temp(2:2:length(temp)) ;%fdata1打开的文件中，第1和第4个数存在了locationjucntion数组的第一列中，第2和第5个数存在了第二列中，第3和第6个数存在了第三列中
    fclose(fdata1);
    %路口点信息存储完毕
    %初始化branchset元胞，将路段信息存入branchset元胞中
    branchset = cell(2,26);
    path = 'D:\matlab2018a\result for all\system_branch';
    txtfile = dir(fullfile(path,'*.txt'));
    for i =1:26
        name = fullfile(path,[txtfile(i).name]);
        fdata = fopen(name,'r');
        temp = fscanf(fdata,'%e %e');
        branch(:,1) = 1:length(temp)/2;
        branch(:,2) = temp(1:2:length(temp));
        branch(:,3) = temp(2:2:length(temp));
        fclose(fdata);
        branchset{1,i} = txtfile(i).name;
        branchset{2,i} = branch;
        branch = [];
    end
    %路段信息存储完毕
    %将障碍物信息存入locationbar数组中
    fdata1 = fopen('system_bar.txt','r');%
    temp = fscanf(fdata1,'%e %e %e');%读取障碍物点的信息
    locationbar = ones(length(temp)/2,2);%生成障碍物信息数组
    locationbar(:,1)=temp(1:2:length(temp));
    locationbar(:,2)=temp(2:2:length(temp));%将障碍物点的信息存入障碍物数组中
    fclose(fdata1);
    mainroad2 = branchset{2,2};
    Linmap1m=sum((mainroad2(7,2:3)-mainroad2(1,2:3)).^2).^0.5/147;
    
%     for i = 1:26 %测试一下，看locationmark点与ima2的障碍物图片相比，是否全在正确的位置上
%         for j = 1:length(branchset{2,i}(:,1))
%             plot(branchset{2,i}(j,2),branchset{2,i}(j,3),'o','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor',[0.7,0.7,0.7],'MarkerSize',4);
%             hold on
%         end
%     end  
 
    for i = 1:length(density)
        %准备生成车辆信息
        code_time_exponent24 = zeros(1,simu_num);
        PDR_exponent24 = zeros(1,simu_num);
        code_time_3P3B = zeros(1,simu_num);
        PDR_3P3B = zeros(1,simu_num);
        code_time_beacon = zeros(1,simu_num);
        PDR_beacon = zeros(1,simu_num);
        maxhop_exponent24 = zeros(1,simu_num);
        maxhop_3P3B = zeros(1,simu_num);
        maxhop_beacon = zeros(1,simu_num);
        for n = 1:simu_num
            [location_vehi,num_vehi]=generatetvehi(density(i),locationjunction,branchset,R,Linmap1m,locationbar);%按泊松分布，在道路上（包括支路上分布车辆）。(x,y)=(0,0)标示车辆无
            fdata1 = fopen('system_vehi.txt','w');
            fprintf(fdata1,'%d ',location_vehi(1:num_vehi,:));%车辆信息。1:序号；2，3：（x,y);4:在分割时，用于所在seg标示
            fprintf(fdata1,'\n');
            fclose(fdata1);
            for j = 1:length(isbeacon_set(:,1))
                isbeacon = isbeacon_set(j,1);%是否做基于beacon的中继节点选择运算,1为做beacon运算，0为做指数分区运算
                if isbeacon==0%表示做指数分区运算
                    for m = 1:length(N_part(:,1))
                        if N_part(m,1) == 4
                            [code_time_part,PDR_part,location_vehi,num_vehi,maxhop_part] = jyc_main(R,N_iter(m,1),N_part(m,1),location_vehi,num_vehi);
                            code_time_exponent24(n) = code_time_part;
                            PDR_exponent24(n) = PDR_part;
                            maxhop_exponent24(n) = maxhop_part;
%                             clf;
                        else                           
                            [code_time_part,PDR_part,location_vehi,num_vehi,maxhop_part] = jyc_main(R,N_iter(m,1),N_part(m,1),location_vehi,num_vehi);
                            code_time_3P3B(n) = code_time_part;
                            PDR_3P3B(n) = PDR_part;     
                            maxhop_3P3B(n) = maxhop_part;
%                             clf;
                        end
                    end
                else%表示做基于beacon运算                   
                    [code_time_part,PDR_part,location_vehi,num_vehi,maxhop_part] = jyc_main(R,2,4,location_vehi,num_vehi);%对信标算法而言，没有指数迭代分区来确定中继节点，故N_iter,N_part参数没必要，我们这里输入2,4只是为了确保有数值占输入参数的位置
                    code_time_beacon(n) = code_time_part;
                    PDR_beacon(n) = PDR_part;
                    maxhop_beacon(n) = maxhop_part;
%                     clf;
                end
            end
        end

        average_time_exponent24(i) = mean(code_time_exponent24(code_time_exponent24<100));%将延时存储数组中，由于争用次数达到最大争用次数导致的消息传输失败，进而延时无限大的情况排除掉
        average_time_exponent24_SE(i) = 1.96*std(code_time_exponent24(code_time_exponent24<100))/(length(code_time_exponent24(code_time_exponent24<100))).^0.5;
        average_PDR_exponent24(i) = mean(PDR_exponent24);
        average_PDR_exponent24_SE(i) = 1.96*std(PDR_exponent24)/(length(PDR_exponent24)).^0.5;
        average_maxhop_exponent24(i) = mean(maxhop_exponent24(maxhop_exponent24<20));
        average_maxhop_exponent24_SE(i) = 1.96*std(maxhop_exponent24(maxhop_exponent24<20))/(length(maxhop_exponent24(maxhop_exponent24<20))).^0.5;
        
        average_time_3P3B(i) = mean(code_time_3P3B(code_time_3P3B<100));%将延时存储数组中，由于争用次数达到最大争用次数导致的消息传输失败，进而延时无限大的情况排除掉
        average_time_3P3B_SE(i) = 1.96*std(code_time_3P3B(code_time_3P3B<100))/(length(code_time_3P3B(code_time_3P3B<100))).^0.5;
        average_PDR_3P3B(i) = mean(PDR_3P3B);  
        average_PDR_3P3B_SE(i) = 1.96*std(PDR_3P3B)/(length(PDR_3P3B)).^0.5;
        average_maxhop_3P3B(i) = mean(maxhop_3P3B(maxhop_3P3B<20));
        average_maxhop_3P3B_SE(i) = 1.96*std(maxhop_3P3B(maxhop_3P3B<20))/(length(maxhop_3P3B(maxhop_3P3B<20))).^0.5;
        
        average_time_beacon(i) = mean(code_time_beacon(code_time_beacon<100));%将延时存储数组中，由于争用次数达到最大争用次数导致的消息传输失败，进而延时无限大的情况排除掉
        average_time_beacon_SE(i) = 1.96*std(code_time_beacon(code_time_beacon<100))/(length(code_time_beacon(code_time_beacon<100))).^0.5;
        average_PDR_beacon(i) = mean(PDR_beacon);
        average_PDR_beacon_SE(i) = 1.96*std(PDR_beacon)/(length(PDR_beacon)).^0.5;
        average_maxhop_beacon(i) = mean(maxhop_beacon(maxhop_beacon<20));
        average_maxhop_beacon_SE(i) = 1.96*std(maxhop_beacon(maxhop_beacon<20))/(length(maxhop_beacon(maxhop_beacon<20))).^0.5;
        disp(density(i))
    end

    
    
    fdata1 = fopen('end_to_end_delay.txt','w');%   
    fprintf(fdata1,'t_ABMEP: ');
    fprintf(fdata1,'%e ',average_time_exponent24(1,:));
    fprintf(fdata1,'\n');
    fprintf(fdata1,'t_3P3B ');
    fprintf(fdata1,'%e ',average_time_3P3B(1,:));
    fprintf(fdata1,'\n');
    fprintf(fdata1,'t_beacon: ');
    fprintf(fdata1,'%e ',average_time_beacon(1,:));
    fprintf(fdata1,'\n');   
    fclose(fdata1);
    
    fdata1 = fopen('PDR.txt','w');%   
    fprintf(fdata1,'PDR_ABMEP: ');
    fprintf(fdata1,'%e ',average_PDR_exponent24(1,:));
    fprintf(fdata1,'\n');
    fprintf(fdata1,'PDR_3P3B ');
    fprintf(fdata1,'%e ',average_PDR_3P3B(1,:));
    fprintf(fdata1,'\n');
    fprintf(fdata1,'PDR_beacon: ');
    fprintf(fdata1,'%e ',average_PDR_beacon(1,:));
    fprintf(fdata1,'\n');   
    fclose(fdata1);
    
    fdata1 = fopen('maxhop.txt','w');%
    fprintf(fdata1,'maxhop_ABMEP: ');
    fprintf(fdata1,'%e ',average_maxhop_exponent24(1,:));
    fprintf(fdata1,'\n');
    fprintf(fdata1,'maxhop_3P3B ');
    fprintf(fdata1,'%e ',average_maxhop_3P3B(1,:));
    fprintf(fdata1,'\n');
    fprintf(fdata1,'maxhop_beacon: ');
    fprintf(fdata1,'%e ',average_maxhop_beacon(1,:));
    fprintf(fdata1,'\n');
    fclose(fdata1);
    
    
    %准备先把折线图绘制出来，然后再加入指数分割中继节点选择算法延迟考虑，最后再加上别人的代码作性能比较
    x_vehicle_density =density;%通信范围内的车辆节点密度
    
    figure('NumberTitle', 'off', 'Name', 'Delay against Vehicle Density');
    errorbar(x_vehicle_density,reshape(average_time_exponent24,1,length(x_vehicle_density))*1e3,reshape(average_time_exponent24_SE,1,length(x_vehicle_density))*1e3,'s-','linewidth',1.1);hold on;
    errorbar(x_vehicle_density,reshape(average_time_3P3B,1,length(x_vehicle_density))*1e3,reshape(average_time_3P3B_SE,1,length(x_vehicle_density))*1e3,'ok-','linewidth',1.1);hold on;
    errorbar(x_vehicle_density,reshape(average_time_beacon,1,length(x_vehicle_density))*1e3,reshape(average_time_beacon_SE,1,length(x_vehicle_density))*1e3,'r->','linewidth',1.1);hold on;
    axis([0.01, 0.25, 3 8])
    set(gca,'XTick',[0.01:0.03:0.25],'linewidth',1.1,'fontsize',16,'fontname','times','ygrid','on');
    xlabel('Vehicle Density(Vehicle/Meter)');
    ylabel('End-to-end Delay(ms)');
    legend('ABMEP','3P3B','BBM','Location','southeast');%算法名字ABMEP：自适应指数分区广播算法   
    
%     figure('NumberTitle', 'off', 'Name', 'Delay against Vehicle Density');
%     plot(x_vehicle_density,average_time_exponent24.*1e3,'s-','linewidth',1.1);hold on;
%     plot(x_vehicle_density,average_time_3P3B.*1e3,'ok-','linewidth',1.1);hold on;
%     plot(x_vehicle_density,average_time_beacon.*1e3,'r->','linewidth',1.1);hold on;
%     axis([0.01, 0.25, 3 8])
%     set(gca,'XTick',[0.01:0.03:0.25],'linewidth',1.1,'fontsize',16,'fontname','times','ygrid','on');
%     xlabel('Vehicle Density(Vehicle/Meter)');
%     ylabel('End-to-end Delay(ms)');
%     legend('ABMEP','3P3B','BBM','Location','southeast');%算法名字ABMEP：自适应指数分区广播算法

    
    figure('NumberTitle', 'off', 'Name', 'PDR against Vehicle Density');
    errorbar(x_vehicle_density,reshape(average_PDR_exponent24,1,length(x_vehicle_density))*1e2,reshape(average_PDR_exponent24_SE,1,length(x_vehicle_density))*1e2,'s-','linewidth',1.1);hold on;
    errorbar(x_vehicle_density,reshape(average_PDR_3P3B,1,length(x_vehicle_density))*1e2,reshape(average_PDR_3P3B_SE,1,length(x_vehicle_density))*1e2,'ok-','linewidth',1.1);hold on;
    errorbar(x_vehicle_density,reshape(average_PDR_beacon,1,length(x_vehicle_density))*1e2,reshape(average_PDR_beacon_SE,1,length(x_vehicle_density))*1e2,'r->','linewidth',1.1);hold on;
    axis([0.01,0.25, 70 100])
    set(gca,'XTick',[0.01:0.03:0.25],'linewidth',1.1,'fontsize',16,'fontname','times','ygrid','on');
    xlabel('Vehicle Density(Vehicle/Meter)');
    ylabel('PDR(%)');
    legend('ABMEP','3P3B','BBM','Location','southeast');
    
%     figure('NumberTitle', 'off', 'Name', 'PDR against Vehicle Density');
%     plot(x_vehicle_density,average_PDR_exponent24.*1e2,'s-','linewidth',1.1);hold on;
%     plot(x_vehicle_density,average_PDR_3P3B.*1e2,'ok-','linewidth',1.1);hold on;
%     plot(x_vehicle_density,average_PDR_beacon.*1e2,'r->','linewidth',1.1);hold on;
%     axis([0.01,0.25, 70 100])
%     set(gca,'XTick',[0.01:0.03:0.25],'linewidth',1.1,'fontsize',16,'fontname','times','ygrid','on');
%     xlabel('Vehicle Density(Vehicle/Meter)');
%     ylabel('PDR(%)');
%     legend('ABMEP','3P3B','BBM','Location','southeast');
    
    
    figure('NumberTitle', 'off', 'Name', 'Hop against Vehicle Density');
    errorbar(x_vehicle_density,reshape(average_maxhop_exponent24,1,length(x_vehicle_density))*1e0,reshape(average_maxhop_exponent24_SE,1,length(x_vehicle_density))*1e0,'s-','linewidth',1.1);hold on;
    errorbar(x_vehicle_density,reshape(average_maxhop_3P3B,1,length(x_vehicle_density))*1e0,reshape(average_maxhop_3P3B_SE,1,length(x_vehicle_density))*1e0,'ok-','linewidth',1.1);hold on;
    errorbar(x_vehicle_density,reshape(average_maxhop_beacon,1,length(x_vehicle_density))*1e0,reshape(average_maxhop_beacon_SE,1,length(x_vehicle_density))*1e0,'r->','linewidth',1.1);hold on;
    axis([0.01,0.25, 10 20])
    set(gca,'XTick',[0.01:0.03:0.25],'linewidth',1.1,'fontsize',16,'fontname','times','ygrid','on');
    xlabel('Vehicle Density(Vehicle/Meter)');
    ylabel('Message Broadcast Maximum Hops(Hops)');
    legend('ABMEP','3P3B','BBM','Location','southeast');
    
%     figure('NumberTitle', 'off', 'Name', 'Hop against Vehicle Density');
%     plot(x_vehicle_density,average_maxhop_exponent24.*1e0,'s-','linewidth',1.1);hold on;
%     plot(x_vehicle_density,average_maxhop_3P3B.*1e0,'ok-','linewidth',1.1);hold on;
%     plot(x_vehicle_density,average_maxhop_beacon.*1e0,'r->','linewidth',1.1);hold on;
%     axis([0.01,0.25, 10 20])
%     set(gca,'XTick',[0.01:0.03:0.25],'linewidth',1.1,'fontsize',16,'fontname','times','ygrid','on');
%     xlabel('Vehicle Density(Vehicle/Meter)');
%     ylabel('Message Broadcast Maximum Hops(Hops)');
%     legend('ABMEP','3P3B','BBM','Location','southeast');

    
    t = toc;
    disp(t)
end