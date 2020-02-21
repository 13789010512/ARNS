function result()
    clear;clc;
    close all;
    warning('off');
    tic;
    global vehi_speed beacon_period branchset A Linmap1m T ima2 locationjunction w_mini_slot  retrans_max  t_DFIS t_SIFS t_switch t_slot v_signal delay_max s_EM s_RTB s_CTB s_ACK rate_bit CW_max_DIFS CW_min_DIFS  n_cont_mini n_cont_CTB  density_EM CW_CTB isbeacon n_fail;
    %��ʼ��һЩ��Ҫ����
    R = 200;
    vehi_speed = 60*1000/3600;%vehi_speed��ʾΪ���٣�60km/h����km/h����m/s����ʽ
    beacon_period = 100e-3;%beacon_period��ʾΪ�ű�������ڣ�������Ϊ100ms
    CW_CTB = 2;%CTB���ó�ʼ���ڴ�С
    density_EM = 2;%������Ϣ��Ŀ
    t_DFIS = 58e-6;
    t_SIFS = 32e-6;
    t_switch = 1e-6;
    t_slot = 13e-6;
    v_signal = 3e8;
    delay_max = 2e-6;%R/v_signal;�������ʱ
    s_EM = 500*8; %EM���Ĵ�С����λBytes
    s_RTB = 20*8;
    s_CTB = 14*8;
    s_ACK = 14*8;
    rate_bit = 18e6; %���ݴ����ʣ���λΪbps
    w_mini_slot = 2*delay_max+t_switch;%������Կ���mini_slot���Ա�slot�̣�mini_slot�����5e-6��slot��13e-6
    CW_max_DIFS = 2^10;%mini_DFIS���������cw�������contention������ѭ�ز�������·���ʻ��ƣ�CSMA/CD���Ķ�����ָ���˱��㷨ԭ�����Ϊ2��10�η���ȥ1����ȥ1�Ĳ�����jyc_contention�����е�t_backoff������
    CW_min_DIFS = 2;%��ʼ��mini_DFIS�������Ĵ�С����ѭ�ز�������·���ʻ��ƣ�CSMA/CD���Ķ�����ָ���˱��㷨ԭ��
    n_cont_mini = 16; %mini_DFIS������ͻ�������Ͻ磬��ѭ�ز�������·���ʻ��ƣ�CSMA/CD���Ķ�����ָ���˱��㷨ԭ�����Ϊ16
    n_cont_CTB = 5; %mini_CTB������ͻ�������Ͻ�
    T = 10;%�㲥����ʱ������
    % bar_symbol = 1;�ϰ����ǣ�0���������ϰ��������1�������ϰ������
    N_iter = [3;2];%���ε���
    N_part = [3;4];%���������ķ���ΪLog�㷨��������Ϊ3P3B�㷨
    A = 2;
    retrans_max = 5;%��ʼ������ش�����Ϊ3��   
    n_fail = 0;%��ʼ������ʧ�ܴ���Ϊ0��contention fail times are zeros��
    isbeacon_set = [0;1];%isbeacon������
    %��ʼ����Ҫ�������    
    simu_num = 2000;%�����������Ϊ5000��
    density = [0.01:0.03:0.25];%�����ܶ�����Ϊ0.005����������������/һ��������0.25��������������/һ��������ÿ��0.049���պõݽ���Σ��ݽ�������Ľڵ��ܶ��ǵ������ڵ��ܶȣ���λ��/�� 
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
    
    %�����ϰ���Ҷȶ�ֵ������
    ima = imread('system_map_bar.jpg');%imaΪͼ����󣨲�ɫ��
    ima1 = rgb2gray(ima);%ima1Ϊͼ��ĻҶȻ�����
    ima2 = ima1;%��ʼ��ima2��ima1ͬ��ͬ��ֵͬ
    %֮��ima2������ֻ���ϰ�������ֵ��0���ͷ��ϰ�������ֵ��242���Ķ�ֵ������
    for i = 1:length(ima2(:,1))
        for j = 1:length(ima2(1,:))
            if ima2(i,j)<=220%�ҳ�ima2�е��ϰ�������(�ϰ�������ֵΪ149)����,�����趨����ֵ��145-155��Χ�ڽ����ϰ���
                ima2(i,j) = 0;%���ҳ����ϰ������������Ӧ������ֵ��0�����ú�ɫ
            else
                ima2(i,j) = 242; %�Է��ϰ������������ֵ��242��242����ɫ
            end
        end
    end
%     imshow(ima2)
%     hold on
    %�ϰ���Ҷȶ�ֵ�������������
    
    %��·�ڵ���Ϣ����locationjucntion������
    fdata1 = fopen('system_junction.txt','r');%fdata1���ļ����ţ�fdata1=+N(N��������)����ʾ�ļ��򿪳ɹ����ļ�������N. fdata1=-1 : ��ʾ�ļ��򿪲��ɹ���fileID�ڴ˴��ļ��ر�ǰ������Ч�ġ�
    temp = fscanf(fdata1,'%e %e');%�Ӵ򿪵�fdata1�ļ����Կ�ѧ�������ķ�ʽ��ȡһ������������Ϊ�򿪵��ļ�����һ��������һ�����еķ�ʽ��ȡ��ֵ�ģ���һ����ȡ�����������ų�һ�С�
    locationjunction=ones(length(temp)/2,3);
    locationjunction(:,1)=1:length(temp)/2 ;%֧·��Ϣ��1:��ţ�2��3����x,y)
    locationjunction(:,2)=temp(1:2:length(temp)) ;
    locationjunction(:,3)=temp(2:2:length(temp)) ;%fdata1�򿪵��ļ��У���1�͵�4����������locationjucntion����ĵ�һ���У���2�͵�5���������˵ڶ����У���3�͵�6���������˵�������
    fclose(fdata1);
    %·�ڵ���Ϣ�洢���
    %��ʼ��branchsetԪ������·����Ϣ����branchsetԪ����
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
    %·����Ϣ�洢���
    %���ϰ�����Ϣ����locationbar������
    fdata1 = fopen('system_bar.txt','r');%
    temp = fscanf(fdata1,'%e %e %e');%��ȡ�ϰ�������Ϣ
    locationbar = ones(length(temp)/2,2);%�����ϰ�����Ϣ����
    locationbar(:,1)=temp(1:2:length(temp));
    locationbar(:,2)=temp(2:2:length(temp));%���ϰ�������Ϣ�����ϰ���������
    fclose(fdata1);
    mainroad2 = branchset{2,2};
    Linmap1m=sum((mainroad2(7,2:3)-mainroad2(1,2:3)).^2).^0.5/147;
    
%     for i = 1:26 %����һ�£���locationmark����ima2���ϰ���ͼƬ��ȣ��Ƿ�ȫ����ȷ��λ����
%         for j = 1:length(branchset{2,i}(:,1))
%             plot(branchset{2,i}(j,2),branchset{2,i}(j,3),'o','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor',[0.7,0.7,0.7],'MarkerSize',4);
%             hold on
%         end
%     end  
 
    for i = 1:length(density)
        %׼�����ɳ�����Ϣ
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
            [location_vehi,num_vehi]=generatetvehi(density(i),locationjunction,branchset,R,Linmap1m,locationbar);%�����ɷֲ����ڵ�·�ϣ�����֧·�Ϸֲ���������(x,y)=(0,0)��ʾ������
            fdata1 = fopen('system_vehi.txt','w');
            fprintf(fdata1,'%d ',location_vehi(1:num_vehi,:));%������Ϣ��1:��ţ�2��3����x,y);4:�ڷָ�ʱ����������seg��ʾ
            fprintf(fdata1,'\n');
            fclose(fdata1);
            for j = 1:length(isbeacon_set(:,1))
                isbeacon = isbeacon_set(j,1);%�Ƿ�������beacon���м̽ڵ�ѡ������,1Ϊ��beacon���㣬0Ϊ��ָ����������
                if isbeacon==0%��ʾ��ָ����������
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
                else%��ʾ������beacon����                   
                    [code_time_part,PDR_part,location_vehi,num_vehi,maxhop_part] = jyc_main(R,2,4,location_vehi,num_vehi);%���ű��㷨���ԣ�û��ָ������������ȷ���м̽ڵ㣬��N_iter,N_part����û��Ҫ��������������2,4ֻ��Ϊ��ȷ������ֵռ���������λ��
                    code_time_beacon(n) = code_time_part;
                    PDR_beacon(n) = PDR_part;
                    maxhop_beacon(n) = maxhop_part;
%                     clf;
                end
            end
        end

        average_time_exponent24(i) = mean(code_time_exponent24(code_time_exponent24<100));%����ʱ�洢�����У��������ô����ﵽ������ô������µ���Ϣ����ʧ�ܣ�������ʱ���޴������ų���
        average_time_exponent24_SE(i) = 1.96*std(code_time_exponent24(code_time_exponent24<100))/(length(code_time_exponent24(code_time_exponent24<100))).^0.5;
        average_PDR_exponent24(i) = mean(PDR_exponent24);
        average_PDR_exponent24_SE(i) = 1.96*std(PDR_exponent24)/(length(PDR_exponent24)).^0.5;
        average_maxhop_exponent24(i) = mean(maxhop_exponent24(maxhop_exponent24<20));
        average_maxhop_exponent24_SE(i) = 1.96*std(maxhop_exponent24(maxhop_exponent24<20))/(length(maxhop_exponent24(maxhop_exponent24<20))).^0.5;
        
        average_time_3P3B(i) = mean(code_time_3P3B(code_time_3P3B<100));%����ʱ�洢�����У��������ô����ﵽ������ô������µ���Ϣ����ʧ�ܣ�������ʱ���޴������ų���
        average_time_3P3B_SE(i) = 1.96*std(code_time_3P3B(code_time_3P3B<100))/(length(code_time_3P3B(code_time_3P3B<100))).^0.5;
        average_PDR_3P3B(i) = mean(PDR_3P3B);  
        average_PDR_3P3B_SE(i) = 1.96*std(PDR_3P3B)/(length(PDR_3P3B)).^0.5;
        average_maxhop_3P3B(i) = mean(maxhop_3P3B(maxhop_3P3B<20));
        average_maxhop_3P3B_SE(i) = 1.96*std(maxhop_3P3B(maxhop_3P3B<20))/(length(maxhop_3P3B(maxhop_3P3B<20))).^0.5;
        
        average_time_beacon(i) = mean(code_time_beacon(code_time_beacon<100));%����ʱ�洢�����У��������ô����ﵽ������ô������µ���Ϣ����ʧ�ܣ�������ʱ���޴������ų���
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
    
    
    %׼���Ȱ�����ͼ���Ƴ�����Ȼ���ټ���ָ���ָ��м̽ڵ�ѡ���㷨�ӳٿ��ǣ�����ټ��ϱ��˵Ĵ��������ܱȽ�
    x_vehicle_density =density;%ͨ�ŷ�Χ�ڵĳ����ڵ��ܶ�
    
    figure('NumberTitle', 'off', 'Name', 'Delay against Vehicle Density');
    errorbar(x_vehicle_density,reshape(average_time_exponent24,1,length(x_vehicle_density))*1e3,reshape(average_time_exponent24_SE,1,length(x_vehicle_density))*1e3,'s-','linewidth',1.1);hold on;
    errorbar(x_vehicle_density,reshape(average_time_3P3B,1,length(x_vehicle_density))*1e3,reshape(average_time_3P3B_SE,1,length(x_vehicle_density))*1e3,'ok-','linewidth',1.1);hold on;
    errorbar(x_vehicle_density,reshape(average_time_beacon,1,length(x_vehicle_density))*1e3,reshape(average_time_beacon_SE,1,length(x_vehicle_density))*1e3,'r->','linewidth',1.1);hold on;
    axis([0.01, 0.25, 3 8])
    set(gca,'XTick',[0.01:0.03:0.25],'linewidth',1.1,'fontsize',16,'fontname','times','ygrid','on');
    xlabel('Vehicle Density(Vehicle/Meter)');
    ylabel('End-to-end Delay(ms)');
    legend('ABMEP','3P3B','BBM','Location','southeast');%�㷨����ABMEP������Ӧָ�������㲥�㷨   
    
%     figure('NumberTitle', 'off', 'Name', 'Delay against Vehicle Density');
%     plot(x_vehicle_density,average_time_exponent24.*1e3,'s-','linewidth',1.1);hold on;
%     plot(x_vehicle_density,average_time_3P3B.*1e3,'ok-','linewidth',1.1);hold on;
%     plot(x_vehicle_density,average_time_beacon.*1e3,'r->','linewidth',1.1);hold on;
%     axis([0.01, 0.25, 3 8])
%     set(gca,'XTick',[0.01:0.03:0.25],'linewidth',1.1,'fontsize',16,'fontname','times','ygrid','on');
%     xlabel('Vehicle Density(Vehicle/Meter)');
%     ylabel('End-to-end Delay(ms)');
%     legend('ABMEP','3P3B','BBM','Location','southeast');%�㷨����ABMEP������Ӧָ�������㲥�㷨

    
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