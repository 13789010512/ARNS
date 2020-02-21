function [delay_one_hop_mini_average,PDR] = jyc_log_partition (location_vehi_in_Rp,density_EM,N_iter,N_part,CW_CTB,sender,posi_opt,relay)

%% ����log-partition���ƣ�����һ����ʱ

%��Ҫע��������������location_vehi_in_Rp��sender��ͨ�ŷ�Χ�ڵĳ����ڵ���Ϣ��Ȼ��relay���Ѿ�ѡ�������м̽ڵ㣬����������������þ������ѡ���relay������Ҫ���ѵ���ʱ
%����������֪sender��location_vehi_in_Rp��posi_opt���Ѿ�������ѡ������relay����ѡ����relay���ѵ�ʱ��
%���Ǹ�Ϊ��Popt��Ϊ��ʼ0λ�ã���ͨ�ŷ�Χ�ڿɹ�ͨ�ŵĽڵ���Popt����Ծ���Ϊinform_vehi(:,4)   9.21��
%    ������ʼ��
global t_SIFS  t_slot s_EM s_RTB s_CTB rate_bit n_cont_mini n_cont_CTB Linmap1m CW_min_DIFS;
if isempty(location_vehi_in_Rp)
    location_vehi_in_Rp = zeros(1,4);
    location_vehi_in_Rp(1,1) = 1;
end
R = (sum((sender-posi_opt).^2))^0.5/Linmap1m; %�����Rָ����sender���м̽ڵ�����λ��posi_opt֮��ľ���

PDR = 1; %PDR�ǳɹ���ǣ�1��ʾ�ɹ���0��ʾʧ�ܣ����ǳ�ʼ����Ϊ1
for i = 1:length(location_vehi_in_Rp(:,1))%��һ�ҿ�location_vehi_in_Rp���Ƿ���relay��û�еĻ���˵��relay��ֱ��ѡposi_opt�����ȥ��,location_vehi_in_Rp������ֻ��ȫ����һ�У���ô������Ҫ��location_vehi_in_Rp������ɾ��ȫ���в�����relay����Ϣ
    if location_vehi_in_Rp(i,2) == relay(1,1) && location_vehi_in_Rp(i,3) == relay(1,2)
        haverelay = 1;%��relay�ı�ǣ���relay�Ļ�����1��ֱ������forѭ��
        break;
    else
        haverelay = 0;%û��relay�Ļ��Ǿ���0
    end
end
if haverelay == 0%���location_vehi_in_Rp������û��relay����
    if any(any(location_vehi_in_Rp(:,2:3)))==0
        location_vehi_in_Rp = [];
        location_vehi_in_Rp = [1 relay 0];   %���ǽ�relay����ʽ����location_vehi_in_Rp������,location_vehi_in_Rp�ĵ����д�ó����ڵ�����·�Σ��������¸�ֵlocation_vehi_in_Rp���䳵���ڵ�����·��������0
    else
        location_vehi_in_Rp = [location_vehi_in_Rp;length(location_vehi_in_Rp(:,1))+1 relay 0]; 
    end
end
%����������inform_vehi���飬�����е�һ��Ϊ��������С����ţ��ڶ���Ϊ���٣�ĿǰΪ0����������ΪX���꣨����location_vehi_in_Rp�еĳ���X������0���������д�location_vehi_in_Rp�еĳ�����sender��ֱ�߾���
inform_vehi = zeros(length(location_vehi_in_Rp(:,1)),4);
inform_vehi(:,1) = 0;%inform_vehi(:,1)�洢���ǳ�������С�����
inform_vehi(:,2) = 0;%inform_vehi(:,2)�洢���ǳ��٣�ĿǰΪ0
inform_vehi(:,3) = 0;%inform_vehi(:,3)�洢������0��X����
for i = 1:length(location_vehi_in_Rp(:,1))
    inform_vehi(i,4) = (sum((posi_opt-location_vehi_in_Rp(i,2:3)).^2))^0.5/Linmap1m;  
end
%�������ʵ����inform_vehi���������
%������������Ҫ�޸�һ��relay����������Ҳ���X����Ϊ0��Y����Ϊrelay��sender��ֱ�߾���
relay(1,2) = (sum((posi_opt-relay(1,1:2)).^2))^0.5/Linmap1m;
relay(1,1) = 0;
%relay�����޸����
if relay(1,2)>R
    R = relay(1,2);
end
%idea�����ǽ�relay_y����Popt����Ϊ0��λ�ã�ͨ�ŷ�Χ������������λ�ö�����Popt�ľ�����棬��Ȼ����ָ�ġ���������������������Ϣ���������෴�ĳ����ڵ㣨���೵���ڵ��Ѿ���ɸѡ��ȥ�ˣ�
%������֪sender��posi_opt���Լ�����ѡ������relay��Ȼ�����ǽ���relay��Ӧsender��posi_opt֮��·�εķ���λ�ã���������ӳٺ����Ӧ�������ӳ�

t_mini_DIFS = zeros(2,1);
t_partition_mini = zeros(2,1);
t_cont_CTB = zeros(2,1);
t_data = zeros(2,1);
%�������ǽ�X������0��Ȼ��ֻ�������ڵ���sender�ľ��룬��������ΪY���꣬��ô������������sender��Y����Ϊrelay_y��������Ϊ���
relay_y=0;%���Ϊ0
n_seg_min = (2^(N_part-1))^N_iter;% �ڴ��䷶Χ����Сsegmennt�ĸ���
w_seg_min = R/n_seg_min; % ��Сsegmennt�Ŀ���

%��������ʼ������ʱ
while(1)

    inform_vehi(:,1)=zeros(length(inform_vehi(:,1)),1);
    n_EM =  max(1,random('Poisson',density_EM)); %�������n_EM��EMs�����Ӳ��ɷֲ�
    if n_EM == 0
        n_EM=1;
    end
    %mini_DIFS in one hop
    
    [delay_contention,n_collision] = jyc_contention(n_EM,CW_min_DIFS,n_cont_mini,1);%�ŵ�������ʱ������루�����޸ģ�
    t_accsee_channel = sum(delay_contention)+(t_SIFS+s_RTB/rate_bit)*(n_collision+1);%����mini_DFIS�ľ����ŵ���ʱ��
    if t_accsee_channel==Inf%����ŵ������ӳ����޴���ô��ʾʧ����
        PDR = 0;
        break;
    end
    t_mini_DIFS = t_accsee_channel;
    
    %parttion in one hop
    [~,t_partition_mini,bound_left,bound_right,inform_vehi]=jyc_partition(relay_y,relay,R,inform_vehi,N_iter,N_part);%������ʱ������루��Ҫ�޸ģ�
    
    %contition CTB
    vehicle_relay_parti_index=find(bound_left<inform_vehi(:,1)&(inform_vehi(:,1)<=bound_right));
    n_relay_parti=length(vehicle_relay_parti_index);%�ҳ���relayͬ��һ��С���εĳ�����Ŀ������֮��������������ʱ��n_relay_parti���ǲ��뾺���ĳ�����Ŀ��Ϊ1���ʾֻ��relay���뾺��
    if n_relay_parti==0
        %             plot(inform_vehi(:,3),inform_vehi(:,4),'.');
        sprintf('bound_right:%d,bound_left:%d,hop_count:%d',bound_right,bound_left,hop_count)
        pause
    end
    [delay_contention_CTB,n_collision] = jyc_contention(n_relay_parti,CW_CTB,n_cont_CTB,0);%С���ڽڵ�������ʱ�������
    t_cont_in_CTB = sum(delay_contention_CTB);
    
    if  t_cont_in_CTB==Inf%��������ӳ����޴���ô��ʾʧ����
        PDR = 0;
        break;
    end

    t_cont_CTB =  t_cont_in_CTB+(n_collision+1)*(t_slot*0.5+s_CTB/rate_bit)-t_slot*0.5;
    
    %data transmit
    t_data = t_SIFS+s_EM/rate_bit;
    %delay in one hop
    delay_one_hop_mini= t_mini_DIFS+t_partition_mini+t_cont_CTB+t_data;  
    break;%����while(1)
end


    
if  PDR == 1
    delay_one_hop_mini_average = delay_one_hop_mini;     
else
    delay_one_hop_mini_average = NaN;
end