function [t_partition_average,t_cont_CTB_average,delay_one_hop_average,m_p_one_hop_average,delay_end_to_end,hop_count,PDR,ratio_last_hop,ratio_delay_end_to_end] ...
    = triP3B(inform_vehi,density_EM,R,CW_CTB,relay)

    %    ������ʼ��
    global w_mini_slot R n_fail t_DFIS t_SIFS t_switch t_slot v_signal  delay_max s_EM s_RTB s_CTB rate_bit CW_max_DFIS CW_min_DFIS n_cont_mini n_cont_CTB;
    
    t_DFIS = 58e-6;
    t_SIFS = 32e-6;
    t_switch = 1e-6;
    t_slot = 13e-6;
    v_signal = 3e8;
    R = 900; %�źŴ��䷶Χ
    delay_max = 2e-6;%R/v_signal;�������ʱ
    s_EM = 500*8; %EM���Ĵ�С����λBytes
    s_RTB = 20*8;
    s_CTB = 14*8;
    rate_bit = 18e6; %���ݴ����ʣ���λΪbps
    w_mini_slot = 2*delay_max+t_switch;%������Կ���mini_slot���Ա�slot�̣�mini_slot�����5e-6��slot��13e-6
    CW_max_DFIS = fix((t_DFIS-t_SIFS)/(2*delay_max+t_switch));%mini_DFIS slot��������е���Ŀ�����contention����fixΪ��ֵ�����㴦ȡ��
    CW_min_DFIS= max(ceil((ceil(CW_max_DFIS/2)+1)/4-1),1);%��ʼ��mini_DFIS�������Ĵ�С����ʱΪ1��mini_DFIS slot��ceilΪ��ֵ���������ȡ��,����ļ�һ�������ceil(CW_max_DFIS/2)+1)/4����֮���ټ�һ
    n_cont_mini = 5; %mini_DFIS������ͻ�������Ͻ�
    n_cont_CTB = 20; %mini_CTB������ͻ�������Ͻ�
    n_fail = 0;%ʧ�ܴ���
    
    delay_one_hop = zeros(2,1);
    m_p_one_hop = zeros(2,1);
    %���С�ceil(l_way/R)��Ϊ����Ϣ��ԭʼ���˷������ն���������Ҫ������������Ϣ���Ͷ�Ϊ�����0,��Ϣ���ܶ�Ϊ�����1_way��������
    delay_end_to_end = 0;    %delay_end_to_end Ϊ��Ϣ��ԭʼ�������ն˵����ӳ٣�����������ֵδ��ʹ������Ϊ����Ը��β����¸���ֵ������
    hop_count = 0;           %��������������
    %���������˳�ʼ����ɣ�����
    
    %%log based partition EMsbroadcast
    
    t_mini_DIFS = zeros(2,1);
    t_partition = zeros(2,1);
    t_cont_CTB = zeros(2,1);
    t_data = zeros(2,1);
    
    relay_y=0;%��ʼ����ȷ��ÿ��partition_one_hop����ʼλ�ã���ֻ�ǵ�����ֱ������������룬�����ȷ�ر���intersection��Ӧ���Ǿ���
    %relay_yΪת�����м̣��ڵ�λ�ã���ʼΪ0������
    n_itera=2;%���ε���������
    n_parti=3;%��Ԫ����������
    n_seg_min = n_parti^n_itera;% �ڴ��䷶Χ����Сsegment�ĸ���
    w_seg_min = R/n_seg_min; % ��Сsegment�Ŀ��
    
    %���½���3p3b
    hop_count = hop_count+1;%����������һ������
    inform_vehi(:,1)=zeros(length(inform_vehi(:,1)),1);%��ʼ��inform_vehi�����һ�е�Ԫ�أ�����
    if hop_count==1
        n_EM =  max(1,random('Poisson',density_EM)); %�������n_EM��EMs�����Ӳ��ɷֲ���/�����n_EM��ָ�������n_EM��������Ϣ��Դ�ˣ������ں���Ҫͨ��miniDIFS���������ȵ�����ɵ�Դ�˻�÷��ͣ�RTB����Ϣ��Ȩ����������
    else
        n_EM = 1;
    end
    %mini_DIFS in one hop
    
    [delay_contention,n_collision] = contention(n_EM,CW_min_DFIS,n_cont_mini,1);  %����1��mini_DIFS_markλ����ʾΪ:���ڱ�־�Ƿ���ѭ��ͻ�󴰷Ŵ�Ļ��ƣ�1��ʾ"��"������
    t_accsee_channel = sum(delay_contention)+(t_SIFS+s_RTB/rate_bit)*(n_collision+1);%����mini_DFIS�ľ����ŵ���ʱ��
    if t_accsee_channel==Inf
        %         s=sprintf('fail:%d\n',n_fail);
        %         disp(s);
        break;
    end
    t_mini_DIFS(hop_count) = t_accsee_channel;%��ÿ����mini_DIFS�ӳٴ��������У�����
    
    %parttion in one hop
    if l_way-relay_y>R  %�������ͨ�ŷ�Χ��û����Ϣ�նˣ���˵������Ҫ����ת����Ϣ����range��ΪR�����ں��������Ԫ����������
        range = R;
    else range = l_way-relay_y; %�������ͨ�ŷ�Χ������Ϣ�նˣ�������Ҫת����·��Ϊ������Ϣ���˵���Ϣ�ն����·����range��Ϊl_way-relay_y������
    end
    
    [t_partition(hop_count),bound_left,bound_right,inform_vehi]...%ʡ�Ժű�ʾ����û�н�������������һ�У�t_partition(hop_count)��ָ��hop_count���ķ�����ʱ��bound_left��ָ����������Զ���ε���߽磬bound_right��ָ��Զ���ε��ұ߽磨����
        =uniform_partition(relay,range,inform_vehi,3,2);  %�����3�Ǹ����β�n_uniform_parti��ʵ�Σ���Ϊ��Ԫ�����������2�Ǹ����β�n_uniform_itera��ʵ�Σ���Ϊ���ε���������
    
    %contition CTB
    vehicle_relay_parti_index=find(bound_left<inform_vehi(:,1)&(inform_vehi(:,1)<=bound_right));%find�������ܾ��ǴӾ������ҳ��ض�Ҫ���Ԫ�أ�����Ҫ���Ǵ�inform_vehi(:,1)�������ҳ�����Զ���Σ�����bound_left&С��bound_right���е�Ԫ�أ�����
    n_relay_parti=length(vehicle_relay_parti_index);    %n_relay_parti��Ϊ��Զ�����к�ѡת���ڵ����������
    %         if n_relay_parti==0
    %         sprintf('bound_right:%d,bound_left:%d,hop_count:%d',bound_right,bound_left,hop_count)
    %         pause
    %     end
    [delay_contention_CTB,n_collision] = contention_old(n_relay_parti,CW_CTB,n_cont_CTB,0);  %����������ý׶Σ�delay_contention_CTB�ǽڵ����ý׶εĵ����ӳ�
    %n_collision�����ý�������Դ�˷���CTB���ݰ�������ײ�Ĵ���
    %����0��mini_DIFS_markλ����ʾΪ:���ڱ�־�Ƿ���ѭ��ͻ�󴰷Ŵ�Ļ��ƣ�1��ʾ"��"������
    t_cont_in_CTB = sum(delay_contention_CTB);  %sum����Ĭ�϶�delay_contention_CTB�����ÿ�н�����������е���ʽչʾ�������������͵ĺ���ֻ��һ�л�ֻ��һ�У�����ͽ��Ϊһ��ֵ������
    
    if  t_cont_in_CTB==Inf
        %         s=sprintf('fail:%d\n',n_fail);
        %         disp(s);
        break;
    end
    t_cont_CTB(hop_count) =  t_cont_in_CTB+(n_collision+1)*(t_slot/2+s_CTB/rate_bit)-t_slot/2;  %�ɹ�����CTB���ݰ���������������ĵ�ʱ�䣬t_cont_in_CTBΪ������ý׶ε������ĵ���ʱ��
    %��n_collision+1����Ϊ������n_collision����Դ�˲�����ײ��CTB���ݰ������һ�γɹ����͵�CTB���ݰ�
    %rate_bitΪ���ش������ʣ���(t_slot/2+s_CTB/rate_bit)�е�s_CTB/rate_bit��CTB���ݰ�����Դ�˵ĺ�ʱ
    %t_slot/2��ÿ����Դ�˲���CTB��ײ��Դ����Ҫ���������ĺ�ʱ������ȥһ��t_slot/2����Ϊ���һ��CTB���ݰ�����ɹ��ˣ�Դ�˲��ý��еȴ����壨����
    
    %data transmit
    t_data(hop_count) = t_SIFS+s_EM/rate_bit; %���ݴ���ĺ�ʱ��Ϊһ��sifsʱ��������������Ա��ش������ʣ�����
    
    %result in one hop
    n_seg_min = n_parti^n_itera;% �ڴ��䷶Χ����Сsegmennt�ĸ���
    w_seg_min = R/n_seg_min; % ��Сsegmennt�Ŀ��
    delay_one_hop(hop_count) = t_mini_DIFS(hop_count)+t_partition(hop_count)+t_cont_CTB(hop_count)+t_data(hop_count); %һ�������ӳ٣�����

    index = randi(length(vehicle_relay_parti_index),1,1); %�ڿ�����  [0 , length(vehicle_relay_parti_index)]  ������1X1�������(��һ����ֵ)������
    m_p_one_hop(hop_count) = ((sum((relay-location_vehi_in_Rp(vehicle_relay_parti_index(index),2:3)).^2))^0.5/Linmap1m)/R;%(max(inform_vehi(vehicle_relay_parti_index,4))-relay_y);%���ѡ����Զ�����еĽڵ���Ϊת���ڵ㣬���������Դ�˵ľ��룬���þ������R���һ�����̣�����
    relay = location_vehi_in_Rp(vehicle_relay_parti_index(index),2:3);%ѡ�е��м̽ڵ���Ϊ��һ����Դ�ڵ㣨����
    %3p3b����


