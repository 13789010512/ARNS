function [t_partition_average,t_cont_CTB_average,delay_one_hop_average,m_p_one_hop_average,delay_end_to_end,hop_count,PDR,ratio_last_hop,ratio_delay_end_to_end] ...
    = triP3B(inform_vehi,density_EM,R,CW_CTB,relay)

    %    参数初始化
    global w_mini_slot R n_fail t_DFIS t_SIFS t_switch t_slot v_signal  delay_max s_EM s_RTB s_CTB rate_bit CW_max_DFIS CW_min_DFIS n_cont_mini n_cont_CTB;
    
    t_DFIS = 58e-6;
    t_SIFS = 32e-6;
    t_switch = 1e-6;
    t_slot = 13e-6;
    v_signal = 3e8;
    R = 900; %信号传输范围
    delay_max = 2e-6;%R/v_signal;最大传输延时
    s_EM = 500*8; %EM包的大小，单位Bytes
    s_RTB = 20*8;
    s_CTB = 14*8;
    rate_bit = 18e6; %数据传输率，单位为bps
    w_mini_slot = 2*delay_max+t_switch;%这里可以看出mini_slot明显比slot短，mini_slot大概是5e-6，slot是13e-6
    CW_max_DFIS = fix((t_DFIS-t_SIFS)/(2*delay_max+t_switch));%mini_DFIS slot的最多能有的数目（详见contention），fix为让值往靠零处取整
    CW_min_DFIS= max(ceil((ceil(CW_max_DFIS/2)+1)/4-1),1);%初始化mini_DFIS竞争窗的大小，此时为1个mini_DFIS slot，ceil为让值往正无穷大处取整,这里的减一是在完成ceil(CW_max_DFIS/2)+1)/4操作之后再减一
    n_cont_mini = 5; %mini_DFIS竞争冲突次数的上界
    n_cont_CTB = 20; %mini_CTB竞争冲突次数的上界
    n_fail = 0;%失败次数
    
    delay_one_hop = zeros(2,1);
    m_p_one_hop = zeros(2,1);
    %其中“ceil(l_way/R)”为“消息从原始发端发送至终端最少所需要的跳数”（消息发送端为坐标点0,消息接受端为坐标点1_way）（蒋）
    delay_end_to_end = 0;    %delay_end_to_end 为消息从原始发端至终端的总延迟，这里所赋的值未被使用是因为后面对该形参重新赋了值（蒋）
    hop_count = 0;           %跳数计数（蒋）
    %各参数至此初始化完成（蒋）
    
    %%log based partition EMsbroadcast
    
    t_mini_DIFS = zeros(2,1);
    t_partition = zeros(2,1);
    t_cont_CTB = zeros(2,1);
    t_data = zeros(2,1);
    
    relay_y=0;%初始用于确定每次partition_one_hop的起始位置，这只是单纯用直线坐标决定距离，如果精确特别在intersection下应该是距离
    %relay_y为转发（中继）节点位置，初始为0（蒋）
    n_itera=2;%两次迭代（蒋）
    n_parti=3;%三元分区（蒋）
    n_seg_min = n_parti^n_itera;% 在传输范围内最小segment的个数
    w_seg_min = R/n_seg_min; % 最小segment的宽度
    
    %以下进入3p3b
    hop_count = hop_count+1;%跳数计数加一（蒋）
    inform_vehi(:,1)=zeros(length(inform_vehi(:,1)),1);%初始化inform_vehi矩阵第一列的元素（蒋）
    if hop_count==1
        n_EM =  max(1,random('Poisson',density_EM)); %随机产生n_EM个EMs，服从泊松分布。/这里的n_EM是指随机生成n_EM个待发消息的源端，它们在后面要通过miniDIFS倒数，最先倒数完成的源端获得发送（RTB）消息的权利（蒋）。
    else
        n_EM = 1;
    end
    %mini_DIFS in one hop
    
    [delay_contention,n_collision] = contention(n_EM,CW_min_DFIS,n_cont_mini,1);  %最后的1是mini_DIFS_mark位，表示为:用于标志是否遵循冲突后窗放大的机制，1表示"是"（蒋）
    t_accsee_channel = sum(delay_contention)+(t_SIFS+s_RTB/rate_bit)*(n_collision+1);%计算mini_DFIS的竞争信道的时长
    if t_accsee_channel==Inf
        %         s=sprintf('fail:%d\n',n_fail);
        %         disp(s);
        break;
    end
    t_mini_DIFS(hop_count) = t_accsee_channel;%将每跳的mini_DIFS延迟存入数组中（蒋）
    
    %parttion in one hop
    if l_way-relay_y>R  %如果该跳通信范围内没有消息终端，则说明还需要接着转发消息，故range置为R，并在后面进行三元分区（蒋）
        range = R;
    else range = l_way-relay_y; %如果该跳通信范围内有消息终端，则最终要转发的路径为该跳消息发端到消息终端这段路径，range即为l_way-relay_y（蒋）
    end
    
    [t_partition(hop_count),bound_left,bound_right,inform_vehi]...%省略号表示该行没有结束，续行至下一行，t_partition(hop_count)是指第hop_count跳的分区耗时，bound_left是指迭代出的最远区段的左边界，bound_right是指最远区段的右边界（蒋）
        =uniform_partition(relay,range,inform_vehi,3,2);  %这里的3是赋给形参n_uniform_parti的实参，意为三元分区，这里的2是赋给形参n_uniform_itera的实参，意为二次迭代（蒋）
    
    %contition CTB
    vehicle_relay_parti_index=find(bound_left<inform_vehi(:,1)&(inform_vehi(:,1)<=bound_right));%find函数功能就是从矩阵中找出特定要求的元素，这里要求是从inform_vehi(:,1)矩阵中找出在最远区段（大于bound_left&小于bound_right）中的元素（蒋）
    n_relay_parti=length(vehicle_relay_parti_index);    %n_relay_parti即为最远区段中候选转发节点个数（蒋）
    %         if n_relay_parti==0
    %         sprintf('bound_right:%d,bound_left:%d,hop_count:%d',bound_right,bound_left,hop_count)
    %         pause
    %     end
    [delay_contention_CTB,n_collision] = contention_old(n_relay_parti,CW_CTB,n_cont_CTB,0);  %进入随机争用阶段，delay_contention_CTB是节点争用阶段的倒数延迟
    %n_collision是争用结束后往源端发送CTB数据包产生碰撞的次数
    %最后的0是mini_DIFS_mark位，表示为:用于标志是否遵循冲突后窗放大的机制，1表示"是"（蒋）
    t_cont_in_CTB = sum(delay_contention_CTB);  %sum函数默认对delay_contention_CTB数组的每列进行求和且以行的形式展示出来，如果被求和的函数只有一行或只有一列，则求和结果为一个值（蒋）
    
    if  t_cont_in_CTB==Inf
        %         s=sprintf('fail:%d\n',n_fail);
        %         disp(s);
        break;
    end
    t_cont_CTB(hop_count) =  t_cont_in_CTB+(n_collision+1)*(t_slot/2+s_CTB/rate_bit)-t_slot/2;  %成功传输CTB数据包至达成握手所消耗的时间，t_cont_in_CTB为随机争用阶段倒数消耗的总时间
    %（n_collision+1）意为发送了n_collision个在源端产生碰撞的CTB数据包和最后一次成功发送的CTB数据包
    %rate_bit为比特传输速率，则(t_slot/2+s_CTB/rate_bit)中的s_CTB/rate_bit是CTB数据包传回源端的耗时
    %t_slot/2是每次在源端产生CTB碰撞后源端需要缓冲休整的耗时，最后减去一个t_slot/2是因为最后一次CTB数据包传输成功了，源端不用进行等待缓冲（蒋）
    
    %data transmit
    t_data(hop_count) = t_SIFS+s_EM/rate_bit; %数据传输的耗时，为一个sifs时间加上数据量除以比特传输速率（蒋）
    
    %result in one hop
    n_seg_min = n_parti^n_itera;% 在传输范围内最小segmennt的个数
    w_seg_min = R/n_seg_min; % 最小segmennt的宽度
    delay_one_hop(hop_count) = t_mini_DIFS(hop_count)+t_partition(hop_count)+t_cont_CTB(hop_count)+t_data(hop_count); %一跳的总延迟（蒋）

    index = randi(length(vehicle_relay_parti_index),1,1); %在开区间  [0 , length(vehicle_relay_parti_index)]  内生成1X1随机矩阵(即一个数值)（蒋）
    m_p_one_hop(hop_count) = ((sum((relay-location_vehi_in_Rp(vehicle_relay_parti_index(index),2:3)).^2))^0.5/Linmap1m)/R;%(max(inform_vehi(vehicle_relay_parti_index,4))-relay_y);%随机选择最远区段中的节点作为转发节点，并求出其离源端的距离，将该距离除以R算出一跳进程（蒋）
    relay = location_vehi_in_Rp(vehicle_relay_parti_index(index),2:3);%选中的中继节点作为下一跳的源节点（蒋）
    %3p3b结束


