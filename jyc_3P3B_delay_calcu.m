function [delay_one_hop_average,PDR,relay] = jyc_3P3B_delay_calcu (location_vehi_in_Rp,density_EM,n_itera,n_parti,CW_CTB,sender,posi_opt,relay)

%% 基于log-partition机制，可以在inform_vehi中控制l_way=R,使得只是one-hop
%% m_p_one_hop：massege_progress_one_hop
%需要注意的是我们这里的location_vehi_in_Rp是sender的通信范围内的车辆节点信息，然后relay是已经选出来的中继节点，我们这个函数的作用就是算出选这个relay我们需要花费的延时
%故我们是已知sender、location_vehi_in_Rp、posi_opt和已经被我们选出来的relay，求选出该relay花费的时间
%    参数初始化
global t_SIFS  t_slot s_EM s_RTB s_CTB rate_bit n_cont_mini n_cont_CTB CW_min_DIFS Linmap1m;
if isempty(location_vehi_in_Rp)
    location_vehi_in_Rp = zeros(1,4);
    location_vehi_in_Rp(1,1) = 1;
end
R = (sum((sender-posi_opt).^2))^0.5/Linmap1m; %这里的R指的是sender和中继节点最优位置posi_opt之间的距离
delay_one_hop = zeros(2,1);
PDR = 1; %PDR是成功标记，1表示成功，0表示失败，我们初始化其为1
for i = 1:length(location_vehi_in_Rp(:,1))%找一找看location_vehi_in_Rp中是否有relay，没有的话，说明relay是直接选posi_opt点加上去的,location_vehi_in_Rp数组中只有全零行一行，那么我们需要在location_vehi_in_Rp数组中删除全零行并加入relay的信息
    if location_vehi_in_Rp(i,2) == relay(1,1) && location_vehi_in_Rp(i,3) == relay(1,2)
        haverelay = 1;%有relay的标记，有relay的话就置1并直接跳出for循环
        break;
    else
        haverelay = 0;%没有relay的话那就置0
    end
end
if haverelay == 0%如果location_vehi_in_Rp数组中没有relay存在
    if any(any(location_vehi_in_Rp(:,2:3)))==0
        location_vehi_in_Rp = [];
        location_vehi_in_Rp = [1 relay 0];   %我们将relay按格式加入location_vehi_in_Rp数组中,location_vehi_in_Rp的第四列存该车辆节点所在路段，我们重新赋值location_vehi_in_Rp，其车辆节点所在路段我们置0
    else
        location_vehi_in_Rp = [location_vehi_in_Rp;length(location_vehi_in_Rp(:,1))+1 relay 0]; 
    end
end
%接下来生成inform_vehi数组，数组中第一列为车辆所在小区标号，第二列为车速（目前为0），第三列为X坐标（我们location_vehi_in_Rp中的车辆X坐标置0），第四列存location_vehi_in_Rp中的车辆距sender的直线距离
inform_vehi = zeros(length(location_vehi_in_Rp(:,1)),4);
inform_vehi(:,1) = 0;%inform_vehi(:,1)存储的是车辆所在小区标号
inform_vehi(:,2) = 0;%inform_vehi(:,2)存储的是车速，目前为0
inform_vehi(:,3) = 0;%inform_vehi(:,3)存储的是置0的X坐标
for i = 1:length(location_vehi_in_Rp(:,1))
    inform_vehi(i,4) = (sum((posi_opt-location_vehi_in_Rp(i,2:3)).^2))^0.5/Linmap1m;  
end
%上面代码实现了inform_vehi数组的生成
%接下来我们需要修改一下relay参数，将其也变成X坐标为0，Y坐标为relay距sender的直线距离
relay(1,2) = (sum((posi_opt-relay(1,1:2)).^2))^0.5/Linmap1m;
relay(1,1) = 0;
%relay参数修改完毕
if relay(1,2)>R
    R = relay(1,2);
end
%idea：考虑将relay_y当作sender，定为0号位置，通信范围内其他车辆的位置都以与sender的距离代替，当然这里指的“其他车辆”不包括与消息传播方向相反的车辆节点（该类车辆节点已经被筛选出去了）
%我们已知sender，posi_opt，以及我们选出来的relay，然后我们将该relay对应sender与posi_opt之间路段的分区位置，求出分区延迟和其对应的争用延迟


%由于我们将X坐标置0，然后只看各个节点与sender的距离，并将其作为Y坐标，那么接下来我们设sender的Y坐标为relay_y，将其作为起点
relay_y=0;%起点为0
n_seg_min = (2^(n_parti-1))^n_itera;% 在传输范围内最小segmennt的个数

%接下来开始计算延时
while(1)
    inform_vehi(:,1)=zeros(length(inform_vehi(:,1)),1);
    n_EM =  max(1,random('Poisson',density_EM)); %随机产生n_EM个EMs，服从泊松分布
    if n_EM == 0
        n_EM=1;
    end
    %mini_DIFS in one hop
    
    [delay_contention,n_collision] = jyc_contention(n_EM,CW_min_DIFS,n_cont_mini,1);%信道争用延时计算代码（不做修改）
    t_accsee_channel = sum(delay_contention)+(t_SIFS+s_RTB/rate_bit)*(n_collision+1);%计算mini_DFIS的竞争信道的时长
    if t_accsee_channel==Inf%如果信道竞争延迟无限大，那么表示失败了
        PDR = 0;
        break;
    end
    t_mini_DIFS = t_accsee_channel;
    
    %parttion in one hop
    
    [t_partition,bound_left,bound_right,inform_vehi] = uniform_partition(relay_y,R,inform_vehi,n_parti,n_itera,relay);  
    
    %contition CTB
    vehicle_relay_parti_index=find(bound_left<inform_vehi(:,1)&(inform_vehi(:,1)<=bound_right));
    n_relay_parti=length(vehicle_relay_parti_index);%找出与relay同在一个小区段的车辆数目，用于之后计算随机争用延时，n_relay_parti就是参与竞争的车辆数目，为1则表示只有relay参与竞争
    if n_relay_parti==0
        sprintf('bound_right:%d,bound_left:%d,hop_count:%d',bound_right,bound_left,hop_count)
        pause
    end
    [delay_contention_CTB,n_collision] = jyc_contention(n_relay_parti,CW_CTB,n_cont_CTB,0);%小区内节点争用延时计算代码
    t_cont_in_CTB = sum(delay_contention_CTB);
    
    if  t_cont_in_CTB==Inf%如果争用延迟无限大，那么表示失败了
        PDR = 0;
        break;
    end
    t_cont_CTB = t_cont_in_CTB+(n_collision+1)*(t_slot*0.5+s_CTB/rate_bit)-t_slot*0.5;
    
    %data transmit
    t_data = t_SIFS+s_EM/rate_bit;
    %result in one hop
    delay_one_hop = t_mini_DIFS+t_partition+t_cont_CTB+t_data; 
    
    relay_index = vehicle_relay_parti_index(randperm(length(vehicle_relay_parti_index(:,1)),1),1);
    relay = location_vehi_in_Rp(relay_index,2:3);
    
    break;%跳出while(1)
end


    
if  PDR == 1
    delay_one_hop_average = delay_one_hop;   
else
    delay_one_hop_average = NaN;
end
