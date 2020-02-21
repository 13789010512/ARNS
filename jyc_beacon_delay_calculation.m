function [delay,PDR,newrelay,route_table,PDR_channel] = jyc_beacon_delay_calculation(route_table,density_EM,sender,relay,R,location_vehi,ima2,send_end,locationmark,posi_opt,fail_time)

%% 基于log-partition机制，可以在inform_vehi中控制l_way=R,使得只是one-hop
%% m_p_one_hop：massege_progress_one_hop
%需要注意的是我们这里的location_vehi_in_Rp是sender的通信范围内的车辆节点信息，然后relay是已经选出来的中继节点，我们这个函数的作用就是算出选这个relay我们需要花费的延时
%故我们是已知sender、location_vehi_in_Rp、posi_opt和已经被我们选出来的relay，求选出该relay花费的时间
%    参数初始化
global t_SIFS s_EM s_RTB s_CTB rate_bit n_cont_mini CW_max_DIFS Linmap1m vehi_speed beacon_period s_ACK CW_min_DIFS;
PDR_channel = 1;%信道争用的PDR，初始化为1
relay_index = [];
for i = 1:length(route_table(:,1))
    route_table(i,5) = beacon_period*(randi(100)/100);%将route_table的第5列更新为该车辆节点的信标更新时间
    if route_table(i,2) == relay(1) && route_table(i,3) == relay(2)%找出relay在路由表中的位置
        relay_index = i;
    end
end
if isempty(relay_index)%如果没有找出relay_index，那么可能是relay_index出现了信标更新，我们需要找出route_table(:,3)中与relay(2)差值最小的数所在行，作为我们的relay_index
    D_value = zeros(length(route_table(:,1)),1);
    for i = 1:length(route_table(:,1))
        D_value(i) = abs(route_table(i,3)-relay(2));
    end
    [relay_index,~] = find(D_value(:,1)==min(D_value(:,1)));
end
if route_table(relay_index,6)>R%如果我们所选relay距sender的距离大于R，说明该relay应该是Popt点，我们找Popt点时，会出现计算上的一点偏差，为了公平，将其更新为R即可
    route_table(relay_index,6)=R;
end

if route_table(relay_index,6)+route_table(relay_index,5)*vehi_speed>R%如果中继节点与sender的距离+中继节点信标更新时延*车速的结果大于通信半径
    %那么说明我们选中的relay有一定的概率在信标更新间隙行驶出通信范围
    %随机生成一个0或1的数，表示随机这个车辆节点是向外行驶还是向内行驶
    %0：向外，此车会开出通信范围进而导致sender（也就是old_relay）找不到其作为new_relay，通信失败；
    %1：向内，通信成功
    PDR = round(rand(1,1));%PDR从0和1两值中随机一个
else
    PDR = 1;     
end

delay = 0;%初始化延迟为0

%考虑将通信范围内满足条件的车辆位置都以与sender的距离代替
%生成inform_vehi数组，数组中第1列为车辆所在小区标号，第2列为车速，第3列为X坐标（我们location_vehi_in_Rp中的车辆X坐标置0），第四列存车辆距sender的直线距离
inform_vehi = [];%初始化inform_vehi为空，其在后面始终与route_table数组行数保持一致，第1列存储的是车辆编号；第2列存储的是车辆行驶方向：0向外，1向内；第3列存储的是信标更新间隙(<beacon_period)；第4列存储的是车辆距sender的直线距离
%如果上面inform_vehi数组中的某一车辆节点，其信标更新间隙>beacon_period，则更新其距sender的直线距离，并将信标更新间隙重置（重置：原信标更新间隙-beacon_period）
newrelay = zeros(1,2);%初始化newrelay数组，只有在出现原relay行驶出通信范围时才会进行赋值，newrelay中储存的是[0 与sender的距离]

%接下来开始计算这一跳不论失败与否的延时
n_EM = max(1,random('Poisson',density_EM)); %随机产生n_EM个EMs，服从泊松分布
if n_EM == 0
    n_EM=1;
end
[delay_contention,n_collision] = jyc_contention(n_EM,CW_min_DIFS,n_cont_mini,1);%信道竞争完毕并成功广播RTS的延时计算代码，delay_contention为DIFS竞争时长数组，n_collision为广播RTS冲突次数
t_accsee_channel = sum(delay_contention)+(s_RTB/rate_bit+t_SIFS)*(n_collision+1);%计算竞争信道的总耗时，由DIFS竞争完成时延和成功广播RTS时延构成，其中成功广播RTS时延由（RTS广播时延+模式切换时延SIFS）*（广播RTS冲突次数+1）构成
if t_accsee_channel==Inf      %如果信道竞争延迟无限大，那么表示广播RTS冲突次数达到上限
    PDR_channel = 0;                  %本次消息广播由于信道争用广播RTS冲突次数达上限，导致通信失败，不再加入重传中
end
t_cont_CTB = s_CTB/rate_bit;  %计算广播CTS时延
t_data = t_SIFS+s_EM/rate_bit;%计算传输数据时延，由模式切换时延SIFS+数据传输时延构成
t_ACK = t_SIFS+s_ACK/rate_bit;%计算传输ACK信号时延，由模式切换时延SIFS+ACK信号传输时延构成
delay = t_accsee_channel+t_cont_CTB+t_data+t_ACK;

if delay==Inf                 %如果是由于信道争用广播RTS冲突次数达到上限导致的失败，我们将此次delay参数做缺失处理
    delay = NaN;              %NaN表示数据缺失
    
    %接下来加入重传前的车辆节点位置信息更新赋值在inform_vehi数组中，并选一个新的relay赋值进newrelay数组中，只考虑“我们选中的relay在信标更新间隙行驶出通信范围”这一种PDR=0的情况，
    %因为“由于信道争用失败多次，达最大次数”导致的PDR=0的情况，其在contention函数中已经做了相应操作，并得出了对应的延时、PDR参数
elseif PDR==0 && delay~=Inf   %如果PDR等于0，且不是信道争用达最大次数的情况，那么我们进行一定范围内的车辆节点位置信息更新
    route_table(:,5) = route_table(:,5)+delay;%更新路由表车辆节点的信标更新间隔时间
    R_expand = beacon_period*vehi_speed+R;%求出拓展区域半径为信标更新周期乘以车速，在大于R小于R_expand的范围内会有一定可能存在从通信范围外行驶至通信范围内的车辆节点，如果其信标不到更新周期beacon_period，sender是不知道其存在的，反之，sender知道其存在，可以选其作为new_relay
    if fail_time == 0%如果是第一次进入延时计算函数，那么可以求出拓展区域内的车辆节点将其加入路由表中计算，如果不是第一次进入，那么拓展区域内的车辆节点已经被加入进路由表了，没必要再次加入
        [~,location_vehi_in_RE]=jyc_bar_posi_opt(location_vehi,locationmark,R_expand,Linmap1m,sender,send_end,ima2); %求出location_vehi_in_RE（通信拓展区域内的车辆节点）
    else
        location_vehi_in_RE = [];
    end
    if isempty(location_vehi_in_RE)==0%如果location_vehi_in_RE不为零
        for i = 1:length(route_table(:,1))
            for j = 1:length(location_vehi_in_RE(:,1))
                if route_table(i,2) == location_vehi_in_RE(j,2) && route_table(i,3) == location_vehi_in_RE(j,3)
                    location_vehi_in_RE(j,:)=0;%将已经在路由表中的车辆节点给排除
                end
            end
        end
        location_vehi_in_RE(location_vehi_in_RE(:,1)==0,:)=[];
        rank_num = length(route_table(:,1));%rank_num1用来更新route_table信息
        if isempty(location_vehi_in_RE)==0%如果location_vehi_in_RE不为零，表示存在大于R小于R_expand的范围内的车辆节点，将location_vehi_in_RE（拓展区域）内的车辆节点加入路由表中，因为这类节点有一定可能从通信范围外行驶至通信范围内
            for n = 1:length(location_vehi_in_RE(:,1))%将拓展区域内的车辆节点新增至路由表中
                route_table(rank_num+1,1:4) = location_vehi_in_RE(n,1:4);
                route_table(rank_num+1,5) = beacon_period*(randi(100)/100)+delay;%将route_table新增的第5列赋值为该车辆节点的信标更新时间加上我们的delay延迟
                route_table(rank_num+1,6) = (sum((route_table(rank_num+1,2:3)-sender(1,1:2)).^2))^0.5/Linmap1m;%将route_table新增的第6列赋值为该车辆节点与sender的距离
                rank_num = rank_num+1;
            end
        end
    end
    %接下来对newrelay和inform_vehi进行赋值
    inform_vehi(:,1) = route_table(:,1);                                         %inform_vehi(:,1)存储的是车辆编号
    for i = 1:length(route_table(:,1))                                           %生成inform_vehi数组详细信息
        if route_table(i,5)-beacon_period>=0                                     %如果路由表中的车辆节点信标更新间隙长度达到了信标更新周期
            if i~=relay_index                                                    %如果该点不是relay点，则给其随机一个方向属性
                inform_vehi(i,2) = round(rand(1,1));                             %inform_vehi(:,2)存储的是车辆行驶方向：0向外，1向内
            else                                                                 %如果该点是relay点
                inform_vehi(i,2) = 0;                                            %则赋予其向外的方向属性
            end
            inform_vehi(i,3) = route_table(i,5)-beacon_period;                   %inform_vehi(:,3)存储的是新的信标更新间隙(<beacon_period)
            if inform_vehi(i,2)==0                                               %如果车辆节点向外行驶，则将其原距离加上beacon_period*vehi_speed
                inform_vehi(i,4) = route_table(i,6)+beacon_period*vehi_speed;    %inform_vehi(:,4)存储的是车辆距sender的直线距离
            else                                                                 %如果车辆节点向内行驶，则将其原距离减去beacon_period*vehi_speed
                inform_vehi(i,4) = route_table(i,6)-beacon_period*vehi_speed;    %inform_vehi(:,4)存储的是车辆距sender的直线距离
            end
        else                                                                     %如果路由表中的车辆节点信标更新间隙长度未达到信标更新周期
            if i~=relay_index                                                    %如果该点不是relay点，则给其随机一个方向属性
                inform_vehi(i,2) = round(rand(1,1));                             %inform_vehi(:,2)存储的是车辆行驶方向：0向外，1向内
            else                                                                 %如果该点是relay点
                inform_vehi(i,2) = 0;                                            %则赋予其向外的方向属性
            end
            inform_vehi(i,3) = route_table(i,5);
            inform_vehi(i,4) = route_table(i,6);
        end
    end
    dis_sender_newrelay = max(inform_vehi(inform_vehi(:,4)<R,4));%找出小于R且距离值最大的距离
    if isempty(dis_sender_newrelay)%如果找不到        
        newrelay_index = length(inform_vehi(:,1))+1;
        inform_vehi(newrelay_index,1) = length(location_vehi(:,1))+1;
        inform_vehi(newrelay_index,2) = round(rand(1,1));
        inform_vehi(newrelay_index,3) = beacon_period*(randi(100)/100);
        inform_vehi(newrelay_index,4) = (sum((posi_opt(1,1:2)-sender(1,1:2)).^2))^0.5/Linmap1m;
        if inform_vehi(newrelay_index,4)>R%我们找Popt点时，会出现计算上的一点偏差，为了公平，将其更新为R即可
            inform_vehi(newrelay_index,4)=R;
        end
        newrelay = [0 inform_vehi(newrelay_index,4)];%则将newrelay赋值为posi_opt
    else                   %如果找到了
        [newrelay_index,~] = find(inform_vehi(:,4)==dis_sender_newrelay);
        if length(newrelay_index)>1
            newrelay_index = newrelay_index(1,1);
        end
        newrelay = [0 route_table(newrelay_index,6)];%则将newrelay赋值为该车辆节点
    end
    if length(inform_vehi(:,1))-length(route_table(:,1))>0
        [posi_opt_road_index]=jyc_find_relayroad(locationmark,posi_opt);
        route_table(end+1,4) = posi_opt_road_index;
    end
    route_table(:,1) = inform_vehi(:,1);
    route_table(:,2) = 0;
    route_table(:,3) = inform_vehi(:,4);  
    route_table(:,5) = inform_vehi(:,3);
    route_table(:,6) = inform_vehi(:,4);
end


