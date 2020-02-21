function  [t,location_relay,relay,PDR,location_vehi,num_vehi]=jyc_bar_complete_relay_selection(location_vehi,locationmark,R,num_vehi,N_iter,N_part,A,Linmap1m,T,ima2,t_relay)
%%%生成道路像素信息，及表示出道路轮廓及十字路口
 global locationjunction density_EM CW_CTB;
 send_start(1,1:2) = locationmark(length(locationmark(:,1)),1:2);%直接取路径车辆运行目的点，即消息发送端点
 send_start(1,3) = length(locationmark(:,1));
                    %%%-----找离start最近的节点做消息的初始发送节点
                    %dis_vehi_start = ((location_vehi(:,2)-start(1)).^2+(location_vehi(:,3)-start(2)).^2).^0.5;%找离start最近的节点做消息的初始发送节点?
                    %index_start = find(dis_vehi_start==min(dis_vehi_start));
                    %if isempty(index_start)
                    %    a=1;
                    %end
                    %send_start = location_vehi(index_start,2:3);%消息传播起点车辆，只保存位置信息（x,y)
 %消息传播起点和终点都设置成路段标记点
 send_end = locationmark(1,1:2);%消息传播终点车辆
 send_end(1,3) = 1;
 isend = 0;%isend参数用来表示send_end点是不是终结junction点，初始化为0，即不是，如果是终结junction点，那么isend值为1
 for i = 1:11%终结junction点都储存在locationjunction数组的前11行
     if locationjunction(i,2) == send_end(1,1) && locationjunction(i,3) == send_end(1,2)%如果send_end是终结junction点
         isend = 1;
         break;
     end
 end
%  if isend ==0
%      plot(send_start(1),send_start(2),'o','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);
%      hold on
%      plot(send_end(1),send_end(2),'o','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);
%      hold on
%  else
%      plot(send_start(1),send_start(2),'o','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','r','MarkerSize',6);
%      hold on
%      plot(send_end(1),send_end(2),'o','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','r','MarkerSize',6);
%      hold on
%  end
%%%-----找消息传递的终点位置
 %relayend = locationjucntion(1,2:3);%只保存位置信息（x,y)
  relayend = send_end(1:2);
  bar_bef_end = 0;%relay和relayend之间存在障碍物的标志位初始化为0，0表示不存在relay和relayend之间的障碍物，1表示存在
 %城市情景的Linmap1m = sum((locationmark(3,1:2)-locationmark(4,1:2)).^2).^0.5/395;%计算1米在图中对应的像素间距离
 %高速路情景的Linmap1m=sum((locationmark(6,1:2)-locationmark(7,1:2)).^2).^0.5/3700;%高速路情景，计算1米在图中对应的像素间距离
 locationmark_cover = zeros(length(locationmark(:,1)),3);%初始化路段标记点覆盖数组，用来储存已被广播覆盖的路段标记点
 locationmark_nocover = zeros(length(locationmark(:,1)),3);%初始化路段标记点未被覆盖数组，用来存储未被广播覆盖的路段标记点
 locationmark_nocover(:,1) = 1:length(locationmark(:,1));%给未被广播覆盖的路段标记点数组赋值
 locationmark_nocover(:,2:3) = locationmark(:,1:2);%给未被广播覆盖的路段标记点数组赋值
 %下面的循环用来更新路段标记点的状态，将已满足被覆盖条件的路段标记点从未被覆盖路段标记点数组中选出放入已被覆盖路段标记点数组中，并将未被覆盖路段标记点数组中其的信息置极大数10000
 %在之后的代码中下面循环的结构会经常被用到
 for i = 1:length(locationmark_nocover(:,1))
     crossbar = 0;%初始化穿越障碍物标志位置0
     dis_point_to_sendstart = (sum((locationmark_nocover(i,2:3)-send_start(1:2)).^2))^0.5/Linmap1m;%dis_point_to_sendstart是未被覆盖的路段标记点到send_start的距离
     if dis_point_to_sendstart<=R %如果未被覆盖的路段标记点到send_start的距离小于通信半径R
         sample_num = dis_point_to_sendstart/10;%sample_num是抽样次数，这里我们设置为每10m抽样一次
         detx = (locationmark_nocover(i,2)-send_start(1))/sample_num;%每次抽样x坐标的递进量
         dety = (locationmark_nocover(i,3)-send_start(2))/sample_num;%每次抽样y坐标的递进量
         for n = 1:sample_num
             templex = ceil(send_start(1)+n*detx);%每次抽样的x坐标（向大取整）
             templey = ceil(send_start(2)+n*dety);%每次抽样的y坐标（向大取整）
             if ima2(templey,templex)==0%如果被障碍物阻挡
                 crossbar = 1;%穿越障碍物标志位置1
                 break;
             end            
         end
         if crossbar == 0%如果该路段标记点未被障碍物阻碍（路段标记点与send_start的连线不穿过障碍物），且在广播通信覆盖范围内（第一个if的条件），说明该点应被覆盖
             locationmark_cover(i,:) = locationmark_nocover(i,:);%将该点的信息转移到已被覆盖数组中
             locationmark_nocover(i,:) = 10000;%将未被覆盖数组中该点位置的信息置极大数10000
         end
     end
 end
 %路段标记点的状态更新完毕
 
 
 L_road = sum((locationmark(2:length(locationmark(:,1)),1:2)-locationmark(1:length(locationmark(:,1))-1,1:2)).^2,2).^0.5/Linmap1m;%计算在整个道路上的由mark确定的第i条路段的现实长度
 L_road_all = sum(L_road);%路段的现实总长度
 
 location_relay = zeros(100,4);%保存所有中继节点，1：序号； 2，3：(x,y)； 4：场景(0任意情况，1直道，2弯道，3十字路口)
 n_relay = 1;%正常广播的序号标记
 location_re_relay = zeros(100,4);%保存所有的反向广播中继节点，1：序号； 2，3：(x,y)； 4：场景(0任意情况，1直道，2弯道，3十字路口)
 n_re_relay = 1;%反向广播的序号标记 
 relay = send_start(1,1:2);%只保存位置信息（x,y)
 
 old_sender = [];
 sender = [];
 posi_opt = [];
 old_posi_opt = [];%初始化参数为空
 
 has_cov_symbol = 0;%初始化空白区域已经被覆盖的标记位为0
 relay_main = relay;
 t = t_relay;
 re_t = 0;%初始化反向广播的时间re_t为0
 PDR_set = [];%初始化PDR为空
 last_t = 0;%初始化最后一跳的耗时，当我们需要做最后一跳relay更正（更正并非新增一跳，而是对原跳进行更改）时，用来在本函数最后做更正后relay的耗时更正，用更正前的总耗时t减去last_t最后再加上更正后的新耗时，即得更正后的准确总耗时
 
 while ((isend == 1 && (bar_bef_end==1 || (sum((relay-relayend).^2))^0.5/Linmap1m>R)) || (isend == 0 && (bar_bef_end==1 || (sum((relay-relayend).^2))^0.5/Linmap1m>R/2))) && t<T 
     %设置条件必须要满足：选到普通junction点（relayend点）R/2范围内的车辆为中继节点（或终结Junction点R范围内的车辆为中继节点）且relay和relayend之间不能存在障碍物才能结束本循环
     old_posi_opt = posi_opt;
     old_sender = sender;
     sender = relay;
     [posi_opt,location_vehi_in_Rp]=jyc_bar_posi_opt(location_vehi,locationmark,R,Linmap1m,relay_main,send_end,ima2);     %找出Popt点
     %计算在Rp范围内的路段长度
     road_in_Rp = 0; %初始在Rp范围内的路段长度为0
     index_Popt = 1;
     index_relay = 1;
     part_Popt = zeros(length(locationmark(:,1)),1);
     part_relay = zeros(length(locationmark(:,1)),1);
     for i = 1:length(locationmark(:,1))
         if (sum((posi_opt-locationmark(i,1:2)).^2))^0.5/Linmap1m<=R
             part_Popt(index_Popt) = i;
             index_Popt = index_Popt+1;
         end
         if (sum((relay-locationmark(i,1:2)).^2))^0.5/Linmap1m<=R
             part_relay(index_relay) = i;
             index_relay = index_relay+1;
         end
     end
     part_in_Rp = intersect(part_Popt,part_relay);
     part_in_Rp(part_in_Rp==0) = [];
     l_relay_road = zeros(length(part_in_Rp),1);
     l_Popt_road = zeros(length(part_in_Rp),1);
     for i = 1:length(part_in_Rp)
         l_relay_road(i) = (sum((relay-locationmark(part_in_Rp(i),1:2)).^2))^0.5/Linmap1m;%relay点与其自身所在路段的终点的距离
         l_Popt_road(i) = (sum((posi_opt-locationmark(part_in_Rp(i),1:2)).^2))^0.5/Linmap1m;%Popt点与其自身所在路段的起点的距离
     end
     relay_road = min(l_relay_road);
     Popt_road = min(l_Popt_road);
     road_in_Rp = relay_road+Popt_road;             %road_in_Rp中先加入首尾段的长度
     for i = 1:length(part_in_Rp)-1
         if part_in_Rp(i)+1 == part_in_Rp(i+1)%如果第i个part_in_Rp数组中存储的路段点与第i+1个part_in_Rp数组中存储的路段点相连
             road_in_Rp = road_in_Rp+L_road(part_in_Rp(i));%在Rp范围内的路段长度road_in_Rp就加上该路段的长度L_road(i)
         end
     end
     if has_cov_symbol == 0
         [vacant0,vacant,re_symbol]=jyc_vacantbool(n_relay,locationmark_nocover,location_relay,send_start,locationmark);
     else%如果has_cov_symbol标记位为1
         re_symbol = 0;%反向广播标记位置0
         has_cov_symbol = 0;%has_cov_symbol作用：将一次的re_symbol置0
     end
     %上面计算完在Rp范围内的路段长度，并存在了road_in_Rp中
     %接下来根据在Rp中的路段长度判断是进入弯道选择还是直道选择
     if re_symbol==0
         [t,location_relay,n_relay,relay_main,relay,num_vehi,location_vehi,n_re_relay,PDR_part,last_t]...
             =jyc_bar_curve_relay_selection(location_vehi_in_Rp,location_vehi,n_relay,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,relay,t,posi_opt,n_re_relay,locationmark,ima2,send_end,old_sender,old_posi_opt);
     else
         [location_relay,re_t,num_vehi,location_vehi,location_re_relay,n_re_relay,has_cov_symbol,PDR_part,last_t]...
             =jyc_bar_reverse_relay_selection(location_vehi,n_relay,locationmark,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,send_start,t,re_t,location_re_relay,n_re_relay,vacant0,vacant,re_symbol,relay_main,has_cov_symbol,T,ima2);
         %如果上述反向广播jyc_bar_reverse_relay_selection结束，那么将vacant0中存储的路段标记点对应locationmark_nocover和locationmark_cover中的路段标记点信息给更新一下
         %即vacant0中储存的路段标记点全被覆盖了，以下为更新locationmark_nocover和locationmark_cover数组操作
         for num = 1:length(vacant0(:,1))
             locationmark_cover(vacant0(num,3),:) = locationmark_nocover(vacant0(num,3),:);
             locationmark_nocover(vacant0(num,3),:) = 10000;
         end
     end 
     PDR_set = [PDR_set;PDR_part];
     %每跳结束更新路段标记点覆盖情况
     for i = 1:length(locationmark_nocover(:,1))
         crossbar = 0;%初始化穿越障碍物标志位置0
         dis_point_to_relay = (sum((locationmark_nocover(i,2:3)-relay(1:2)).^2))^0.5/Linmap1m;%dis_point_to_relay是未被覆盖的路段标记点到relay的距离
         if dis_point_to_relay<=R %如果未被覆盖的路段标记点到relay的距离小于通信半径R
             sample_num = dis_point_to_relay/10;%sample_num是抽样次数，这里我们设置为每10m抽样一次
             detx = (locationmark_nocover(i,2)-relay(1))/sample_num;%每次抽样x坐标的递进量
             dety = (locationmark_nocover(i,3)-relay(2))/sample_num;%每次抽样y坐标的递进量
             for n = 1:sample_num
                 templex = ceil(relay(1)+n*detx);%每次抽样的x坐标（向大取整）
                 templey = ceil(relay(2)+n*dety);%每次抽样的y坐标（向大取整）
                 if ima2(templey,templex)==0%如果被障碍物阻挡
                     crossbar = 1;%穿越障碍物标志位置1
                     break;
                 end
             end
             if crossbar == 0%如果该路段标记点未被障碍物阻碍（路段标记点与relay的连线不穿过障碍物），且在广播通信覆盖范围内（第一个if的条件），说明该点应被覆盖
                 locationmark_cover(i,:) = locationmark_nocover(i,:);%将该点的信息转移到已被覆盖数组中
                 locationmark_nocover(i,:) = 10000;%将未被覆盖数组中该点位置的信息置极大数10000
             end
         end
     end    
     if (sum((relay-relayend).^2))^0.5/Linmap1m<=R/2%如果已经满足了relay和relayend之间的距离小于R/2，准备要跳出while循环了，我们最后检查一下relay和relayend之间是否存在障碍物
         relay_to_relayend = (sum((relay-relayend).^2))^0.5/Linmap1m;%relay_to_relayend是relay点到relayend点之间的距离
         sample_num = relay_to_relayend/10;%sample_num是抽样次数，这里我们设置为每10m抽样一次
         detx = (relayend(1)-relay(1))/sample_num;%每次抽样x坐标的递进量
         dety = (relayend(2)-relay(2))/sample_num;%每次抽样y坐标的递进量
         for n = 1:sample_num
             templex = ceil(relay(1)+n*detx);%每次抽样的x坐标（向大取整）
             templey = ceil(relay(2)+n*dety);%每次抽样的y坐标（向大取整）
             if ima2(templey,templex)==0%如果被障碍物阻挡
                 bar_bef_end = 1;%relayend前是否存在障碍物标志位置1，表示relay和relayend之间存在障碍物
                 break;
             end
         end
     end   
 end
 
 if any(any(location_relay))==0%如果在本路段上一个中继节点都没有选，那么我们就将本路段的send_start存入location_relay中
     location_relay(1,1) = 1;
     location_relay(1,2:3) = send_start(1,1:2);
 end
 if isend==0%如果send_end是普通Junction点
     %接下来开始确认一下在最后一跳relay通信范围内是否存在比其还要更靠近relayend（也称作junction）的点
     [location_vehi_in_Rp]=jyc_vehi_near_junction_in_Rp_choose(location_vehi,locationmark,R,Linmap1m,relay,send_end,ima2);%找出最后一跳relay通信范围内的车辆
     nearest = relay;%初始化距relayend最近的车辆节点参数nearest，其初值是最后一跳relay的坐标
     for i = 1:length(location_vehi_in_Rp(:,1))%通过for循环选出最靠近relayend的点
         dis1 = (sum((location_vehi_in_Rp(i,2:3)-relayend).^2))^0.5/Linmap1m;
         dis2 = (sum((nearest-relayend).^2))^0.5/Linmap1m;
         if dis1 < dis2 %如果存在比nearest还靠近relayend的车辆节点
             nearest = location_vehi_in_Rp(i,2:3);%更新nearest
         end
     end
     %确认完毕，如果存在nearest的更新，说明存在比最后一跳relay更靠近relayend的点，反之则不存在
     if nearest(1,1)~=relay(1,1) || nearest(1,2)~=relay(1,2)%只要出现nearest中存储的X、Y坐标中的某一个与relay的X、Y坐标不对等，那么说明更新过了nearest参数         
         send_end(1,1:2) = nearest;
         send_end(1,3) = 1;
         posi_opt = nearest;             
         NZ = find(sum(abs(location_relay),2)~=0, 1, 'last' );%NZ意为非零，这里求出的是location_relay中非零行数，在后面当作length(A(;,1))的使用
         if NZ>1%如果location_relay里面存了不止一个relay，意为在本次中继节点选择路段上不止进行过一次中继节点选择
             dis3 = (sum((nearest-location_relay(NZ-1,2:3)).^2))^0.5/Linmap1m;%计算nearest点与最后一跳relay上一跳的距离
             sender = location_relay(NZ-1,2:3);
         else%如果location_relay里面只存了一个relay，意为在本次中继节点选择路段上只进行过一次中继节点选择
             dis3 = (sum((nearest-send_start(1,1:2)).^2))^0.5/Linmap1m;%计算nearest点与send_start的距离
             sender = send_start(1,1:2);
         end
         if dis3 < R %如果在最后一跳relay的上一跳(或在send_start)处就可以直接跳到nearest点，那么就直接把最后一跳relay更新成nearest点             
             relay = nearest;             
             [location_vehi_in_Rp]=jyc_vehi_near_junction_in_Rp_choose(location_vehi,locationmark,R,Linmap1m,sender,send_end,ima2);    %得出location_vehi_in_Rp
             if N_part==4%如果是log算法
                 [delay_one_hop_mini_average,PDR_part] = jyc_log_partition (location_vehi_in_Rp,density_EM,N_iter,N_part,CW_CTB,sender,posi_opt,relay);
                 t = t-last_t+delay_one_hop_mini_average;%重新加入最后一跳耗费的t时间，即上文说的耗时更正
             else        %否则就是N_part==3，即3P3B算法
                 [delay_one_hop_average,PDR_part] = jyc_3P3B_delay_calcu (location_vehi_in_Rp,density_EM,N_iter,N_part,CW_CTB,sender,posi_opt,relay);
                 t = t-last_t+delay_one_hop_average;%重新加入最后一跳耗费的t时间，即上文说的耗时更正
             end
             location_relay(NZ,2:3) = relay;            
         else%如果一跳无法跳到nearest点，那么将原最后一跳relay变为新sender，选择nearest点作为新的最后一跳relay，并存进location_relay数组的对应位置，更新t的值
             sender = relay;%多做一跳，那么原relay就成了新sender
             relay = nearest;
             [location_vehi_in_Rp]=jyc_vehi_near_junction_in_Rp_choose(location_vehi,locationmark,R,Linmap1m,sender,send_end,ima2);    %得出location_vehi_in_Rp
             if N_part==4%如果是log算法
                 [delay_one_hop_mini_average,PDR_part] = jyc_log_partition (location_vehi_in_Rp,density_EM,N_iter,N_part,CW_CTB,sender,posi_opt,relay);
                 t = t+delay_one_hop_mini_average;%多做一跳，那么加入新一跳耗时
             else        %否则就是N_part==3，即3P3B算法
                 [delay_one_hop_average,PDR_part] = jyc_3P3B_delay_calcu (location_vehi_in_Rp,density_EM,N_iter,N_part,CW_CTB,sender,posi_opt,relay);
                 t = t+delay_one_hop_average;%多做一跳，那么加入新一跳耗时
             end
             location_relay(NZ+1,1) = NZ+1;
             location_relay(NZ+1,2:3) = relay;
         end
         PDR_set = [PDR_set;PDR_part];
%          plot(relay(1),relay(2),'^','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);%将新的最后一跳relay在图中画出
%          hold on        
     end
 end
 if isempty(PDR_set)%说明没进入while循环，直接就结束了，那么我们默认PDR_set为1，算成功
     PDR_set = 1;
 end
 
 if mean(PDR_set(:,1))~=1
     PDR = 0;
 else
     PDR = 1;
 end
 
 if re_t>t %如果反向广播耗时大于普通广播耗时
     t = re_t;%则取反向广播耗时为广播耗时
 end