function [t,location_relay,relay,location_vehi,num_vehi,PDR]=jyc_beacon_relay_selection(location_vehi,locationmark,R,num_vehi,N_iter,N_part,A,Linmap1m,T,ima2,t_relay)
%%%生成道路像素信息，及表示出道路轮廓及十字路口
 global locationjunction density_EM CW_CTB retrans_max;
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
 posi_opt = [];
 old_posi_opt = [];
 old_sender = [];
 sender = [];%初始化参数
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
 location_relay = zeros(100,4);%保存所有中继节点，1：序号； 2，3：(x,y)； 4：场景(0任意情况（此列作废），1直道，2弯道，3十字路口)
 n_relay = 1;%正常广播的序号标记
 relay = send_start(1,1:2);%初始化relay等于send_start的坐标
 t = t_relay;
 PDR_set = [];%初始化PDR为空
 last_t = 0;%初始化最后一跳的耗时，当我们需要做最后一跳relay更正（更正并非新增一跳，而是对原跳进行更改）时，用来在本函数最后做更正后relay的耗时更正，用更正前的总耗时t减去last_t最后再加上更正后的新耗时，即得更正后的准确总耗时
 
 
 while ((isend == 1 && (bar_bef_end==1 || (sum((relay-relayend).^2))^0.5/Linmap1m>R)) || (isend == 0 && (bar_bef_end==1 || (sum((relay-relayend).^2))^0.5/Linmap1m>R/2))) && t<T 
     %设置条件必须要满足：选到普通junction点（relayend点）R/2范围内的车辆为中继节点（或终结Junction点R范围内的车辆为中继节点）且relay和relayend之间不能存在障碍物才能结束本循环     
     delay_one_hop = 0;%在每跳开始，初始化一跳消息传输耗时
     fail_time = 0;%初始化失败次数
     if isempty(posi_opt)==0%如果posi_opt不为空
         old_posi_opt = posi_opt;%将老的posi_opt的值放入old_posi_opt中储存
     end
     [posi_opt,location_vehi_in_Rp]=jyc_bar_posi_opt(location_vehi,locationmark,R,Linmap1m,relay,send_end,ima2); %找出Popt点和location_vehi_in_Rp（通信范围内的车辆节点数组）
     route_table = location_vehi_in_Rp;%route_table为路由表数组，其前4列与location_vehi_in_Rp一致，第5列存储距信标上次更新过了多长时间，第6列储存车辆节点与relay的距离
     %以下对route_table数组内的节点进行了一些筛选
     if isempty(route_table)==0%如果route_table不为空
         [relay_road_index]=jyc_find_relayroad(locationmark,relay);%求出relay所在路段编号
         for i = 1:length(route_table(:,1))%在此for循环中，将弯道场景横穿迂回路段导致出现空白区域的情况去掉
             if abs(route_table(i,4)-relay_road_index)>25%横穿时，车辆与relay的路段编号跨度起码在25个路段以上，故这里用通信范围内车辆节点所在路段减去relay所在路段，得出的绝对值如果大于25，说明该车辆位置横穿了迂回路段，应舍去
                 route_table(i,:)=0;
             end
         end
         route_table(route_table(:,1)==0,:)=[];
     end
     %筛选完毕，剔除了横穿弯道迂回路段的车辆节点
     
     if isempty(route_table)%如果经过筛选后的通信范围内没有车辆节点，那么我们直接选posi_opt作为新relay
         if isempty(sender)==0%如果sender不为空
             old_sender = sender;%将老sender赋值给old_sender
         end
         sender = relay;%将老relay赋值给sender，即表示老relay已经成为新sender
         [location_vehi,num_vehi,relay]=add_vehi(locationmark,relay,Linmap1m,ima2,posi_opt,send_end,R,num_vehi,location_vehi,old_sender,old_posi_opt);%放入老relay，求出新relay
         [relay_road_index]=jyc_find_relayroad(locationmark,relay);%求出新relay的所在路段编号
         route_table(1,1) = num_vehi;%第1列存储该车辆节点在location_vehi中的编号
         route_table(1,2:3) = relay;%第2、3列存储posi_opt的坐标
         route_table(1,4) = relay_road_index;%第4列是posi_opt所在路段编号
         route_table(1,5) = 0;%第5列是距信标上次更新过了多长时间，目前暂时置零
         route_table(1,6) = (sum((relay-sender).^2))^0.5/Linmap1m;%route_table第6列存储relay车辆与sender的距离，一般来说与R相比会出现一点点偏差
         %新的relay点加入location_vehi_in_Rp数组中
         %这里要注意，location_vehi_in_Rp数组不一定和route_table数组一样都是空数组，route_table是空数组有可能是在上面经过了一次筛选导致变空
         vehi_num_in_Rp = length(location_vehi_in_Rp(:,1))+1;
         location_vehi_in_Rp(vehi_num_in_Rp,1) = num_vehi;%第1列存储该车辆节点在location_vehi中的编号
         location_vehi_in_Rp(vehi_num_in_Rp,2:3) = relay;%第2、3列存储posi_opt的坐标
         location_vehi_in_Rp(vehi_num_in_Rp,4) = relay_road_index;%第4列是posi_opt所在路段编号

     else
         route_table(:,5) = 0;%第5列是该车辆节点距其信标上次更新过了多长时间，目前暂时置零
         for i = 1:length(route_table(:,1))
             route_table(i,6) = (sum((route_table(i,2:3)-relay(1:2)).^2))^0.5/Linmap1m;%route_table第6列用来存取车辆与relay的距离
         end
         [index,~] = find(route_table(:,6)==max(route_table(:,6)));%找出通信范围内距relay最远的车辆的行下标
         sender = relay;%当我们确定了下一个relay时，先将现有的relay赋值给sender
         relay = route_table(index,2:3);%该车辆就是我们要找的relay，更新relay         
     end
     
     location_relay(n_relay,1) = n_relay;
     location_relay(n_relay,2:3) = relay;
     n_relay = n_relay+1;
%      plot(relay(1),relay(2),'^','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);%将新的最后一跳relay在图中画出
%      hold on
     relay_in_delay_calculation = relay;%生成一个专用用来计算延时的relay参数
     
     %接下来将我们找出的relay、消息发送者sender、route_table等参数，放入延时计算函数中计算延时、PDR等参数，该类参数在经历第一次重传返回时，数组中关于车辆节点的位置信息全部换成0和其距sender的距离，若不经历重传，则不改变车辆节点的位置信息
     while(1)
         [delay,PDR_part,relay_in_delay_calculation,route_table,PDR_channel] ...
             = jyc_beacon_delay_calculation (route_table,density_EM,sender,relay_in_delay_calculation,R,location_vehi,ima2,send_end,locationmark,posi_opt,fail_time);
%          PDR_set = [PDR_set;PDR_part];
         delay_one_hop = delay_one_hop+delay;    %存储一跳消息耗时
         t = t+delay;
         if PDR_part == 1 && PDR_channel==1      %如果消息传输成功
             PDR_set = [PDR_set;1];              %将成功标志1存入PDR_set数组中
             break;                              %并跳出当前while循环
         elseif fail_time >= retrans_max || PDR_channel==0 %如果PDR_part不等于1，且重传达最大次数,或者直接信道争用达最大次数失败，表示此次消息传播失败，延时为NaN，不应该被计入t中(9.16)
             PDR_set = [PDR_set;0];              %将失败标志0存入PDR_set数组中
             t = t-delay_one_hop;                %将已计入t中的一跳消息延迟给减去(9.16)
             delay_one_hop = NaN;                %将原一跳消息延迟改为NaN(9.16)
             break;                              %并跳出while循环
         elseif isnan(delay)                     %如果是信道争用失败，属于重传的情况
             fail_time = fail_time +1;           %失败次数+1
         else                                    %如果是在信标更新间隙被选relay车辆行驶出sender通信范围内，属于重传的情况
             fail_time = fail_time +1;           %失败次数+1
         end
     end
     last_t = delay_one_hop;%将一跳消息耗时赋值给last_t，当进行到最后一跳时，delay_one_hop即为最后一跳耗时
 
 end    
 
 %以下是最后一跳的处理操作，以方便最后一跳更加靠近junction点
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
         send_end(1,1:2) = nearest;%将距relayend最近的车辆节点定为send_end
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
         %接下来判断nearest点与sender点之间是否存在障碍物阻碍，如果存在障碍物阻碍，即使dis3<R也不能让最后一跳直接更新成nearest点，而是通过最后一跳relay点进行消息中继转发至nearest点
         nearest_bar_sender = 0;%初始化是否存在障碍物阻碍标记位为0
         sample_num = 100;%初始化抽样次数为100
         det_X = (nearest(1)-sender(1))/sample_num;
         det_Y = (nearest(2)-sender(2))/sample_num;       
         for j = 1:sample_num
             temp_x = sender(1)+j*det_X;
             temp_y = sender(2)+j*det_Y;
             x_data = ceil(temp_x);
             y_data = ceil(temp_y);
             if ima2(y_data,x_data) == 0    %如果坐标对应图像矩阵的像素值为0，说明此处为障碍物，说明车辆被障碍物阻碍了
                 nearest_bar_sender = 1;
                 break;
             end
         end
         %nearest点与sender点之间是否存在障碍物阻碍，判断完毕         
         if dis3 < R && nearest_bar_sender == 0%如果在最后一跳relay的上一跳(或在send_start)处就可以直接跳到nearest点且无障碍物阻碍，那么就直接把最后一跳relay更新成nearest点             
             relay = nearest;  
             location_relay(NZ,2:3) = relay;            
             [location_vehi_in_Rp]=jyc_vehi_near_junction_in_Rp_choose(location_vehi,locationmark,R,Linmap1m,sender,send_end,ima2);%得出location_vehi_in_Rp，即为sender和nearest（send_end）之间的车辆节点数组（包括nearest在内）
             route_table = [];%更新route_table
             route_table(:,1:4) = location_vehi_in_Rp(:,1:4);
             route_table(:,5) = 0;
             for i = 1:length(route_table(:,1))
                 route_table(i,6) = (sum((route_table(i,2:3)-sender(1:2)).^2))^0.5/Linmap1m;%route_table第6列用来存取车辆与relay的距离
             end
             fail_time = 0;%初始化失败次数为0
             relay_in_delay_calculation = relay;%生成一个专用用来计算延时的relay参数
             t = t-last_t;%总耗时更正第一步：先减去最后一跳耗时
             while(1)
                 [delay,PDR_part,relay_in_delay_calculation,route_table,PDR_channel] ...
                     = jyc_beacon_delay_calculation (route_table,density_EM,sender,relay_in_delay_calculation,R,location_vehi,ima2,send_end,locationmark,posi_opt,fail_time);
                 t = t+delay;                            %总耗时更正第二步：重新加入最后一跳耗时，完成耗时更正
             %    PDR_set = [PDR_set;PDR_part];
                 if PDR_part == 1 && PDR_channel==1      %如果消息传输成功
                     PDR_set = [PDR_set;1];              %将成功标志1存入PDR_set数组中
                     break;                              %并跳出当前while循环
                 elseif fail_time >= retrans_max || PDR_channel==0  %如果PDR_part不等于1，且重传达最大次数，或直接信道争用失败
                     PDR_set = [PDR_set;0];              %将失败标志0存入PDR_set数组中
                     break;                              %并跳出while循环
                 elseif isnan(delay)                     %如果是信道争用失败，属于重传的情况
                     fail_time = fail_time +1;           %失败次数+1
                 else                                    %如果是在信标更新间隙被选relay车辆行驶出sender通信范围内，属于重传的情况
                     fail_time = fail_time +1;           %失败次数+1
                 end
             end
             
         else%如果一跳无法跳到nearest点（因为距离原因或是障碍物原因），那么将原最后一跳relay变为新sender，选择nearest点作为新的最后一跳relay，并存进location_relay数组的对应位置，更新t的值
             sender = relay;%多做一跳，那么原relay就成了新sender
             relay = nearest;
             location_relay(NZ+1,1) = NZ+1;
             location_relay(NZ+1,2:3) = relay;                        
             [location_vehi_in_Rp]=jyc_vehi_near_junction_in_Rp_choose(location_vehi,locationmark,R,Linmap1m,sender,send_end,ima2);    %得出location_vehi_in_Rp
             route_table = [];%更新route_table
             route_table(:,1:4) = location_vehi_in_Rp(:,1:4);
             route_table(:,5) = 0;
             for i = 1:length(route_table(:,1))
                 route_table(i,6) = (sum((route_table(i,2:3)-sender(1:2)).^2))^0.5/Linmap1m;%route_table第6列用来存取车辆与relay的距离
             end
             fail_time = 0;
             relay_in_delay_calculation = relay;%生成一个专用用来计算延时的relay参数
             while(1)
                 [delay,PDR_part,relay_in_delay_calculation,route_table,PDR_channel] ...
                     = jyc_beacon_delay_calculation (route_table,density_EM,sender,relay_in_delay_calculation,R,location_vehi,ima2,send_end,locationmark,posi_opt,fail_time);
           %      PDR_set = [PDR_set;PDR_part];
                 t = t+delay;                            %多做一跳，那么加入新一跳耗时
                 if PDR_part == 1 && PDR_channel==1      %如果消息传输成功
                     PDR_set = [PDR_set;1];              %将成功标志1存入PDR_set数组中
                     break;                              %并跳出当前while循环
                 elseif fail_time >= retrans_max || PDR_channel==0  %如果PDR_part不等于1，且重传达最大次数，或直接信道争用达最大次数失败
                     PDR_set = [PDR_set;0];              %将失败标志0存入PDR_set数组中
                     break;                              %并跳出while循环
                 elseif isnan(delay)                     %如果是信道争用失败，属于重传的情况
                     fail_time = fail_time +1;           %失败次数+1
                 else                                    %如果是在信标更新间隙被选relay车辆行驶出sender通信范围内，属于重传的情况
                     fail_time = fail_time +1;           %失败次数+1
                 end              
             end
         end
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
 
