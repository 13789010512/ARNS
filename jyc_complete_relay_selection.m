function  [t,location_relay]=jyc_complete_relay_selection(location_vehi,locationjucntion,locationbranch,locationmark,R,num_vehi,N_iter,N_part,A,T,Linmap1m)
%%%生成道路像素信息，及表示出道路轮廓及十字路口
 start = locationmark(length(locationmark(:,1)),1:2);%直接取路径车辆运行目的点，即消息发送端点，即猴子石大桥位置
 %%%-----找离start最近的节点做消息的初始发送节点
 
 dis_vehi_start = ((location_vehi(:,2)-start(1)).^2+(location_vehi(:,3)-start(2)).^2).^0.5;%找离start最近的节点做消息的初始发送节点?
 index_start = find(dis_vehi_start==min(dis_vehi_start));
 if isempty(index_start)
     a=1;
 end
 send_start = location_vehi(index_start,2:3);%消息传播起点车辆，只保存位置信息（x,y)
 send_end = location_vehi(1,2:3);%消息传播终点车辆
 plot(send_start(1),send_start(2),'o','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);
 hold on
%%%-----找消息传递的终点位置
 relayend = locationjucntion(1,2:3);%只保存位置信息（x,y)
 %城市情景的Linmap1m = sum((locationmark(3,1:2)-locationmark(4,1:2)).^2).^0.5/395;%计算1米在图中对应的像素间距离
 %高速路情景的Linmap1m=sum((locationmark(6,1:2)-locationmark(7,1:2)).^2).^0.5/3700;%高速路情景，计算1米在图中对应的像素间距离

 
 L_road = sum((locationmark(2:length(locationmark(:,1)),1:2)-locationmark(1:length(locationmark(:,1))-1,1:2)).^2,2).^0.5/Linmap1m;%计算在整个道路上的由mark确定的第i条路段的现实长度
 L_road_all = sum(L_road);%路段的现实总长度
 
 location_relay = zeros(100,4);%保存所有中继节点，1：序号；2，3：（x,y);4:场景1直道，2弯道，3十字路口
 relay = send_start;%只保存位置信息（x,y)
 junction = zeros(length(locationjucntion(:,1)),4);%保存所有路口信息，4：选择路口中继节点时对应的Rp
 junction(:,1:3) = locationjucntion;
 junction(1,4) = min(R/2,(sum((junction(1,2:3)-junction(2,2:3)).^2))^0.5/Linmap1m);
 junction(length(junction(:,1)),4) = min(R/2,(sum((junction(length(junction(:,1)),2:3)-junction(length(junction(:,1))-1,2:3)).^2))^0.5/Linmap1m);
 for i=2:length(junction(:,1))-1
     junction(i,4) = min(min(R/2,(sum((junction(i,2:3)-junction(i-1,2:3)).^2))^0.5/Linmap1m),(sum((junction(i,2:3)-junction(i+1,2:3)).^2))^0.5/Linmap1m);%4：每个十字路口的Rp
 end
 junction = junction(length(junction(:,1)):-1:1,:); %按离start的距离由近到远排序
 n_jucntion = 1;
 n_relay = 1;
 relay_main = relay;
 t = 0;
 count = 1;%循环计数初始为1
 d_relay_junc = 0;%初始化中继节点距路口的距离
 re_t = 0;%初始化反向广播的时间re_t为0
 n_junc_record = zeros(length(junction(:,1)),1);%初始化n_junc_record数组用来记录已经被路口中继节点选择算法选择过的路口
 rec = 1;%初始化n_junc_record数组记录参数为1
 range = min(R,((relay_main(1)-junction(n_jucntion,2)).^2+(relay_main(2)-junction(n_jucntion,3)).^2).^0.5./Linmap1m+junction(n_jucntion,4));%为保证在主道上选择的中继节点超过下一个路口的R/2而造成跳过路口的情况出现
 
 while ((sum((relay-relayend).^2))^0.5/Linmap1m>R || n_jucntion<=length(junction(:,1))) && t<=T
     
     if d_relay_junc ~= Inf
        d_set_relay_junc = zeros(length(junction(:,1)),2);%d_set_relay_junc第一列存中继节点relay与各个路口标记之间的距离，第二列存中继节点relay是否满足进入某路口条件的标记，如果满足则在其与对应路口距离位置行第二列标上1
        for i = 1:length(junction(:,1))
            d_set_relay_junc(i,1) = (sum((relay-junction(i,2:3)).^2))^0.5/Linmap1m;
            if d_set_relay_junc(i,1) <= junction(i,4)
                d_set_relay_junc(i,2) = 1;
            end
        end
        [index,~] = find(d_set_relay_junc(:,2)==1);
        if isempty(index)
            index = 0;
        end
        for i  = 1:length(index)
            if isempty(find(n_junc_record==index(i),1))
                index(index~=index(i)) = [];
                break;
            else
                index(i) = 0;
            end
        end 
        index(index==0)=[];
        if isempty(index)
            index = 0;
        end
        if n_jucntion ~= index && index ~= 0 
            d_relay_junc = d_set_relay_junc(index);
            n_jucntion = index;
        else
            d_relay_junc = (sum((relay-junction(n_jucntion,2:3)).^2))^0.5/Linmap1m;  %中继节点和其最近路口的距离
        end
     else
        n_jucntion = 1;       
     end
     if d_relay_junc<=junction(n_jucntion,4) %判断是否进入十字路口
          %%%%--------寻找路口中继节点          
        n_junc_record(rec,1) = n_jucntion;%将进入了路口中继节点选择的路口序号存入n_junc_record数组中记录起来
        rec = rec+1;
        [t,location_relay,n_relay,n_jucntion,relay_main,relay]...
            =jyc_crossroads_relay_selection(location_vehi,junction,n_jucntion,n_relay,locationbranch,locationmark,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,t,d_relay_junc,range);  
        
     else  %如果没进入路口中继节点选择，那么计算Popt点，准备根据条件进入弯道或者直道中继节点选择算法
         n_jucntion = n_jucntion+1;   %也要使得路口计数加一方便我们届时跳出while循环
         [posi_opt]=jyc_posi_opt(locationmark,R,Linmap1m,relay_main,send_end);     %找出Popt点   
         
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
         %上面计算完在Rp范围内的路段长度，并存在了road_in_Rp中
         %接下来根据在Rp中的路段长度判断是进入弯道选择还是直道选择
         if R*1.1 < road_in_Rp  %%%%----------弯道场景    
             [t,location_relay,n_relay,relay_main,relay,re_t]...
                 =jyc_curve_relay_selection(location_vehi,n_relay,locationmark,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,relay,send_end,send_start,t,T,re_t,posi_opt);         
         else    %%%%---------直道场景        
             [t,location_relay,n_relay,n_jucntion,relay_main,relay,range]...
                 =jyc_straight_relay_selection(location_vehi,junction,n_jucntion,n_relay,locationmark,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,relay_main,t,posi_opt);                 
         end        
         
     end
  
     
     if d_relay_junc == Inf
         n_jucntion = n_jucntion+length(junction(:,1)); 
     end   
     
 end
 
 if re_t>t %如果反向广播耗时大于普通广播耗时
     t = re_t;%则取反向广播耗时为广播耗时
 end