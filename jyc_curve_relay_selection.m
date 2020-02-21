function  [t,location_relay,n_relay,relay_main,relay,re_t]=jyc_curve_relay_selection(location_vehi,n_relay,locationmark,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,relay,send_end,send_start,t,T,re_t,posi_opt)
%做的是弯道的中继节点选择算法，算法完成后返回记录时间的t，存储中继节点的矩阵location_relay，记录当前中继节点是第几个中继节点的n_relay，记录主路中继节点的relay_main，记录中继节点的relay，记录弯道反向广播的耗时re_t


         t = t+0.4e-3 ;
        
         
         Rp = R;                     %sum((posi_opt-relay_main(1:2)).^2).^0.5/Linmap1m
                
         j_re = 1;
         j_op = 1;
         index_in_relay = zeros(1,num_vehi);    %初始化中继节点范围内的车辆序号
         index_in_popt = zeros(1,num_vehi);      %初始化Popt范围内的车辆序号       
         for i = 1:num_vehi
             if (sum((location_vehi(i,2:3)-relay(1:2)).^2))^0.5/Linmap1m<R && (sum((location_vehi(i,2:3)-relay(1:2)).^2))^0.5/Linmap1m>0
                 index_in_relay(j_re) = location_vehi(i,1);
                 j_re = j_re + 1;
             end
         end
         for i = 1:num_vehi
             if (sum((location_vehi(i,2:3)-posi_opt(1:2)).^2))^0.5/Linmap1m<R
                 index_in_popt(j_op) = location_vehi(i,1);
                 j_op = j_op + 1;
             end
         end
         index_in_relay(index_in_relay==0) = [];%去掉里面为0的元素
         index_in_popt(index_in_popt==0) = [];%去掉里面为0的元素
         index_vehi_part = intersect(index_in_relay,index_in_popt);
         
          %考虑如果一跳范围内没有中继节点，则添加posi_opt为车辆节点，车辆数加一
         if isempty(index_vehi_part)
             relay = posi_opt;
             num_vehi = num_vehi + 1;
         else
             inform_vehi_part = zeros(length(index_vehi_part),4);
             inform_vehi_part = location_vehi(index_vehi_part,1:4);
             relay = partition_exp(posi_opt,Rp,inform_vehi_part,N_iter,N_part,A,Linmap1m);    
         end
                
         plot(relay(1),relay(2),'^','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);
         hold on
         
         location_relay(n_relay,1) = n_relay;
         location_relay(n_relay,2:3) = relay;
         location_relay(n_relay,4) = 2;%location_relay(n_relay,4) = 2 表示弯道场景下的中继节点选择
         n_relay = n_relay+1;    
         relay_main = relay;
         
         %接下来考虑反向广播情况
         %根据判断X坐标在send和relay中间的路段是否被广播覆盖来判断是否出现空白区域  
         vacant = zeros(length(locationmark(:,1)),4);%初始化4列的vacant，第一二列用来存储x,y坐标，第三列存0用来求d_vacant（没办法d_vacant的求取必须要从三维的角度求得，vacant的z轴默认为0），第四列存储vacant的路段序号
         re_symbol = 0;   %初始化反向广播标志位为0
         i_count = 1;     %vacant数组存储计数
         for i = 1:length(locationmark(:,1))
             if n_relay==2 && locationmark(i,1)<location_relay(n_relay-1,2) && locationmark(i,1)>send_start(1,1)
                 if (sum((locationmark(i,1:2)-location_relay(n_relay-1,2:3)).^2))^0.5/Linmap1m>R && (sum((locationmark(i,1:2)-send_start(1,1:2)).^2))^0.5/Linmap1m>R
                     re_symbol = 1;
                     vacant(i_count,1:2) = locationmark(i,1:2);
                     vacant(i_count,4) = i;
                     i_count = i_count + 1;
                 end
             elseif n_relay>2 && locationmark(i,1)<location_relay(n_relay-1,2) && locationmark(i,1)>location_relay(n_relay-2,2)
                 if (sum((locationmark(i,1:2)-location_relay(n_relay-1,2:3)).^2))^0.5/Linmap1m>R && (sum((locationmark(i,1:2)-location_relay(n_relay-2,2:3)).^2))^0.5/Linmap1m>R
                     re_symbol = 1;
                     vacant(i_count,1:2) = locationmark(i,1:2);
                     vacant(i_count,4) = i;
                     i_count = i_count + 1;
                 end
             end
         end 
         vacant0 = vacant;
         vacant0(vacant==0) = [];
         if re_symbol == 1 %如果反向广播标志位为1，进入反向广播          
             r = length(vacant0)/3;%vacant（或vacant0）数组的有效行数，即非零行数
             r_end = length(vacant(:,1));
             for i = r+1:r_end
                 vacant(r+1,:)=[];%清除掉vacant数组中第r行之后的行数
             end
       
             
             if re_t == 0
                 re_t = t;  %如果反向广播的计时re_t为0，表示之前没有进入过反向广播，则初始化反向广播的计时从t开始
             end
             if n_relay==2
                 send_start(1,3) = 0;
                 location_relay(n_relay-1,4) = 0;
                 vacant(:,3) = 0;
                 for i = 1:length(vacant(:,1))
                     d_vacant(i) = norm(cross(send_start(1:3)-location_relay(n_relay-1,2:4),vacant(i,1:3)-location_relay(n_relay-1,2:4)))/norm(send_start(1:3)-location_relay(n_relay-1,2:4));  %求出vacant中的路段点距以sender和relay为两端点的线段的距离  
                   
                 end
             elseif n_relay>2
                 location_relay(n_relay-1,4) = 0;
                 location_relay(n_relay-2,4) = 0;
                 vacant(:,3) = 0;                 
                 for i = 1:length(vacant(:,1))
                     d_vacant(i) = norm(cross(location_relay(n_relay-2,2:4)-location_relay(n_relay-1,2:4),vacant(i,1:3)-location_relay(n_relay-1,2:4)))/norm(location_relay(n_relay-2,2:4)-location_relay(n_relay-1,2:4));  %求出vacant中的路段点距以sender和relay为两端点的线段的距离
                 end
             end
             d_vacant_max = max(d_vacant);                  %找出vacant中最远路段与send、relay连线的距离d_vacant_max
             [~,row_max] = find(d_vacant==d_vacant_max);    %找出vacant中距离最远的路段点的行下标             
             re_send_start = location_relay(n_relay-1,2:3);
             re_send_end(1,1:2) = vacant(row_max,1:2);             %最远路段点re_send_end即为我们反向广播的终点，其中re_send_end的第一二列为x,y轴坐标
             re_send_end(1,3) = vacant(row_max,4);                 %re_send_end第三列为反向广播终点所在的路段
             plot(re_send_end(1),re_send_end(2),'o','LineWidth',1,'MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',6);%标出反向广播的终点位置
             hold on
             re_relay = re_send_start;
             re_relayend = re_send_end;

             re_relay_main = re_relay;
             n_re_relay = 1;
             %反向广播与正常的弯道广播方案一致,319到398行都是反向广播情景
             while (sum((re_relay(1:2)-re_relayend(1:2)).^2))^0.5/Linmap1m>R && re_t <= T
                 
                 re_t = re_t+0.4e-3 ;  
                 
                [posi_opt]=jyc_posi_opt(locationmark,R,Linmap1m,re_relay_main,re_send_end);
         
                 Rp = sum((posi_opt-re_relay_main(1:2)).^2).^0.5/Linmap1m;
                
                 j_re = 1;
                 j_op = 1;
                 index_in_re_relay = zeros(1,num_vehi);    %初始化中继节点范围内的车辆序号
                 index_in_popt = zeros(1,num_vehi);      %初始化Popt范围内的车辆序号       
                 for i = 1:num_vehi
                     if (sum((location_vehi(i,2:3)-re_relay(1:2)).^2))^0.5/Linmap1m<R && (sum((location_vehi(i,2:3)-re_relay(1:2)).^2))^0.5/Linmap1m>0
                        index_in_re_relay(j_re) = location_vehi(i,1);
                        j_re = j_re + 1;
                     end
                 end
                 for i = 1:num_vehi
                     if (sum((location_vehi(i,2:3)-posi_opt(1:2)).^2))^0.5/Linmap1m<R
                         index_in_popt(j_op) = location_vehi(i,1);
                         j_op = j_op + 1;
                     end
                 end
                 index_in_re_relay(index_in_re_relay==0) = [];%去掉里面为0的元素
                 index_in_popt(index_in_popt==0) = [];%去掉里面为0的元素
                 index_vehi_part = intersect(index_in_re_relay,index_in_popt);
         
          %考虑如果一跳范围内没有中继节点，则添加posi_opt为车辆节点，车辆数加一，下面都是反向广播的re_relay地图标注代码
                 if isempty(index_vehi_part)
                     re_relay = posi_opt;
                     num_vehi = num_vehi + 1;
                 else
                     inform_vehi_part = zeros(length(index_vehi_part),4);
                     inform_vehi_part = location_vehi(index_vehi_part,1:4);
                     re_relay = partition_exp(posi_opt,Rp,inform_vehi_part,N_iter,N_part,A,Linmap1m);    
                 end
                
                 plot(re_relay(1),re_relay(2),'v','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','r','MarkerSize',6);
                 hold on
         
                 location_re_relay(n_re_relay,1) = n_re_relay;
                 location_re_relay(n_re_relay,2:3) = re_relay;
                 location_re_relay(n_re_relay,4) = 2;%location_re_relay(n_re_relay,4) = 2 表示弯道场景下的中继节点选择
                 n_re_relay = n_re_relay+1;    
                 re_relay_main = re_relay;
             end
             if (sum((re_relay(1:2)-re_relayend(1:2)).^2))^0.5/Linmap1m <= R
                 re_relay = re_relayend;
                 plot(re_relay(1),re_relay(2),'v','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','r','MarkerSize',6);
                 hold on
                 location_re_relay(n_re_relay,1) = n_re_relay;
                 location_re_relay(n_re_relay,2:3) = re_relay(1:2);
                 location_re_relay(n_re_relay,4) = 2;%location_re_relay(n_re_relay,4) = 2 表示弯道场景下的中继节点选择
                 n_re_relay = n_re_relay+1;    
                 re_relay_main = re_relay;
             end
             
         end
            