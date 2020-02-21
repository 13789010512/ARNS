function  [t,location_relay,n_relay,relay_main,relay,num_vehi,location_vehi,n_re_relay,PDR,last_t]=jyc_bar_3P3B_curve_relay_selection(location_vehi_in_Rp,location_vehi,n_relay,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,relay,t,posi_opt,n_re_relay,locationmark,ima2,send_end,old_sender,old_posi_opt)
%做的是弯道的中继节点选择算法，算法完成后返回记录时间的t，存储中继节点的矩阵location_relay，记录当前中继节点是第几个中继节点的n_relay，记录主路中继节点的relay_main，记录中继节点的relay，记录弯道反向广播的耗时re_t
         global density_EM CW_CTB;
         sender = relay;%将relay的值赋予sender参数，因为之后relay会发生更新，我们需要老relay的数据来做延时计算
         recover = 0;%初始化重复覆盖参数为0，1表示存在重复覆盖的情况，0表示不存在重复覆盖的情况         
         Rp = R;                     %sum((posi_opt-relay_main(1:2)).^2).^0.5/Linmap1m
         if isempty(location_vehi_in_Rp)
             location_vehi_in_Rp = zeros(1,4);
             location_vehi_in_Rp(1,1) = 1;        
         end
         num_vehi_in_Rp = length(location_vehi_in_Rp(:,1));        
         j_re = 1;
         j_op = 1;
         index_in_relay = zeros(1,num_vehi_in_Rp);    %初始化中继节点范围内的车辆序号
         index_in_popt = zeros(1,num_vehi_in_Rp);      %初始化Popt范围内的车辆序号       
         for i = 1:num_vehi_in_Rp
             if (sum((location_vehi_in_Rp(i,2:3)-relay(1:2)).^2))^0.5/Linmap1m<R && (sum((location_vehi_in_Rp(i,2:3)-relay(1:2)).^2))^0.5/Linmap1m>0
                 index_in_relay(j_re) = location_vehi_in_Rp(i,1);
                 j_re = j_re + 1;
             end
         end
         for i = 1:num_vehi_in_Rp
             if (sum((location_vehi_in_Rp(i,2:3)-posi_opt(1:2)).^2))^0.5/Linmap1m<R
                 index_in_popt(j_op) = location_vehi_in_Rp(i,1);
                 j_op = j_op + 1;
             end
         end
         index_in_relay(index_in_relay==0) = [];%去掉里面为0的元素
         index_in_popt(index_in_popt==0) = [];%去掉里面为0的元素
         index_vehi_part = intersect(index_in_relay,index_in_popt);
         
          %考虑如果一跳范围内没有中继节点，则添加posi_opt为车辆节点，车辆数加一
         if isempty(index_vehi_part)
             [location_vehi,num_vehi,relay]=add_vehi(locationmark,relay,Linmap1m,ima2,posi_opt,send_end,R,num_vehi,location_vehi,old_sender,old_posi_opt);            
         else             
             inform_vehi_part = zeros(length(index_vehi_part),4);
             inform_vehi_part = location_vehi(index_vehi_part,1:4);
             relay = partition_exp(posi_opt,Rp,inform_vehi_part,N_iter,N_part,A,Linmap1m);    
         end
                

         [delay_one_hop_average,PDR,relay] = jyc_3P3B_delay_calcu (location_vehi_in_Rp,density_EM,N_iter,N_part,CW_CTB,sender,posi_opt,relay);
         t = t + delay_one_hop_average;
         last_t = delay_one_hop_average;         
%          plot(relay(1),relay(2),'^','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);
%          hold on

         location_relay(n_relay,1) = n_relay;
         location_relay(n_relay,2:3) = relay;
         location_relay(n_relay,4) = 2;%location_relay(n_relay,4) = 2 表示弯道场景下的中继节点选择
         n_relay = n_relay+1;    
         relay_main = relay;



end