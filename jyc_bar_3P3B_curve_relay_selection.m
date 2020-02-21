function  [t,location_relay,n_relay,relay_main,relay,num_vehi,location_vehi,n_re_relay,PDR,last_t]=jyc_bar_3P3B_curve_relay_selection(location_vehi_in_Rp,location_vehi,n_relay,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,relay,t,posi_opt,n_re_relay,locationmark,ima2,send_end,old_sender,old_posi_opt)
%������������м̽ڵ�ѡ���㷨���㷨��ɺ󷵻ؼ�¼ʱ���t���洢�м̽ڵ�ľ���location_relay����¼��ǰ�м̽ڵ��ǵڼ����м̽ڵ��n_relay����¼��·�м̽ڵ��relay_main����¼�м̽ڵ��relay����¼�������㲥�ĺ�ʱre_t
         global density_EM CW_CTB;
         sender = relay;%��relay��ֵ����sender��������Ϊ֮��relay�ᷢ�����£�������Ҫ��relay������������ʱ����
         recover = 0;%��ʼ���ظ����ǲ���Ϊ0��1��ʾ�����ظ����ǵ������0��ʾ�������ظ����ǵ����         
         Rp = R;                     %sum((posi_opt-relay_main(1:2)).^2).^0.5/Linmap1m
         if isempty(location_vehi_in_Rp)
             location_vehi_in_Rp = zeros(1,4);
             location_vehi_in_Rp(1,1) = 1;        
         end
         num_vehi_in_Rp = length(location_vehi_in_Rp(:,1));        
         j_re = 1;
         j_op = 1;
         index_in_relay = zeros(1,num_vehi_in_Rp);    %��ʼ���м̽ڵ㷶Χ�ڵĳ������
         index_in_popt = zeros(1,num_vehi_in_Rp);      %��ʼ��Popt��Χ�ڵĳ������       
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
         index_in_relay(index_in_relay==0) = [];%ȥ������Ϊ0��Ԫ��
         index_in_popt(index_in_popt==0) = [];%ȥ������Ϊ0��Ԫ��
         index_vehi_part = intersect(index_in_relay,index_in_popt);
         
          %�������һ����Χ��û���м̽ڵ㣬�����posi_optΪ�����ڵ㣬��������һ
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
         location_relay(n_relay,4) = 2;%location_relay(n_relay,4) = 2 ��ʾ��������µ��м̽ڵ�ѡ��
         n_relay = n_relay+1;    
         relay_main = relay;



end