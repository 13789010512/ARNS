function  [t,location_relay,n_relay,n_jucntion,relay_main,relay,num_vehi,location_vehi]=jyc_bar_straight_relay_selection(location_vehi,junction,n_jucntion,n_relay,locationmark,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,relay_main,t,posi_opt)
 %����ֱ���м̽ڵ�ѡ���㷨��ɺ󷵻ؼ�¼ʱ���t���洢�м̽ڵ�ľ���location_relay����¼��ǰ�м̽ڵ��ǵڼ����м̽ڵ��n_relay����¼��ǰ·���ǵڼ���·�ڵ�n_jucntion����¼��·�м̽ڵ��relay_main����¼�м̽ڵ��relay,����ÿ������count
         t = t+0.37e-3;      %��¼ÿһ��ֱ�������м̽ڵ�ѡ����ӳ�

         juction_one = junction(n_jucntion,2:3);
         index_roadmark_x = find(locationmark(:,1)==juction_one(1));%Ѱ�ҵ�·�ϵ��¸���ǵ���locationmark�е�λ�ã��п������۵㣬Ҳ������·��
         index_roadmark_y = find(locationmark(:,2)==juction_one(2));
         index_roadmark = intersect(index_roadmark_x,index_roadmark_y); 
         index_roadmarkintial = index_roadmark;
          range = min(R,((relay_main(1)-junction(n_jucntion,2)).^2+(relay_main(2)-junction(n_jucntion,3)).^2).^0.5./Linmap1m+junction(n_jucntion,4));%Ϊ��֤��������ѡ����м̽ڵ㳬����һ��·�ڵ�R/2���������·�ڵ��������               
%%���� 
         while (sum((locationmark(index_roadmark,1:2)-relay_main(1:2)).^2))^0.5/Linmap1m<range
                 index_roadmark =index_roadmark-1;
         end
             while (sum((locationmark(index_roadmark+1,1:2)-relay_main(1:2)).^2))^0.5/Linmap1m>range
                 if index_roadmark==length(locationmark(:,1))-1
                     break
                 end                     
                     index_roadmark =index_roadmark+1;
             end
%%%%����??
           if index_roadmark==length(locationmark(:,1))-1   
              index_roadmark = index_roadmarkintial;
             while index_roadmark<=length(locationmark(:,1))-1
                 if (locationmark(index_roadmark+1,1)<=relay_main(1))&&(locationmark(index_roadmark,1)>relay_main(1))
                     break
                 end
                 index_roadmark = index_roadmark+1;
             end
           end
             
      
             index_vehi_part_2 = zeros(num_vehi,1);
             index_vehi_part_1 = zeros(num_vehi,1);
             Rp = sum((posi_opt-relay_main(1:2)).^2).^0.5/Linmap1m;
             index_vehi_part_1 = find(((location_vehi(:,2)-posi_opt(1)).^2+(location_vehi(:,3)-posi_opt(2)).^2).^0.5./Linmap1m<=Rp);
             index_vehi_part_2 = find(((location_vehi(:,2)-relay_main(1)).^2+(location_vehi(:,3)-relay_main(2)).^2).^0.5./Linmap1m<=Rp & ((location_vehi(:,2)-relay_main(1)).^2+(location_vehi(:,3)-relay_main(2)).^2).^0.5./Linmap1m>0);
             index_vehi_part =intersect(index_vehi_part_1,index_vehi_part_2);
             index_vehi_part(index_vehi_part==0) = [];
             
             if isempty(index_vehi_part)
                 relay = posi_opt;            %����һ����Χ��û���м̽ڵ㣬������Popt��Ϊ�м̽ڵ㲢����������һ
                 num_vehi = num_vehi + 1;
                 location_vehi(num_vehi,1) = num_vehi;%����location_vehi���������еĳ�����Ϣ
                 location_vehi(num_vehi,2:3) = posi_opt(1:2);
                 location_vehi(num_vehi,4) = 0;
             else
                 inform_vehi_part = zeros(length(index_vehi_part),4);
                 inform_vehi_part = location_vehi(index_vehi_part,1:4); 
                 relay = partition_exp(posi_opt,Rp,inform_vehi_part,N_iter,N_part,A,Linmap1m);
             end
             
                     
             
             plot(relay(1),relay(2),'^','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);
             hold on
                      
              location_relay(n_relay,1) = n_relay;
              location_relay(n_relay,2:3) = relay;
              location_relay(n_relay,4) = 1;
              n_relay = n_relay+1;   
              relay_main = relay;
              
     end