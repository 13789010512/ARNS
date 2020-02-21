function  [t,location_relay,n_relay,relay_main,relay,re_t]=jyc_curve_relay_selection(location_vehi,n_relay,locationmark,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,relay,send_end,send_start,t,T,re_t,posi_opt)
%������������м̽ڵ�ѡ���㷨���㷨��ɺ󷵻ؼ�¼ʱ���t���洢�м̽ڵ�ľ���location_relay����¼��ǰ�м̽ڵ��ǵڼ����м̽ڵ��n_relay����¼��·�м̽ڵ��relay_main����¼�м̽ڵ��relay����¼�������㲥�ĺ�ʱre_t


         t = t+0.4e-3 ;
        
         
         Rp = R;                     %sum((posi_opt-relay_main(1:2)).^2).^0.5/Linmap1m
                
         j_re = 1;
         j_op = 1;
         index_in_relay = zeros(1,num_vehi);    %��ʼ���м̽ڵ㷶Χ�ڵĳ������
         index_in_popt = zeros(1,num_vehi);      %��ʼ��Popt��Χ�ڵĳ������       
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
         index_in_relay(index_in_relay==0) = [];%ȥ������Ϊ0��Ԫ��
         index_in_popt(index_in_popt==0) = [];%ȥ������Ϊ0��Ԫ��
         index_vehi_part = intersect(index_in_relay,index_in_popt);
         
          %�������һ����Χ��û���м̽ڵ㣬�����posi_optΪ�����ڵ㣬��������һ
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
         location_relay(n_relay,4) = 2;%location_relay(n_relay,4) = 2 ��ʾ��������µ��м̽ڵ�ѡ��
         n_relay = n_relay+1;    
         relay_main = relay;
         
         %���������Ƿ���㲥���
         %�����ж�X������send��relay�м��·���Ƿ񱻹㲥�������ж��Ƿ���ֿհ�����  
         vacant = zeros(length(locationmark(:,1)),4);%��ʼ��4�е�vacant����һ���������洢x,y���꣬�����д�0������d_vacant��û�취d_vacant����ȡ����Ҫ����ά�ĽǶ���ã�vacant��z��Ĭ��Ϊ0���������д洢vacant��·�����
         re_symbol = 0;   %��ʼ������㲥��־λΪ0
         i_count = 1;     %vacant����洢����
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
         if re_symbol == 1 %�������㲥��־λΪ1�����뷴��㲥          
             r = length(vacant0)/3;%vacant����vacant0���������Ч����������������
             r_end = length(vacant(:,1));
             for i = r+1:r_end
                 vacant(r+1,:)=[];%�����vacant�����е�r��֮�������
             end
       
             
             if re_t == 0
                 re_t = t;  %�������㲥�ļ�ʱre_tΪ0����ʾ֮ǰû�н��������㲥�����ʼ������㲥�ļ�ʱ��t��ʼ
             end
             if n_relay==2
                 send_start(1,3) = 0;
                 location_relay(n_relay-1,4) = 0;
                 vacant(:,3) = 0;
                 for i = 1:length(vacant(:,1))
                     d_vacant(i) = norm(cross(send_start(1:3)-location_relay(n_relay-1,2:4),vacant(i,1:3)-location_relay(n_relay-1,2:4)))/norm(send_start(1:3)-location_relay(n_relay-1,2:4));  %���vacant�е�·�ε����sender��relayΪ���˵���߶εľ���  
                   
                 end
             elseif n_relay>2
                 location_relay(n_relay-1,4) = 0;
                 location_relay(n_relay-2,4) = 0;
                 vacant(:,3) = 0;                 
                 for i = 1:length(vacant(:,1))
                     d_vacant(i) = norm(cross(location_relay(n_relay-2,2:4)-location_relay(n_relay-1,2:4),vacant(i,1:3)-location_relay(n_relay-1,2:4)))/norm(location_relay(n_relay-2,2:4)-location_relay(n_relay-1,2:4));  %���vacant�е�·�ε����sender��relayΪ���˵���߶εľ���
                 end
             end
             d_vacant_max = max(d_vacant);                  %�ҳ�vacant����Զ·����send��relay���ߵľ���d_vacant_max
             [~,row_max] = find(d_vacant==d_vacant_max);    %�ҳ�vacant�о�����Զ��·�ε�����±�             
             re_send_start = location_relay(n_relay-1,2:3);
             re_send_end(1,1:2) = vacant(row_max,1:2);             %��Զ·�ε�re_send_end��Ϊ���Ƿ���㲥���յ㣬����re_send_end�ĵ�һ����Ϊx,y������
             re_send_end(1,3) = vacant(row_max,4);                 %re_send_end������Ϊ����㲥�յ����ڵ�·��
             plot(re_send_end(1),re_send_end(2),'o','LineWidth',1,'MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',6);%�������㲥���յ�λ��
             hold on
             re_relay = re_send_start;
             re_relayend = re_send_end;

             re_relay_main = re_relay;
             n_re_relay = 1;
             %����㲥������������㲥����һ��,319��398�ж��Ƿ���㲥�龰
             while (sum((re_relay(1:2)-re_relayend(1:2)).^2))^0.5/Linmap1m>R && re_t <= T
                 
                 re_t = re_t+0.4e-3 ;  
                 
                [posi_opt]=jyc_posi_opt(locationmark,R,Linmap1m,re_relay_main,re_send_end);
         
                 Rp = sum((posi_opt-re_relay_main(1:2)).^2).^0.5/Linmap1m;
                
                 j_re = 1;
                 j_op = 1;
                 index_in_re_relay = zeros(1,num_vehi);    %��ʼ���м̽ڵ㷶Χ�ڵĳ������
                 index_in_popt = zeros(1,num_vehi);      %��ʼ��Popt��Χ�ڵĳ������       
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
                 index_in_re_relay(index_in_re_relay==0) = [];%ȥ������Ϊ0��Ԫ��
                 index_in_popt(index_in_popt==0) = [];%ȥ������Ϊ0��Ԫ��
                 index_vehi_part = intersect(index_in_re_relay,index_in_popt);
         
          %�������һ����Χ��û���м̽ڵ㣬�����posi_optΪ�����ڵ㣬��������һ�����涼�Ƿ���㲥��re_relay��ͼ��ע����
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
                 location_re_relay(n_re_relay,4) = 2;%location_re_relay(n_re_relay,4) = 2 ��ʾ��������µ��м̽ڵ�ѡ��
                 n_re_relay = n_re_relay+1;    
                 re_relay_main = re_relay;
             end
             if (sum((re_relay(1:2)-re_relayend(1:2)).^2))^0.5/Linmap1m <= R
                 re_relay = re_relayend;
                 plot(re_relay(1),re_relay(2),'v','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','r','MarkerSize',6);
                 hold on
                 location_re_relay(n_re_relay,1) = n_re_relay;
                 location_re_relay(n_re_relay,2:3) = re_relay(1:2);
                 location_re_relay(n_re_relay,4) = 2;%location_re_relay(n_re_relay,4) = 2 ��ʾ��������µ��м̽ڵ�ѡ��
                 n_re_relay = n_re_relay+1;    
                 re_relay_main = re_relay;
             end
             
         end
            