function  [t,location_relay,relay,PDR,location_vehi,num_vehi]=jyc_bar_complete_relay_selection(location_vehi,locationmark,R,num_vehi,N_iter,N_part,A,Linmap1m,T,ima2,t_relay)
%%%���ɵ�·������Ϣ������ʾ����·������ʮ��·��
 global locationjunction density_EM CW_CTB;
 send_start(1,1:2) = locationmark(length(locationmark(:,1)),1:2);%ֱ��ȡ·����������Ŀ�ĵ㣬����Ϣ���Ͷ˵�
 send_start(1,3) = length(locationmark(:,1));
                    %%%-----����start����Ľڵ�����Ϣ�ĳ�ʼ���ͽڵ�
                    %dis_vehi_start = ((location_vehi(:,2)-start(1)).^2+(location_vehi(:,3)-start(2)).^2).^0.5;%����start����Ľڵ�����Ϣ�ĳ�ʼ���ͽڵ�?
                    %index_start = find(dis_vehi_start==min(dis_vehi_start));
                    %if isempty(index_start)
                    %    a=1;
                    %end
                    %send_start = location_vehi(index_start,2:3);%��Ϣ������㳵����ֻ����λ����Ϣ��x,y)
 %��Ϣ���������յ㶼���ó�·�α�ǵ�
 send_end = locationmark(1,1:2);%��Ϣ�����յ㳵��
 send_end(1,3) = 1;
 isend = 0;%isend����������ʾsend_end���ǲ����ս�junction�㣬��ʼ��Ϊ0�������ǣ�������ս�junction�㣬��ôisendֵΪ1
 for i = 1:11%�ս�junction�㶼������locationjunction�����ǰ11��
     if locationjunction(i,2) == send_end(1,1) && locationjunction(i,3) == send_end(1,2)%���send_end���ս�junction��
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
%%%-----����Ϣ���ݵ��յ�λ��
 %relayend = locationjucntion(1,2:3);%ֻ����λ����Ϣ��x,y)
  relayend = send_end(1:2);
  bar_bef_end = 0;%relay��relayend֮������ϰ���ı�־λ��ʼ��Ϊ0��0��ʾ������relay��relayend֮����ϰ��1��ʾ����
 %�����龰��Linmap1m = sum((locationmark(3,1:2)-locationmark(4,1:2)).^2).^0.5/395;%����1����ͼ�ж�Ӧ�����ؼ����
 %����·�龰��Linmap1m=sum((locationmark(6,1:2)-locationmark(7,1:2)).^2).^0.5/3700;%����·�龰������1����ͼ�ж�Ӧ�����ؼ����
 locationmark_cover = zeros(length(locationmark(:,1)),3);%��ʼ��·�α�ǵ㸲�����飬���������ѱ��㲥���ǵ�·�α�ǵ�
 locationmark_nocover = zeros(length(locationmark(:,1)),3);%��ʼ��·�α�ǵ�δ���������飬�����洢δ���㲥���ǵ�·�α�ǵ�
 locationmark_nocover(:,1) = 1:length(locationmark(:,1));%��δ���㲥���ǵ�·�α�ǵ����鸳ֵ
 locationmark_nocover(:,2:3) = locationmark(:,1:2);%��δ���㲥���ǵ�·�α�ǵ����鸳ֵ
 %�����ѭ����������·�α�ǵ��״̬���������㱻����������·�α�ǵ��δ������·�α�ǵ�������ѡ�������ѱ�����·�α�ǵ������У�����δ������·�α�ǵ������������Ϣ�ü�����10000
 %��֮��Ĵ���������ѭ���Ľṹ�ᾭ�����õ�
 for i = 1:length(locationmark_nocover(:,1))
     crossbar = 0;%��ʼ����Խ�ϰ����־λ��0
     dis_point_to_sendstart = (sum((locationmark_nocover(i,2:3)-send_start(1:2)).^2))^0.5/Linmap1m;%dis_point_to_sendstart��δ�����ǵ�·�α�ǵ㵽send_start�ľ���
     if dis_point_to_sendstart<=R %���δ�����ǵ�·�α�ǵ㵽send_start�ľ���С��ͨ�Ű뾶R
         sample_num = dis_point_to_sendstart/10;%sample_num�ǳ���������������������Ϊÿ10m����һ��
         detx = (locationmark_nocover(i,2)-send_start(1))/sample_num;%ÿ�γ���x����ĵݽ���
         dety = (locationmark_nocover(i,3)-send_start(2))/sample_num;%ÿ�γ���y����ĵݽ���
         for n = 1:sample_num
             templex = ceil(send_start(1)+n*detx);%ÿ�γ�����x���꣨���ȡ����
             templey = ceil(send_start(2)+n*dety);%ÿ�γ�����y���꣨���ȡ����
             if ima2(templey,templex)==0%������ϰ����赲
                 crossbar = 1;%��Խ�ϰ����־λ��1
                 break;
             end            
         end
         if crossbar == 0%�����·�α�ǵ�δ���ϰ����谭��·�α�ǵ���send_start�����߲������ϰ�������ڹ㲥ͨ�Ÿ��Ƿ�Χ�ڣ���һ��if����������˵���õ�Ӧ������
             locationmark_cover(i,:) = locationmark_nocover(i,:);%���õ����Ϣת�Ƶ��ѱ�����������
             locationmark_nocover(i,:) = 10000;%��δ�����������иõ�λ�õ���Ϣ�ü�����10000
         end
     end
 end
 %·�α�ǵ��״̬�������
 
 
 L_road = sum((locationmark(2:length(locationmark(:,1)),1:2)-locationmark(1:length(locationmark(:,1))-1,1:2)).^2,2).^0.5/Linmap1m;%������������·�ϵ���markȷ���ĵ�i��·�ε���ʵ����
 L_road_all = sum(L_road);%·�ε���ʵ�ܳ���
 
 location_relay = zeros(100,4);%���������м̽ڵ㣬1����ţ� 2��3��(x,y)�� 4������(0���������1ֱ����2�����3ʮ��·��)
 n_relay = 1;%�����㲥����ű��
 location_re_relay = zeros(100,4);%�������еķ���㲥�м̽ڵ㣬1����ţ� 2��3��(x,y)�� 4������(0���������1ֱ����2�����3ʮ��·��)
 n_re_relay = 1;%����㲥����ű�� 
 relay = send_start(1,1:2);%ֻ����λ����Ϣ��x,y)
 
 old_sender = [];
 sender = [];
 posi_opt = [];
 old_posi_opt = [];%��ʼ������Ϊ��
 
 has_cov_symbol = 0;%��ʼ���հ������Ѿ������ǵı��λΪ0
 relay_main = relay;
 t = t_relay;
 re_t = 0;%��ʼ������㲥��ʱ��re_tΪ0
 PDR_set = [];%��ʼ��PDRΪ��
 last_t = 0;%��ʼ�����һ���ĺ�ʱ����������Ҫ�����һ��relay������������������һ�������Ƕ�ԭ�����и��ģ�ʱ�������ڱ����������������relay�ĺ�ʱ�������ø���ǰ���ܺ�ʱt��ȥlast_t����ټ��ϸ�������º�ʱ�����ø������׼ȷ�ܺ�ʱ
 
 while ((isend == 1 && (bar_bef_end==1 || (sum((relay-relayend).^2))^0.5/Linmap1m>R)) || (isend == 0 && (bar_bef_end==1 || (sum((relay-relayend).^2))^0.5/Linmap1m>R/2))) && t<T 
     %������������Ҫ���㣺ѡ����ͨjunction�㣨relayend�㣩R/2��Χ�ڵĳ���Ϊ�м̽ڵ㣨���ս�Junction��R��Χ�ڵĳ���Ϊ�м̽ڵ㣩��relay��relayend֮�䲻�ܴ����ϰ�����ܽ�����ѭ��
     old_posi_opt = posi_opt;
     old_sender = sender;
     sender = relay;
     [posi_opt,location_vehi_in_Rp]=jyc_bar_posi_opt(location_vehi,locationmark,R,Linmap1m,relay_main,send_end,ima2);     %�ҳ�Popt��
     %������Rp��Χ�ڵ�·�γ���
     road_in_Rp = 0; %��ʼ��Rp��Χ�ڵ�·�γ���Ϊ0
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
         l_relay_road(i) = (sum((relay-locationmark(part_in_Rp(i),1:2)).^2))^0.5/Linmap1m;%relay��������������·�ε��յ�ľ���
         l_Popt_road(i) = (sum((posi_opt-locationmark(part_in_Rp(i),1:2)).^2))^0.5/Linmap1m;%Popt��������������·�ε����ľ���
     end
     relay_road = min(l_relay_road);
     Popt_road = min(l_Popt_road);
     road_in_Rp = relay_road+Popt_road;             %road_in_Rp���ȼ�����β�εĳ���
     for i = 1:length(part_in_Rp)-1
         if part_in_Rp(i)+1 == part_in_Rp(i+1)%�����i��part_in_Rp�����д洢��·�ε����i+1��part_in_Rp�����д洢��·�ε�����
             road_in_Rp = road_in_Rp+L_road(part_in_Rp(i));%��Rp��Χ�ڵ�·�γ���road_in_Rp�ͼ��ϸ�·�εĳ���L_road(i)
         end
     end
     if has_cov_symbol == 0
         [vacant0,vacant,re_symbol]=jyc_vacantbool(n_relay,locationmark_nocover,location_relay,send_start,locationmark);
     else%���has_cov_symbol���λΪ1
         re_symbol = 0;%����㲥���λ��0
         has_cov_symbol = 0;%has_cov_symbol���ã���һ�ε�re_symbol��0
     end
     %�����������Rp��Χ�ڵ�·�γ��ȣ���������road_in_Rp��
     %������������Rp�е�·�γ����ж��ǽ������ѡ����ֱ��ѡ��
     if re_symbol==0
         [t,location_relay,n_relay,relay_main,relay,num_vehi,location_vehi,n_re_relay,PDR_part,last_t]...
             =jyc_bar_curve_relay_selection(location_vehi_in_Rp,location_vehi,n_relay,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,relay,t,posi_opt,n_re_relay,locationmark,ima2,send_end,old_sender,old_posi_opt);
     else
         [location_relay,re_t,num_vehi,location_vehi,location_re_relay,n_re_relay,has_cov_symbol,PDR_part,last_t]...
             =jyc_bar_reverse_relay_selection(location_vehi,n_relay,locationmark,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,send_start,t,re_t,location_re_relay,n_re_relay,vacant0,vacant,re_symbol,relay_main,has_cov_symbol,T,ima2);
         %�����������㲥jyc_bar_reverse_relay_selection��������ô��vacant0�д洢��·�α�ǵ��Ӧlocationmark_nocover��locationmark_cover�е�·�α�ǵ���Ϣ������һ��
         %��vacant0�д����·�α�ǵ�ȫ�������ˣ�����Ϊ����locationmark_nocover��locationmark_cover�������
         for num = 1:length(vacant0(:,1))
             locationmark_cover(vacant0(num,3),:) = locationmark_nocover(vacant0(num,3),:);
             locationmark_nocover(vacant0(num,3),:) = 10000;
         end
     end 
     PDR_set = [PDR_set;PDR_part];
     %ÿ����������·�α�ǵ㸲�����
     for i = 1:length(locationmark_nocover(:,1))
         crossbar = 0;%��ʼ����Խ�ϰ����־λ��0
         dis_point_to_relay = (sum((locationmark_nocover(i,2:3)-relay(1:2)).^2))^0.5/Linmap1m;%dis_point_to_relay��δ�����ǵ�·�α�ǵ㵽relay�ľ���
         if dis_point_to_relay<=R %���δ�����ǵ�·�α�ǵ㵽relay�ľ���С��ͨ�Ű뾶R
             sample_num = dis_point_to_relay/10;%sample_num�ǳ���������������������Ϊÿ10m����һ��
             detx = (locationmark_nocover(i,2)-relay(1))/sample_num;%ÿ�γ���x����ĵݽ���
             dety = (locationmark_nocover(i,3)-relay(2))/sample_num;%ÿ�γ���y����ĵݽ���
             for n = 1:sample_num
                 templex = ceil(relay(1)+n*detx);%ÿ�γ�����x���꣨���ȡ����
                 templey = ceil(relay(2)+n*dety);%ÿ�γ�����y���꣨���ȡ����
                 if ima2(templey,templex)==0%������ϰ����赲
                     crossbar = 1;%��Խ�ϰ����־λ��1
                     break;
                 end
             end
             if crossbar == 0%�����·�α�ǵ�δ���ϰ����谭��·�α�ǵ���relay�����߲������ϰ�������ڹ㲥ͨ�Ÿ��Ƿ�Χ�ڣ���һ��if����������˵���õ�Ӧ������
                 locationmark_cover(i,:) = locationmark_nocover(i,:);%���õ����Ϣת�Ƶ��ѱ�����������
                 locationmark_nocover(i,:) = 10000;%��δ�����������иõ�λ�õ���Ϣ�ü�����10000
             end
         end
     end    
     if (sum((relay-relayend).^2))^0.5/Linmap1m<=R/2%����Ѿ�������relay��relayend֮��ľ���С��R/2��׼��Ҫ����whileѭ���ˣ����������һ��relay��relayend֮���Ƿ�����ϰ���
         relay_to_relayend = (sum((relay-relayend).^2))^0.5/Linmap1m;%relay_to_relayend��relay�㵽relayend��֮��ľ���
         sample_num = relay_to_relayend/10;%sample_num�ǳ���������������������Ϊÿ10m����һ��
         detx = (relayend(1)-relay(1))/sample_num;%ÿ�γ���x����ĵݽ���
         dety = (relayend(2)-relay(2))/sample_num;%ÿ�γ���y����ĵݽ���
         for n = 1:sample_num
             templex = ceil(relay(1)+n*detx);%ÿ�γ�����x���꣨���ȡ����
             templey = ceil(relay(2)+n*dety);%ÿ�γ�����y���꣨���ȡ����
             if ima2(templey,templex)==0%������ϰ����赲
                 bar_bef_end = 1;%relayendǰ�Ƿ�����ϰ����־λ��1����ʾrelay��relayend֮������ϰ���
                 break;
             end
         end
     end   
 end
 
 if any(any(location_relay))==0%����ڱ�·����һ���м̽ڵ㶼û��ѡ����ô���Ǿͽ���·�ε�send_start����location_relay��
     location_relay(1,1) = 1;
     location_relay(1,2:3) = send_start(1,1:2);
 end
 if isend==0%���send_end����ͨJunction��
     %��������ʼȷ��һ�������һ��relayͨ�ŷ�Χ���Ƿ���ڱ��仹Ҫ������relayend��Ҳ����junction���ĵ�
     [location_vehi_in_Rp]=jyc_vehi_near_junction_in_Rp_choose(location_vehi,locationmark,R,Linmap1m,relay,send_end,ima2);%�ҳ����һ��relayͨ�ŷ�Χ�ڵĳ���
     nearest = relay;%��ʼ����relayend����ĳ����ڵ����nearest�����ֵ�����һ��relay������
     for i = 1:length(location_vehi_in_Rp(:,1))%ͨ��forѭ��ѡ�����relayend�ĵ�
         dis1 = (sum((location_vehi_in_Rp(i,2:3)-relayend).^2))^0.5/Linmap1m;
         dis2 = (sum((nearest-relayend).^2))^0.5/Linmap1m;
         if dis1 < dis2 %������ڱ�nearest������relayend�ĳ����ڵ�
             nearest = location_vehi_in_Rp(i,2:3);%����nearest
         end
     end
     %ȷ����ϣ��������nearest�ĸ��£�˵�����ڱ����һ��relay������relayend�ĵ㣬��֮�򲻴���
     if nearest(1,1)~=relay(1,1) || nearest(1,2)~=relay(1,2)%ֻҪ����nearest�д洢��X��Y�����е�ĳһ����relay��X��Y���겻�Եȣ���ô˵�����¹���nearest����         
         send_end(1,1:2) = nearest;
         send_end(1,3) = 1;
         posi_opt = nearest;             
         NZ = find(sum(abs(location_relay),2)~=0, 1, 'last' );%NZ��Ϊ���㣬�����������location_relay�з����������ں��浱��length(A(;,1))��ʹ��
         if NZ>1%���location_relay������˲�ֹһ��relay����Ϊ�ڱ����м̽ڵ�ѡ��·���ϲ�ֹ���й�һ���м̽ڵ�ѡ��
             dis3 = (sum((nearest-location_relay(NZ-1,2:3)).^2))^0.5/Linmap1m;%����nearest�������һ��relay��һ���ľ���
             sender = location_relay(NZ-1,2:3);
         else%���location_relay����ֻ����һ��relay����Ϊ�ڱ����м̽ڵ�ѡ��·����ֻ���й�һ���м̽ڵ�ѡ��
             dis3 = (sum((nearest-send_start(1,1:2)).^2))^0.5/Linmap1m;%����nearest����send_start�ľ���
             sender = send_start(1,1:2);
         end
         if dis3 < R %��������һ��relay����һ��(����send_start)���Ϳ���ֱ������nearest�㣬��ô��ֱ�Ӱ����һ��relay���³�nearest��             
             relay = nearest;             
             [location_vehi_in_Rp]=jyc_vehi_near_junction_in_Rp_choose(location_vehi,locationmark,R,Linmap1m,sender,send_end,ima2);    %�ó�location_vehi_in_Rp
             if N_part==4%�����log�㷨
                 [delay_one_hop_mini_average,PDR_part] = jyc_log_partition (location_vehi_in_Rp,density_EM,N_iter,N_part,CW_CTB,sender,posi_opt,relay);
                 t = t-last_t+delay_one_hop_mini_average;%���¼������һ���ķѵ�tʱ�䣬������˵�ĺ�ʱ����
             else        %�������N_part==3����3P3B�㷨
                 [delay_one_hop_average,PDR_part] = jyc_3P3B_delay_calcu (location_vehi_in_Rp,density_EM,N_iter,N_part,CW_CTB,sender,posi_opt,relay);
                 t = t-last_t+delay_one_hop_average;%���¼������һ���ķѵ�tʱ�䣬������˵�ĺ�ʱ����
             end
             location_relay(NZ,2:3) = relay;            
         else%���һ���޷�����nearest�㣬��ô��ԭ���һ��relay��Ϊ��sender��ѡ��nearest����Ϊ�µ����һ��relay�������location_relay����Ķ�Ӧλ�ã�����t��ֵ
             sender = relay;%����һ������ôԭrelay�ͳ�����sender
             relay = nearest;
             [location_vehi_in_Rp]=jyc_vehi_near_junction_in_Rp_choose(location_vehi,locationmark,R,Linmap1m,sender,send_end,ima2);    %�ó�location_vehi_in_Rp
             if N_part==4%�����log�㷨
                 [delay_one_hop_mini_average,PDR_part] = jyc_log_partition (location_vehi_in_Rp,density_EM,N_iter,N_part,CW_CTB,sender,posi_opt,relay);
                 t = t+delay_one_hop_mini_average;%����һ������ô������һ����ʱ
             else        %�������N_part==3����3P3B�㷨
                 [delay_one_hop_average,PDR_part] = jyc_3P3B_delay_calcu (location_vehi_in_Rp,density_EM,N_iter,N_part,CW_CTB,sender,posi_opt,relay);
                 t = t+delay_one_hop_average;%����һ������ô������һ����ʱ
             end
             location_relay(NZ+1,1) = NZ+1;
             location_relay(NZ+1,2:3) = relay;
         end
         PDR_set = [PDR_set;PDR_part];
%          plot(relay(1),relay(2),'^','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);%���µ����һ��relay��ͼ�л���
%          hold on        
     end
 end
 if isempty(PDR_set)%˵��û����whileѭ����ֱ�Ӿͽ����ˣ���ô����Ĭ��PDR_setΪ1����ɹ�
     PDR_set = 1;
 end
 
 if mean(PDR_set(:,1))~=1
     PDR = 0;
 else
     PDR = 1;
 end
 
 if re_t>t %�������㲥��ʱ������ͨ�㲥��ʱ
     t = re_t;%��ȡ����㲥��ʱΪ�㲥��ʱ
 end