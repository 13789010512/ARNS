function [t,location_relay,relay,location_vehi,num_vehi,PDR]=jyc_beacon_relay_selection(location_vehi,locationmark,R,num_vehi,N_iter,N_part,A,Linmap1m,T,ima2,t_relay)
%%%���ɵ�·������Ϣ������ʾ����·������ʮ��·��
 global locationjunction density_EM CW_CTB retrans_max;
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
 posi_opt = [];
 old_posi_opt = [];
 old_sender = [];
 sender = [];%��ʼ������
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
 location_relay = zeros(100,4);%���������м̽ڵ㣬1����ţ� 2��3��(x,y)�� 4������(0����������������ϣ���1ֱ����2�����3ʮ��·��)
 n_relay = 1;%�����㲥����ű��
 relay = send_start(1,1:2);%��ʼ��relay����send_start������
 t = t_relay;
 PDR_set = [];%��ʼ��PDRΪ��
 last_t = 0;%��ʼ�����һ���ĺ�ʱ����������Ҫ�����һ��relay������������������һ�������Ƕ�ԭ�����и��ģ�ʱ�������ڱ����������������relay�ĺ�ʱ�������ø���ǰ���ܺ�ʱt��ȥlast_t����ټ��ϸ�������º�ʱ�����ø������׼ȷ�ܺ�ʱ
 
 
 while ((isend == 1 && (bar_bef_end==1 || (sum((relay-relayend).^2))^0.5/Linmap1m>R)) || (isend == 0 && (bar_bef_end==1 || (sum((relay-relayend).^2))^0.5/Linmap1m>R/2))) && t<T 
     %������������Ҫ���㣺ѡ����ͨjunction�㣨relayend�㣩R/2��Χ�ڵĳ���Ϊ�м̽ڵ㣨���ս�Junction��R��Χ�ڵĳ���Ϊ�м̽ڵ㣩��relay��relayend֮�䲻�ܴ����ϰ�����ܽ�����ѭ��     
     delay_one_hop = 0;%��ÿ����ʼ����ʼ��һ����Ϣ�����ʱ
     fail_time = 0;%��ʼ��ʧ�ܴ���
     if isempty(posi_opt)==0%���posi_opt��Ϊ��
         old_posi_opt = posi_opt;%���ϵ�posi_opt��ֵ����old_posi_opt�д���
     end
     [posi_opt,location_vehi_in_Rp]=jyc_bar_posi_opt(location_vehi,locationmark,R,Linmap1m,relay,send_end,ima2); %�ҳ�Popt���location_vehi_in_Rp��ͨ�ŷ�Χ�ڵĳ����ڵ����飩
     route_table = location_vehi_in_Rp;%route_tableΪ·�ɱ����飬��ǰ4����location_vehi_in_Rpһ�£���5�д洢���ű��ϴθ��¹��˶೤ʱ�䣬��6�д��泵���ڵ���relay�ľ���
     %���¶�route_table�����ڵĽڵ������һЩɸѡ
     if isempty(route_table)==0%���route_table��Ϊ��
         [relay_road_index]=jyc_find_relayroad(locationmark,relay);%���relay����·�α��
         for i = 1:length(route_table(:,1))%�ڴ�forѭ���У�����������ᴩ�ػ�·�ε��³��ֿհ���������ȥ��
             if abs(route_table(i,4)-relay_road_index)>25%�ᴩʱ��������relay��·�α�ſ��������25��·�����ϣ���������ͨ�ŷ�Χ�ڳ����ڵ�����·�μ�ȥrelay����·�Σ��ó��ľ���ֵ�������25��˵���ó���λ�úᴩ���ػ�·�Σ�Ӧ��ȥ
                 route_table(i,:)=0;
             end
         end
         route_table(route_table(:,1)==0,:)=[];
     end
     %ɸѡ��ϣ��޳��˺ᴩ����ػ�·�εĳ����ڵ�
     
     if isempty(route_table)%�������ɸѡ���ͨ�ŷ�Χ��û�г����ڵ㣬��ô����ֱ��ѡposi_opt��Ϊ��relay
         if isempty(sender)==0%���sender��Ϊ��
             old_sender = sender;%����sender��ֵ��old_sender
         end
         sender = relay;%����relay��ֵ��sender������ʾ��relay�Ѿ���Ϊ��sender
         [location_vehi,num_vehi,relay]=add_vehi(locationmark,relay,Linmap1m,ima2,posi_opt,send_end,R,num_vehi,location_vehi,old_sender,old_posi_opt);%������relay�������relay
         [relay_road_index]=jyc_find_relayroad(locationmark,relay);%�����relay������·�α��
         route_table(1,1) = num_vehi;%��1�д洢�ó����ڵ���location_vehi�еı��
         route_table(1,2:3) = relay;%��2��3�д洢posi_opt������
         route_table(1,4) = relay_road_index;%��4����posi_opt����·�α��
         route_table(1,5) = 0;%��5���Ǿ��ű��ϴθ��¹��˶೤ʱ�䣬Ŀǰ��ʱ����
         route_table(1,6) = (sum((relay-sender).^2))^0.5/Linmap1m;%route_table��6�д洢relay������sender�ľ��룬һ����˵��R��Ȼ����һ���ƫ��
         %�µ�relay�����location_vehi_in_Rp������
         %����Ҫע�⣬location_vehi_in_Rp���鲻һ����route_table����һ�����ǿ����飬route_table�ǿ������п����������澭����һ��ɸѡ���±��
         vehi_num_in_Rp = length(location_vehi_in_Rp(:,1))+1;
         location_vehi_in_Rp(vehi_num_in_Rp,1) = num_vehi;%��1�д洢�ó����ڵ���location_vehi�еı��
         location_vehi_in_Rp(vehi_num_in_Rp,2:3) = relay;%��2��3�д洢posi_opt������
         location_vehi_in_Rp(vehi_num_in_Rp,4) = relay_road_index;%��4����posi_opt����·�α��

     else
         route_table(:,5) = 0;%��5���Ǹó����ڵ�����ű��ϴθ��¹��˶೤ʱ�䣬Ŀǰ��ʱ����
         for i = 1:length(route_table(:,1))
             route_table(i,6) = (sum((route_table(i,2:3)-relay(1:2)).^2))^0.5/Linmap1m;%route_table��6��������ȡ������relay�ľ���
         end
         [index,~] = find(route_table(:,6)==max(route_table(:,6)));%�ҳ�ͨ�ŷ�Χ�ھ�relay��Զ�ĳ��������±�
         sender = relay;%������ȷ������һ��relayʱ���Ƚ����е�relay��ֵ��sender
         relay = route_table(index,2:3);%�ó�����������Ҫ�ҵ�relay������relay         
     end
     
     location_relay(n_relay,1) = n_relay;
     location_relay(n_relay,2:3) = relay;
     n_relay = n_relay+1;
%      plot(relay(1),relay(2),'^','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);%���µ����һ��relay��ͼ�л���
%      hold on
     relay_in_delay_calculation = relay;%����һ��ר������������ʱ��relay����
     
     %�������������ҳ���relay����Ϣ������sender��route_table�Ȳ�����������ʱ���㺯���м�����ʱ��PDR�Ȳ�������������ھ�����һ���ش�����ʱ�������й��ڳ����ڵ��λ����Ϣȫ������0�����sender�ľ��룬���������ش����򲻸ı䳵���ڵ��λ����Ϣ
     while(1)
         [delay,PDR_part,relay_in_delay_calculation,route_table,PDR_channel] ...
             = jyc_beacon_delay_calculation (route_table,density_EM,sender,relay_in_delay_calculation,R,location_vehi,ima2,send_end,locationmark,posi_opt,fail_time);
%          PDR_set = [PDR_set;PDR_part];
         delay_one_hop = delay_one_hop+delay;    %�洢һ����Ϣ��ʱ
         t = t+delay;
         if PDR_part == 1 && PDR_channel==1      %�����Ϣ����ɹ�
             PDR_set = [PDR_set;1];              %���ɹ���־1����PDR_set������
             break;                              %��������ǰwhileѭ��
         elseif fail_time >= retrans_max || PDR_channel==0 %���PDR_part������1�����ش���������,����ֱ���ŵ����ô�������ʧ�ܣ���ʾ�˴���Ϣ����ʧ�ܣ���ʱΪNaN����Ӧ�ñ�����t��(9.16)
             PDR_set = [PDR_set;0];              %��ʧ�ܱ�־0����PDR_set������
             t = t-delay_one_hop;                %���Ѽ���t�е�һ����Ϣ�ӳٸ���ȥ(9.16)
             delay_one_hop = NaN;                %��ԭһ����Ϣ�ӳٸ�ΪNaN(9.16)
             break;                              %������whileѭ��
         elseif isnan(delay)                     %������ŵ�����ʧ�ܣ������ش������
             fail_time = fail_time +1;           %ʧ�ܴ���+1
         else                                    %��������ű���¼�϶��ѡrelay������ʻ��senderͨ�ŷ�Χ�ڣ������ش������
             fail_time = fail_time +1;           %ʧ�ܴ���+1
         end
     end
     last_t = delay_one_hop;%��һ����Ϣ��ʱ��ֵ��last_t�������е����һ��ʱ��delay_one_hop��Ϊ���һ����ʱ
 
 end    
 
 %���������һ���Ĵ���������Է������һ�����ӿ���junction��
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
         send_end(1,1:2) = nearest;%����relayend����ĳ����ڵ㶨Ϊsend_end
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
         %�������ж�nearest����sender��֮���Ƿ�����ϰ����谭����������ϰ����谭����ʹdis3<RҲ���������һ��ֱ�Ӹ��³�nearest�㣬����ͨ�����һ��relay�������Ϣ�м�ת����nearest��
         nearest_bar_sender = 0;%��ʼ���Ƿ�����ϰ����谭���λΪ0
         sample_num = 100;%��ʼ����������Ϊ100
         det_X = (nearest(1)-sender(1))/sample_num;
         det_Y = (nearest(2)-sender(2))/sample_num;       
         for j = 1:sample_num
             temp_x = sender(1)+j*det_X;
             temp_y = sender(2)+j*det_Y;
             x_data = ceil(temp_x);
             y_data = ceil(temp_y);
             if ima2(y_data,x_data) == 0    %��������Ӧͼ����������ֵΪ0��˵���˴�Ϊ�ϰ��˵���������ϰ����谭��
                 nearest_bar_sender = 1;
                 break;
             end
         end
         %nearest����sender��֮���Ƿ�����ϰ����谭���ж����         
         if dis3 < R && nearest_bar_sender == 0%��������һ��relay����һ��(����send_start)���Ϳ���ֱ������nearest�������ϰ����谭����ô��ֱ�Ӱ����һ��relay���³�nearest��             
             relay = nearest;  
             location_relay(NZ,2:3) = relay;            
             [location_vehi_in_Rp]=jyc_vehi_near_junction_in_Rp_choose(location_vehi,locationmark,R,Linmap1m,sender,send_end,ima2);%�ó�location_vehi_in_Rp����Ϊsender��nearest��send_end��֮��ĳ����ڵ����飨����nearest���ڣ�
             route_table = [];%����route_table
             route_table(:,1:4) = location_vehi_in_Rp(:,1:4);
             route_table(:,5) = 0;
             for i = 1:length(route_table(:,1))
                 route_table(i,6) = (sum((route_table(i,2:3)-sender(1:2)).^2))^0.5/Linmap1m;%route_table��6��������ȡ������relay�ľ���
             end
             fail_time = 0;%��ʼ��ʧ�ܴ���Ϊ0
             relay_in_delay_calculation = relay;%����һ��ר������������ʱ��relay����
             t = t-last_t;%�ܺ�ʱ������һ�����ȼ�ȥ���һ����ʱ
             while(1)
                 [delay,PDR_part,relay_in_delay_calculation,route_table,PDR_channel] ...
                     = jyc_beacon_delay_calculation (route_table,density_EM,sender,relay_in_delay_calculation,R,location_vehi,ima2,send_end,locationmark,posi_opt,fail_time);
                 t = t+delay;                            %�ܺ�ʱ�����ڶ��������¼������һ����ʱ����ɺ�ʱ����
             %    PDR_set = [PDR_set;PDR_part];
                 if PDR_part == 1 && PDR_channel==1      %�����Ϣ����ɹ�
                     PDR_set = [PDR_set;1];              %���ɹ���־1����PDR_set������
                     break;                              %��������ǰwhileѭ��
                 elseif fail_time >= retrans_max || PDR_channel==0  %���PDR_part������1�����ش�������������ֱ���ŵ�����ʧ��
                     PDR_set = [PDR_set;0];              %��ʧ�ܱ�־0����PDR_set������
                     break;                              %������whileѭ��
                 elseif isnan(delay)                     %������ŵ�����ʧ�ܣ������ش������
                     fail_time = fail_time +1;           %ʧ�ܴ���+1
                 else                                    %��������ű���¼�϶��ѡrelay������ʻ��senderͨ�ŷ�Χ�ڣ������ش������
                     fail_time = fail_time +1;           %ʧ�ܴ���+1
                 end
             end
             
         else%���һ���޷�����nearest�㣨��Ϊ����ԭ������ϰ���ԭ�򣩣���ô��ԭ���һ��relay��Ϊ��sender��ѡ��nearest����Ϊ�µ����һ��relay�������location_relay����Ķ�Ӧλ�ã�����t��ֵ
             sender = relay;%����һ������ôԭrelay�ͳ�����sender
             relay = nearest;
             location_relay(NZ+1,1) = NZ+1;
             location_relay(NZ+1,2:3) = relay;                        
             [location_vehi_in_Rp]=jyc_vehi_near_junction_in_Rp_choose(location_vehi,locationmark,R,Linmap1m,sender,send_end,ima2);    %�ó�location_vehi_in_Rp
             route_table = [];%����route_table
             route_table(:,1:4) = location_vehi_in_Rp(:,1:4);
             route_table(:,5) = 0;
             for i = 1:length(route_table(:,1))
                 route_table(i,6) = (sum((route_table(i,2:3)-sender(1:2)).^2))^0.5/Linmap1m;%route_table��6��������ȡ������relay�ľ���
             end
             fail_time = 0;
             relay_in_delay_calculation = relay;%����һ��ר������������ʱ��relay����
             while(1)
                 [delay,PDR_part,relay_in_delay_calculation,route_table,PDR_channel] ...
                     = jyc_beacon_delay_calculation (route_table,density_EM,sender,relay_in_delay_calculation,R,location_vehi,ima2,send_end,locationmark,posi_opt,fail_time);
           %      PDR_set = [PDR_set;PDR_part];
                 t = t+delay;                            %����һ������ô������һ����ʱ
                 if PDR_part == 1 && PDR_channel==1      %�����Ϣ����ɹ�
                     PDR_set = [PDR_set;1];              %���ɹ���־1����PDR_set������
                     break;                              %��������ǰwhileѭ��
                 elseif fail_time >= retrans_max || PDR_channel==0  %���PDR_part������1�����ش�������������ֱ���ŵ����ô�������ʧ��
                     PDR_set = [PDR_set;0];              %��ʧ�ܱ�־0����PDR_set������
                     break;                              %������whileѭ��
                 elseif isnan(delay)                     %������ŵ�����ʧ�ܣ������ش������
                     fail_time = fail_time +1;           %ʧ�ܴ���+1
                 else                                    %��������ű���¼�϶��ѡrelay������ʻ��senderͨ�ŷ�Χ�ڣ������ش������
                     fail_time = fail_time +1;           %ʧ�ܴ���+1
                 end              
             end
         end
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
 
