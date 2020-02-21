function  [t,location_relay]=jyc_complete_relay_selection(location_vehi,locationjucntion,locationbranch,locationmark,R,num_vehi,N_iter,N_part,A,T,Linmap1m)
%%%���ɵ�·������Ϣ������ʾ����·������ʮ��·��
 start = locationmark(length(locationmark(:,1)),1:2);%ֱ��ȡ·����������Ŀ�ĵ㣬����Ϣ���Ͷ˵㣬������ʯ����λ��
 %%%-----����start����Ľڵ�����Ϣ�ĳ�ʼ���ͽڵ�
 
 dis_vehi_start = ((location_vehi(:,2)-start(1)).^2+(location_vehi(:,3)-start(2)).^2).^0.5;%����start����Ľڵ�����Ϣ�ĳ�ʼ���ͽڵ�?
 index_start = find(dis_vehi_start==min(dis_vehi_start));
 if isempty(index_start)
     a=1;
 end
 send_start = location_vehi(index_start,2:3);%��Ϣ������㳵����ֻ����λ����Ϣ��x,y)
 send_end = location_vehi(1,2:3);%��Ϣ�����յ㳵��
 plot(send_start(1),send_start(2),'o','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);
 hold on
%%%-----����Ϣ���ݵ��յ�λ��
 relayend = locationjucntion(1,2:3);%ֻ����λ����Ϣ��x,y)
 %�����龰��Linmap1m = sum((locationmark(3,1:2)-locationmark(4,1:2)).^2).^0.5/395;%����1����ͼ�ж�Ӧ�����ؼ����
 %����·�龰��Linmap1m=sum((locationmark(6,1:2)-locationmark(7,1:2)).^2).^0.5/3700;%����·�龰������1����ͼ�ж�Ӧ�����ؼ����

 
 L_road = sum((locationmark(2:length(locationmark(:,1)),1:2)-locationmark(1:length(locationmark(:,1))-1,1:2)).^2,2).^0.5/Linmap1m;%������������·�ϵ���markȷ���ĵ�i��·�ε���ʵ����
 L_road_all = sum(L_road);%·�ε���ʵ�ܳ���
 
 location_relay = zeros(100,4);%���������м̽ڵ㣬1����ţ�2��3����x,y);4:����1ֱ����2�����3ʮ��·��
 relay = send_start;%ֻ����λ����Ϣ��x,y)
 junction = zeros(length(locationjucntion(:,1)),4);%��������·����Ϣ��4��ѡ��·���м̽ڵ�ʱ��Ӧ��Rp
 junction(:,1:3) = locationjucntion;
 junction(1,4) = min(R/2,(sum((junction(1,2:3)-junction(2,2:3)).^2))^0.5/Linmap1m);
 junction(length(junction(:,1)),4) = min(R/2,(sum((junction(length(junction(:,1)),2:3)-junction(length(junction(:,1))-1,2:3)).^2))^0.5/Linmap1m);
 for i=2:length(junction(:,1))-1
     junction(i,4) = min(min(R/2,(sum((junction(i,2:3)-junction(i-1,2:3)).^2))^0.5/Linmap1m),(sum((junction(i,2:3)-junction(i+1,2:3)).^2))^0.5/Linmap1m);%4��ÿ��ʮ��·�ڵ�Rp
 end
 junction = junction(length(junction(:,1)):-1:1,:); %����start�ľ����ɽ���Զ����
 n_jucntion = 1;
 n_relay = 1;
 relay_main = relay;
 t = 0;
 count = 1;%ѭ��������ʼΪ1
 d_relay_junc = 0;%��ʼ���м̽ڵ��·�ڵľ���
 re_t = 0;%��ʼ������㲥��ʱ��re_tΪ0
 n_junc_record = zeros(length(junction(:,1)),1);%��ʼ��n_junc_record����������¼�Ѿ���·���м̽ڵ�ѡ���㷨ѡ�����·��
 rec = 1;%��ʼ��n_junc_record�����¼����Ϊ1
 range = min(R,((relay_main(1)-junction(n_jucntion,2)).^2+(relay_main(2)-junction(n_jucntion,3)).^2).^0.5./Linmap1m+junction(n_jucntion,4));%Ϊ��֤��������ѡ����м̽ڵ㳬����һ��·�ڵ�R/2���������·�ڵ��������
 
 while ((sum((relay-relayend).^2))^0.5/Linmap1m>R || n_jucntion<=length(junction(:,1))) && t<=T
     
     if d_relay_junc ~= Inf
        d_set_relay_junc = zeros(length(junction(:,1)),2);%d_set_relay_junc��һ�д��м̽ڵ�relay�����·�ڱ��֮��ľ��룬�ڶ��д��м̽ڵ�relay�Ƿ��������ĳ·�������ı�ǣ�����������������Ӧ·�ھ���λ���еڶ��б���1
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
            d_relay_junc = (sum((relay-junction(n_jucntion,2:3)).^2))^0.5/Linmap1m;  %�м̽ڵ�������·�ڵľ���
        end
     else
        n_jucntion = 1;       
     end
     if d_relay_junc<=junction(n_jucntion,4) %�ж��Ƿ����ʮ��·��
          %%%%--------Ѱ��·���м̽ڵ�          
        n_junc_record(rec,1) = n_jucntion;%��������·���м̽ڵ�ѡ���·����Ŵ���n_junc_record�����м�¼����
        rec = rec+1;
        [t,location_relay,n_relay,n_jucntion,relay_main,relay]...
            =jyc_crossroads_relay_selection(location_vehi,junction,n_jucntion,n_relay,locationbranch,locationmark,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,t,d_relay_junc,range);  
        
     else  %���û����·���м̽ڵ�ѡ����ô����Popt�㣬׼���������������������ֱ���м̽ڵ�ѡ���㷨
         n_jucntion = n_jucntion+1;   %ҲҪʹ��·�ڼ�����һ�������ǽ�ʱ����whileѭ��
         [posi_opt]=jyc_posi_opt(locationmark,R,Linmap1m,relay_main,send_end);     %�ҳ�Popt��   
         
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
         %�����������Rp��Χ�ڵ�·�γ��ȣ���������road_in_Rp��
         %������������Rp�е�·�γ����ж��ǽ������ѡ����ֱ��ѡ��
         if R*1.1 < road_in_Rp  %%%%----------�������    
             [t,location_relay,n_relay,relay_main,relay,re_t]...
                 =jyc_curve_relay_selection(location_vehi,n_relay,locationmark,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,relay,send_end,send_start,t,T,re_t,posi_opt);         
         else    %%%%---------ֱ������        
             [t,location_relay,n_relay,n_jucntion,relay_main,relay,range]...
                 =jyc_straight_relay_selection(location_vehi,junction,n_jucntion,n_relay,locationmark,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,relay_main,t,posi_opt);                 
         end        
         
     end
  
     
     if d_relay_junc == Inf
         n_jucntion = n_jucntion+length(junction(:,1)); 
     end   
     
 end
 
 if re_t>t %�������㲥��ʱ������ͨ�㲥��ʱ
     t = re_t;%��ȡ����㲥��ʱΪ�㲥��ʱ
 end