function  [t,location_relay,n_relay,n_jucntion,relay_main,relay]=jyc_crossroads_relay_selection(location_vehi,junction,n_jucntion,n_relay,locationbranch,locationmark,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,t,d_relay_junc,range)
 %�����ǽ���·�ڵ��м̽ڵ�ѡ���㷨���㷨��ɺ󷵻ؼ�¼ʱ���t���洢�м̽ڵ�ľ���location_relay����¼��ǰ�м̽ڵ��ǵڼ����м̽ڵ��n_relay����¼��ǰ·���ǵڼ���·�ڵ�n_jucntion����¼��·�м̽ڵ��relay_main����¼�м̽ڵ��relay

         t = t+1e-3;     %%��¼ÿ��ʮ��·�ڳ����м̽ڵ�ѡ���������ӳ�         
         Rp = d_relay_junc;  %��λ ��  
         posi_opt = junction(n_jucntion,2:3);
         
         index_vehi_part = find(((location_vehi(:,2)-posi_opt(1)).^2+(location_vehi(:,3)-posi_opt(2)).^2).^0.5./Linmap1m<=Rp);
         inform_vehi_part = zeros(length(index_vehi_part),4);
         inform_vehi_part = location_vehi(index_vehi_part,1:4);
         
         relay = jyc_partition_exp(posi_opt,Rp,inform_vehi_part,N_iter,N_part,A,Linmap1m);
         
         plot(relay(1),relay(2),'o','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','r','MarkerSize',6);
         hold on
         
         relay_main =  relay;%�����������м̽ڵ���Ϣ���Ա������������м̽ڵ�ѡ��
         location_relay(n_relay,1) = n_relay;
         location_relay(n_relay,2:3) = relay;
         location_relay(n_relay,4) = 3;
         n_relay = n_relay+1;
         
         index_branchofjunc = setdiff(find(locationbranch(:,3)==junction(n_jucntion,1)),length(locationbranch(:,3)));%Ѱ�ҵ�ǰ·�ڵ�֧·
         %%-------��ʼ����֧·���м�ѡ��֧·��������Ϣ����������
         for i=1:length(index_branchofjunc)
             index_branch(i) = index_branchofjunc(i); 
             tip_extend =0; %��ʾ��ǰ֧·�Ƿ����ӳ���1Ϊ�У�0Ϊû��?
             if index_branchofjunc(i)+1<=length(locationbranch)
                 if  locationbranch(index_branchofjunc(i)+1,3)==0  %�ж�֧·�Ƿ����ӳ����������ӳ�·�εĶ˵�Ϊ֧·���նˡ��ڱ������У�֧·���ֻ��1���ӳ�?
                     index_branch(i) = index_branchofjunc(i)+1;%���Դ洢֧·��Ӧ�ڱ���locationbranch�е�λ��
                     tip_extend =1;
                 end   
             end
             if R<(sum((locationbranch(index_branch(i),1:2)-junction(n_jucntion,2:3)).^2))^0.5/Linmap1m  %��֧·Զ�˵���R��Χ֮��
                 Rp = R;
                 pointA=locationbranch(index_branch(i),1:2); %Ѱ������λ��posi_opt
                 if tip_extend==1                     
                     pointB=locationbranch(index_branch(i)-1,1:2);
                 else 
                     pointB=junction(n_jucntion,2:3);
                 end
                 
                 l_road = sum((pointA-pointB).^2)^0.5;
                  if pointB(1)~= pointA(1)&& pointB(2)~= pointA(2)
                     deltaX = 0.01*(pointB(1)-pointA(1))/l_road;%�Ե�·����1��Ϊ���ȷ������ͼ�����ص�λ��?
                     X = pointA(1): deltaX:pointB(1);
                     Y = (pointB(2)-pointA(2))/(pointB(1)-pointA(1))*(X-pointA(1))+pointA(2);
                   else if pointB(1)== pointA(1)
                           deltaX = 0.01/l_road;
                           Y = pointA(2):deltaX:pointB(2);
                           X = pointA(1)*ones(1,length(Y));
                       else
                           deltaX = 0.01/l_road;
                           X = pointA(1):deltaX:pointB(1);
                           Y = pointA(2)*ones(1,length(Y));
                       end
                  end           

                     [m,n] = min(abs(((X-relay_main(1)).^2+(Y-relay_main(2)).^2).^0.5/Linmap1m-R));
                     posi_opt = [X(n),Y(n)];
             else %��֧·Զ�˵���R��Χ֮��
                 Rp = (sum((locationbranch(index_branch(i),1:2)-relay_main).^2))^0.5/Linmap1m;
                 posi_opt = locationbranch(index_branch(i),1:2);
             end
             
          index_vehi_part_1 = find(((location_vehi(:,2)-posi_opt(1)).^2+(location_vehi(:,3)-posi_opt(2)).^2).^0.5./Linmap1m<=Rp);
          index_vehi_part_2 = find(((location_vehi(:,2)-relay_main(1)).^2+(location_vehi(:,3)-relay_main(2)).^2).^0.5./Linmap1m<=Rp);
          index_vehi_part =intersect(index_vehi_part_1,index_vehi_part_2);
          inform_vehi_part = zeros(length(index_vehi_part),4);
          inform_vehi_part = location_vehi(index_vehi_part,1:4);
             
%               index_vehi_part = find(((location_vehi(:,2)-posi_opt(1)).^2+(location_vehi(:,3)-posi_opt(2)).^2).^0.5./Linmap1m<=Rp);
%               inform_vehi_part = zeros(length(index_vehi_part),4);
%               inform_vehi_part = location_vehi(index_vehi_part,1:4);
              
            if isempty(inform_vehi_part)
                a=1;
            end
              relay = partition_exp(posi_opt,Rp,inform_vehi_part,N_iter,N_part,A,Linmap1m);
              
              plot(relay(1),relay(2),'v','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);%�м̽ڵ�x����Ϊrelay(1)��y����Ϊrelay(2)��ָ����ʶ��Ϊ'v'���������Σ�ָ���߿�'LineWidth'Ϊ1��'MarkerEdgeColor'ָ����ʶ���ı�Ե��ɫΪ��ɫ��'MarkerFaceColor'ָ����ʶ�������ɫΪ��ɫ��'MarkerSize'ָ����ʶ����СΪ6
              hold on
              
              location_relay(n_relay,1) = n_relay;
              location_relay(n_relay,2:3) = relay;
              location_relay(n_relay,4) = 1;
              n_relay = n_relay+1;
         end
                %---Ѱ�������ϵ��м̽ڵ�
              juction_one = junction(n_jucntion,2:3);
              index_roadmark_x = find(locationmark(:,1)==juction_one(1));%Ѱ�ҵ�·�ϵ��¸���ǵ���locationmark�е�λ�ã��п������۵㣬Ҳ������·��
              index_roadmark_y = find(locationmark(:,2)==juction_one(2));
              index_roadmark = intersect(index_roadmark_x,index_roadmark_y); 
              
              if n_jucntion+1<=length(junction(:,1))
                  range = min(R,((relay_main(1)-junction(n_jucntion+1,2)).^2+(relay_main(2)-junction(n_jucntion+1,3)).^2).^0.5./Linmap1m+junction(n_jucntion+1,4));%Ϊ��֤��������ѡ����м̽ڵ㳬����һ��·�ڵ�Rp���������·�ڵ��������?             
              end
              
             while (sum((locationmark(index_roadmark,1:2)-relay_main(1:2)).^2))^0.5/Linmap1m<range  %ֱ��·�ζ˵㴦����range��Χ
                 if index_roadmark>1
                     index_roadmark =index_roadmark -1;%��start��end,���Ӻ���ʯ����������mark�ķ����෴�������� -
                 else break
                 end
             end
               %-------Ѱ������λ��posi_opt
             
             pointA=locationmark(index_roadmark,1:2); 
             pointB=locationmark(index_roadmark+1,1:2); 
             l_road = sum((pointA-pointB).^2)^0.5;
             if pointB(1)~= pointA(1)&& pointB(2)~= pointA(2)
                     deltaX = 0.01*(pointB(1)-pointA(1))/l_road;%�Ե�·����1��Ϊ���ȷ������ͼ�����ص�λ��
                     X = pointA(1): deltaX:pointB(1);
                     Y = (pointB(2)-pointA(2))/(pointB(1)-pointA(1))*(X-pointA(1))+pointA(2);
                   else if pointB(1)== pointA(1)
                           deltaX = 0.01/l_road;
                           Y = pointA(2):deltaX:pointB(2);
                           X = pointA(1)*ones(1,length(Y));
                       else
                           deltaX = 0.01/l_road;
                           X = pointA(1):deltaX:pointB(1);
                           Y = pointA(2)*ones(1,length(Y));
                       end
              end  
                  
%              range = min(R,((relay_main(1)-jucntion(n_jucntion+1,2)).^2+(relay_main(2)-jucntion(n_jucntion+1,3)).^2).^0.5./Linmap1m+R/2);%Ϊ��֤��������ѡ����м̽ڵ㳬����һ��·�ڵ�R/2���������·�ڵ��������
             [m,n] = min(abs(((X-relay_main(1)).^2+(Y-relay_main(2)).^2).^0.5/Linmap1m-range));
             posi_opt = [X(n),Y(n)];         
               Rp = sum((posi_opt-relay_main(1:2)).^2).^0.5/Linmap1m;
               
             index_vehi_part_1 = find(((location_vehi(:,2)-posi_opt(1)).^2+(location_vehi(:,3)-posi_opt(2)).^2).^0.5./Linmap1m<=Rp);
             index_vehi_part_2 = find(((location_vehi(:,2)-relay_main(1)).^2+(location_vehi(:,3)-relay_main(2)).^2).^0.5./Linmap1m<=Rp);
             index_vehi_part =intersect(index_vehi_part_1,index_vehi_part_2);
             inform_vehi_part = zeros(length(index_vehi_part),4);
             inform_vehi_part = location_vehi(index_vehi_part,1:4);        
            if isempty(inform_vehi_part)
                a=1;
            end
             relay = partition_exp(posi_opt,Rp,inform_vehi_part,N_iter,N_part,A,Linmap1m);
             
             plot(relay(1),relay(2),'v','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);
             hold on
                
              location_relay(n_relay,1) = n_relay;
              location_relay(n_relay,2:3) = relay;
              location_relay(n_relay,4) = 1;
              n_relay = n_relay+1;    
              n_jucntion = n_jucntion+1;%�ڵ�ǰ·��ѡ����n_jucntion��1�������´�Ѱ��
              relay_main = relay;
