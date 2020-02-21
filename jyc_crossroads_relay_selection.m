function  [t,location_relay,n_relay,n_jucntion,relay_main,relay]=jyc_crossroads_relay_selection(location_vehi,junction,n_jucntion,n_relay,locationbranch,locationmark,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,t,d_relay_junc,range)
 %做的是交叉路口的中继节点选择算法，算法完成后返回记录时间的t，存储中继节点的矩阵location_relay，记录当前中继节点是第几个中继节点的n_relay，记录当前路口是第几个路口的n_jucntion，记录主路中继节点的relay_main，记录中继节点的relay

         t = t+1e-3;     %%记录每次十字路口场景中继节点选择两跳的延迟         
         Rp = d_relay_junc;  %单位 米  
         posi_opt = junction(n_jucntion,2:3);
         
         index_vehi_part = find(((location_vehi(:,2)-posi_opt(1)).^2+(location_vehi(:,3)-posi_opt(2)).^2).^0.5./Linmap1m<=Rp);
         inform_vehi_part = zeros(length(index_vehi_part),4);
         inform_vehi_part = location_vehi(index_vehi_part,1:4);
         
         relay = jyc_partition_exp(posi_opt,Rp,inform_vehi_part,N_iter,N_part,A,Linmap1m);
         
         plot(relay(1),relay(2),'o','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','r','MarkerSize',6);
         hold on
         
         relay_main =  relay;%保存主道上中继节点信息，以便于做主道上中继节点选择
         location_relay(n_relay,1) = n_relay;
         location_relay(n_relay,2:3) = relay;
         location_relay(n_relay,4) = 3;
         n_relay = n_relay+1;
         
         index_branchofjunc = setdiff(find(locationbranch(:,3)==junction(n_jucntion,1)),length(locationbranch(:,3)));%寻找当前路口的支路
         %%-------开始进行支路上中继选择，支路不包括消息传输主道上
         for i=1:length(index_branchofjunc)
             index_branch(i) = index_branchofjunc(i); 
             tip_extend =0; %标示当前支路是否有延长，1为有，0为没有?
             if index_branchofjunc(i)+1<=length(locationbranch)
                 if  locationbranch(index_branchofjunc(i)+1,3)==0  %判断支路是否有延长，有则以延长路段的端点为支路的终端。在本程序中，支路最多只有1段延长?
                     index_branch(i) = index_branchofjunc(i)+1;%用以存储支路对应在变量locationbranch中的位置
                     tip_extend =1;
                 end   
             end
             if R<(sum((locationbranch(index_branch(i),1:2)-junction(n_jucntion,2:3)).^2))^0.5/Linmap1m  %当支路远端点在R范围之外
                 Rp = R;
                 pointA=locationbranch(index_branch(i),1:2); %寻找最优位置posi_opt
                 if tip_extend==1                     
                     pointB=locationbranch(index_branch(i)-1,1:2);
                 else 
                     pointB=junction(n_jucntion,2:3);
                 end
                 
                 l_road = sum((pointA-pointB).^2)^0.5;
                  if pointB(1)~= pointA(1)&& pointB(2)~= pointA(2)
                     deltaX = 0.01*(pointB(1)-pointA(1))/l_road;%以道路长度1米为间隔确定车辆图上像素点位置?
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
             else %当支路远端点在R范围之内
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
              
              plot(relay(1),relay(2),'v','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);%中继节点x坐标为relay(1)，y坐标为relay(2)，指定标识符为'v'即下三角形，指定线宽'LineWidth'为1，'MarkerEdgeColor'指定标识符的边缘颜色为黑色，'MarkerFaceColor'指定标识符填充颜色为绿色，'MarkerSize'指定标识符大小为6
              hold on
              
              location_relay(n_relay,1) = n_relay;
              location_relay(n_relay,2:3) = relay;
              location_relay(n_relay,4) = 1;
              n_relay = n_relay+1;
         end
                %---寻找主道上的中继节点
              juction_one = junction(n_jucntion,2:3);
              index_roadmark_x = find(locationmark(:,1)==juction_one(1));%寻找道路上的下个标记点在locationmark中的位置，有可能是折点，也可能是路口
              index_roadmark_y = find(locationmark(:,2)==juction_one(2));
              index_roadmark = intersect(index_roadmark_x,index_roadmark_y); 
              
              if n_jucntion+1<=length(junction(:,1))
                  range = min(R,((relay_main(1)-junction(n_jucntion+1,2)).^2+(relay_main(2)-junction(n_jucntion+1,3)).^2).^0.5./Linmap1m+junction(n_jucntion+1,4));%为保证在主道上选择的中继节点超过下一个路口的Rp而造成跳过路口的情况出现?             
              end
              
             while (sum((locationmark(index_roadmark,1:2)-relay_main(1:2)).^2))^0.5/Linmap1m<range  %直到路段端点处超出range范围
                 if index_roadmark>1
                     index_roadmark =index_roadmark -1;%从start到end,即从猴子石到东塘，和mark的方向相反，所以是 -
                 else break
                 end
             end
               %-------寻找最优位置posi_opt
             
             pointA=locationmark(index_roadmark,1:2); 
             pointB=locationmark(index_roadmark+1,1:2); 
             l_road = sum((pointA-pointB).^2)^0.5;
             if pointB(1)~= pointA(1)&& pointB(2)~= pointA(2)
                     deltaX = 0.01*(pointB(1)-pointA(1))/l_road;%以道路长度1米为间隔确定车辆图上像素点位置
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
                  
%              range = min(R,((relay_main(1)-jucntion(n_jucntion+1,2)).^2+(relay_main(2)-jucntion(n_jucntion+1,3)).^2).^0.5./Linmap1m+R/2);%为保证在主道上选择的中继节点超过下一个路口的R/2而造成跳过路口的情况出现
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
              n_jucntion = n_jucntion+1;%在当前路口选择中n_jucntion加1，备于下次寻找
              relay_main = relay;
