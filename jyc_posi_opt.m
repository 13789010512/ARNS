 function  [posi_opt]=jyc_posi_opt(locationmark,R,Linmap1m,relay_main,send_end)
 %�ô�������ѡ������㷨��ֱ���㷨��Popt�㣬���������send_end�������㲥�¶��Ƕ������еģ�������㷨�еķ���㲥ʱ�����re_send_end�Ƕ������еģ���ʱ����ͨ������send_end�Ƿ��е����д������ж��ǽ��з���㲥��Poptѡȡ���������㲥��Poptѡȡ
         relay = relay_main;
         broadcast_sign = length(send_end(1,:));%���broadcast_signΪ2����ʾ�ڽ��������㲥�����broadcast_signΪ3���ǽ��е��Ƿ���㲥
                 %������ȷ��relay����·�Σ�����֮��ѡ����relay��·��ǰ���Popt�㣬ǰ�����˼�Ǹ�������Ϣ�����յ�send_end
                 relay_road_indexby_x = zeros(length(locationmark(:,1)),1);
                 relay_road_indexby_y = zeros(length(locationmark(:,1)),1);
                 for m = 1:length(locationmark(:,1))-1
                     if (locationmark(m,1)<=relay(1) && relay(1)<=locationmark(m+1,1)) || (locationmark(m,1)>=relay(1) && relay(1)>=locationmark(m+1,1))
                         relay_road_indexby_x(m) = m;
                     end
                 end
                 for m = 1:length(locationmark(:,1))-1
                     if (locationmark(m,2)<=relay(2) && relay(2)<=locationmark(m+1,2)) || (locationmark(m,2)>=relay(2) && relay(2)>=locationmark(m+1,2))
                         relay_road_indexby_y(m) = m;
                     end
                 end                
                 relay_road_index = intersect(relay_road_indexby_x,relay_road_indexby_y);
                 relay_road_index(relay_road_index==0)=[];
                 if isempty(relay_road_index) 
                     relay_road_index = length(locationmark(:,1));
                 end
                 
         j = 1;
         for i = 1:length(locationmark(:,1))-1           
            if i ~= length(locationmark(:,1))
              dis1 = (sum((relay-locationmark(i,1:2)).^2))^0.5/Linmap1m;%�߶ε�����relay�ľ���
              dis2 = (sum((relay-locationmark(i+1,1:2)).^2))^0.5/Linmap1m;%�߶ε��յ��relay�ľ���
              if (dis1 >= R && dis2 <= R) || (dis1 <= R && dis2 >= R)%����߶ε�����relay�ľ������R���յ��relay�ľ���С��R�������߶ε�����relay�ľ���С��R���յ��relay�ľ������R����˼���ж��߶��Ƿ���relay��ͨ�ŷ�ΧR�߽��ཻ
                  %����߶���relay��ͨ�ŷ�ΧR�߽��ཻ����ͨ�����´��������������
                 syms x y;
                 eq1=(x-relay(1))^2+(y-relay(2))^2-(R*Linmap1m)^2;%������relayΪԲ����RΪ�뾶��Բeq1
                 eq1=subs(eq1);
                 eq2=(y-locationmark(i,2))*(locationmark(i+1,1)-locationmark(i,1))-(locationmark(i+1,2)-locationmark(i,2))*(x-locationmark(i,1));%����i·�ε������յ㽨��ֱ��eq2
                 eq2=subs(eq2);
                 [x,y]=solve(eq1,eq2);%��Բ��ֱ�ߵĽ���
                 x = eval([x]);
                 y = eval([y]);  
                 %����Բ��ֱ�ߵĽ�����������������Ҫ������·��i�ϵ��Ǹ����㣬��ͨ���������д���ѡ��
                 X = x((locationmark(i,1)<=x & x<=locationmark(i+1,1))|(locationmark(i,1)>=x & x>=locationmark(i+1,1)));
                 Y = y((locationmark(i,2)<=y & y<=locationmark(i+1,2))|(locationmark(i,2)>=y & y>=locationmark(i+1,2)));  
                 %���´����Ǵ�·����Բ�Ľ�����ɸѡ���������л������õĽ��㣬�����˼�Ǽ���������������Բ��·�εĽ��㷴�����м̽ڵ�relayȥ����Ϣ�յ�send_end�ķ�������Ŀ�ĵر������ۣ��ϣ������ǽ���������������õĽ���
                 if broadcast_sign==2 && isempty(X)==0 && isempty(Y)==0 && relay_road_index>=i%relay_road_index��relay�ڵ��Ӧ��·�α�ţ�relay_road_index>i����˼�ǵ�relay���ڵ�·�α�Ŵ���PoptԤ����ı��ʱ����PoptԤ���㼴Ϊ������Ҫ��Popt�㣬��Ϊ���ǵ�·�α���Ǵ�send_end��ʼ��send_start��С�����ŵģ����ԽС��ʾ��·�α������send_endԽ��
                    point(j,1) = X;
                    point(j,2) = Y;
                    j = j+1;
                 end   
                 
                 %���µĴ�����ר�Ÿ�����㷨�г����˿հ�����vacant�󣬽��뷴��㲥ʱѰ��Popt�����õĴ���
                 if broadcast_sign==3 && isempty(X)==0 && isempty(Y)==0%���broadcast_sign����3����˼���ڽ��з���㲥
                     if send_end(1,3)-relay_road_index>0 && relay_road_index<=i%����㲥re_send_end�е����д洢�������յ����ڵ�·�Σ�������յ����ڵ�·�δ��ڵ�ǰ��relay���ڵ�·�Σ���ѡ���Popt���·��ҲӦ���ڵ�ǰrelay��·��
                         point(j,1) = X;
                         point(j,2) = Y;
                         j = j+1; 
                     elseif send_end(1,3)-relay_road_index<0 && relay_road_index>=i%������յ����ڵ�·��С�ڵ�ǰ��relay���ڵ�·�Σ���ѡ���Popt���·��ҲӦС�ڵ�ǰrelay��·��
                         point(j,1) = X;
                         point(j,2) = Y;
                         j = j+1;
                     end
                 end%����㲥Popt���ѡȡ���˽���
                 
              end
            end
         end        
                     
         
         if isempty(point) %����ʾ�����������ʧ��
             j = j-1;%��j����һ�������´潻������
             x(x==X) = [];%��x�д��ŵ�ʧ�ܽ�������Xɾ��
             y(y==Y) = [];%��y�д��ŵ�ʧ�ܽ�������Yɾ��
             point(j,1) = x;%��x��ʣ�µĽ����������point��
             point(j,2) = y;%��y��ʣ�µĽ����������point��           
         end
         for i = 1:length(point(:,1))
             P = point(i,1:2);
             P(:,3) = 0;
             relay(:,3) = 0;
             send_end(:,3) = 0;
             d(i) = norm(cross(relay-send_end,P-send_end))/norm(relay-send_end);
         end
         a_row = find(d == min(d));
         posi_opt = point(a_row,1:2);           
         point = zeros(length(point(:,1)),2);%����point�����point�ڵ�����
         if broadcast_sign == 2
            plot(posi_opt(1),posi_opt(2),'*','LineWidth',1,'MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',8);%�����㲥��Popt����
            hold on
         elseif broadcast_sign == 3
            plot(posi_opt(1),posi_opt(2),'*','LineWidth',1,'MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',8);%����㲥��Popt���
            hold on
         end