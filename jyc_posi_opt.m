 function  [posi_opt]=jyc_posi_opt(locationmark,R,Linmap1m,relay_main,send_end)
 %该代码用于选择弯道算法和直道算法的Popt点，这里输入的send_end在正常广播下都是多行两列的，在弯道算法中的反向广播时输入的re_send_end是多行三列的，此时代码通过区分send_end是否有第三列存在来判断是进行反向广播的Popt选取还是正常广播的Popt选取
         relay = relay_main;
         broadcast_sign = length(send_end(1,:));%如果broadcast_sign为2，表示在进行正常广播，如果broadcast_sign为3便是进行的是反向广播
                 %接下来确定relay所在路段，便于之后选出在relay点路段前面的Popt点，前面的意思是更靠近消息传播终点send_end
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
              dis1 = (sum((relay-locationmark(i,1:2)).^2))^0.5/Linmap1m;%线段的起点距relay的距离
              dis2 = (sum((relay-locationmark(i+1,1:2)).^2))^0.5/Linmap1m;%线段的终点距relay的距离
              if (dis1 >= R && dis2 <= R) || (dis1 <= R && dis2 >= R)%如果线段的起点距relay的距离大于R，终点距relay的距离小于R，或者线段的起点距relay的距离小于R，终点距relay的距离大于R，意思是判断线段是否与relay的通信范围R边界相交
                  %如果线段与relay的通信范围R边界相交，则通过以下代码求出交点坐标
                 syms x y;
                 eq1=(x-relay(1))^2+(y-relay(2))^2-(R*Linmap1m)^2;%建立以relay为圆心以R为半径的圆eq1
                 eq1=subs(eq1);
                 eq2=(y-locationmark(i,2))*(locationmark(i+1,1)-locationmark(i,1))-(locationmark(i+1,2)-locationmark(i,2))*(x-locationmark(i,1));%根据i路段的起点和终点建立直线eq2
                 eq2=subs(eq2);
                 [x,y]=solve(eq1,eq2);%求圆与直线的交点
                 x = eval([x]);
                 y = eval([y]);  
                 %由于圆与直线的交点有两个，我们需要的是在路段i上的那个交点，故通过以下两行代码选出
                 X = x((locationmark(i,1)<=x & x<=locationmark(i+1,1))|(locationmark(i,1)>=x & x>=locationmark(i+1,1)));
                 Y = y((locationmark(i,2)<=y & y<=locationmark(i+1,2))|(locationmark(i,2)>=y & y>=locationmark(i+1,2)));  
                 %以下代码是从路段与圆的交点中筛选出对我们有积极作用的交点，大概意思是假如我们求出的这个圆与路段的交点反而在中继节点relay去往消息终点send_end的反方向（与目的地背道而驰）上，则我们将抛弃这个消极作用的交点
                 if broadcast_sign==2 && isempty(X)==0 && isempty(Y)==0 && relay_road_index>=i%relay_road_index是relay节点对应的路段编号，relay_road_index>i的意思是当relay所在的路段编号大于Popt预备点的编号时，该Popt预备点即为我们需要的Popt点，因为我们的路段编号是从send_end开始往send_start从小到大排的，编号越小表示在路段编号上离send_end越近
                    point(j,1) = X;
                    point(j,2) = Y;
                    j = j+1;
                 end   
                 
                 %以下的代码是专门给弯道算法中出现了空白区域vacant后，进入反向广播时寻找Popt点所用的代码
                 if broadcast_sign==3 && isempty(X)==0 && isempty(Y)==0%如果broadcast_sign等于3，意思是在进行反向广播
                     if send_end(1,3)-relay_road_index>0 && relay_road_index<=i%反向广播re_send_end中第三列存储的是最终点所在的路段，如果最终点所在的路段大于当前的relay所在的路段，则选择的Popt点的路段也应大于当前relay的路段
                         point(j,1) = X;
                         point(j,2) = Y;
                         j = j+1; 
                     elseif send_end(1,3)-relay_road_index<0 && relay_road_index>=i%如果最终点所在的路段小于当前的relay所在的路段，则选择的Popt点的路段也应小于当前relay的路段
                         point(j,1) = X;
                         point(j,2) = Y;
                         j = j+1;
                     end
                 end%反向广播Popt点的选取至此结束
                 
              end
            end
         end        
                     
         
         if isempty(point) %即表示交点坐标存入失败
             j = j-1;%将j倒回一步，重新存交点坐标
             x(x==X) = [];%将x中存着的失败交点坐标X删除
             y(y==Y) = [];%将y中存着的失败交点坐标Y删除
             point(j,1) = x;%将x中剩下的交点坐标存入point中
             point(j,2) = y;%将y中剩下的交点坐标存入point中           
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
         point = zeros(length(point(:,1)),2);%用完point后，清除point内的数据
         if broadcast_sign == 2
            plot(posi_opt(1),posi_opt(2),'*','LineWidth',1,'MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',8);%正常广播的Popt标绿
            hold on
         elseif broadcast_sign == 3
            plot(posi_opt(1),posi_opt(2),'*','LineWidth',1,'MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',8);%反向广播的Popt标红
            hold on
         end