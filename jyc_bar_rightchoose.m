function[posi_opt] = jyc_bar_rightchoose(road_in_relay_R,locationmark,relay,K,K_min,K_max,R,Linmap1m,send_end)
    %由于我们默认正常广播消息方向都是从路段编号大的地方广播至路段编号小的地方
    %则我们正常广播时只需要选择 road_in_relay_R 数组中存储的路段编号最小的路段作为我们的Popt所在路段即可
    if length(send_end) ==3%如果是在反向广播，则选择最靠近send_end点的路段点作为反向广播Popt所在路段点
        for i = 1:length(road_in_relay_R)%开始对road_in_relay_R中的路段编号进行判断
           dec_num(i) = abs(road_in_relay_R(i)-send_end(3));%注意这里road_in_relay_R虽然是以一列多行的形式排版的，但是dec_num却是以一行多列的形式排版，abs函数是求绝对值的意思
        end
        [~,rank] = min(dec_num);%因为dec_num是以一行多列的形式进行排版的，故通过min函数求出dec_num数组中绝对值最小的数的列下标
        Popt_road_index = road_in_relay_R(rank);%rank对应road_in_relay_R数组中的路段点编号即为最靠近send_end点的路段点编号
    else
        Popt_road_index = min(road_in_relay_R); %取出存储的最小的路段编号，因为最小的编号路段与扇形区域边界有交点，可能是半径边界与该路段有交点，可能是弧边界与该路段有交点
    end
    syms x y;
    eq1 = K_min*(y-relay(2))-(x-relay(1));%以K_min为斜率，以relay为点，做出直线eq1
    eq1=subs(eq1);
    %以路段标记点编号为Popt_road_index和Popt_road_index-1的两点（后简写为0和-1路段点），做出直线eq3，用0和-1路段求其与Kmin线的交点原因是Kmin所交路段是前向路段
    eq3 = (y-locationmark(Popt_road_index-1,2))*(locationmark(Popt_road_index,1)-locationmark(Popt_road_index-1,1))-(locationmark(Popt_road_index,2)-locationmark(Popt_road_index-1,2))*(x-locationmark(Popt_road_index-1,1));%以路段标记点编号为Popt_road_index-1和Popt_road_index的两点，做出直线eq3
    eq3=subs(eq3);
    [x,y]=solve(eq1,eq3);%求K_min直线与路段直线的交点
    x = eval([x]);
    y = eval([y]);
    %底下是画图看Kmin线、K线、Kmax线是否正确
   %  b3 = relay(1)-K*relay(2);
   %  y3 = 0:600;
   %  x3 = K*y3+b3;
   %   plot(x3,y3,'MarkerEdgeColor','g');%K直线，尝试将X和Y交换位置看能不能解决错误，然后成功解决错误，从这里得知，X和Y得反过来
   %   hold on
   %  b1 = relay(1)-K_min*relay(2);
   %  y1 = 0:600;
   %  x1 = K_min*y1+b1;
   %   plot(x1,y1,'MarkerEdgeColor','b');%K_min直线，将X和Y反了过来
   %   hold on
   %  b2 = relay(1)-K_max*relay(2);
   %  y2 = 0:600;
   %  x2 = K_max*y2+b2;
   %   plot(x2,y2,'MarkerEdgeColor','r');%K_max直线，将X和Y反了过来
   %   hold on
    %画图结束
    if (locationmark(Popt_road_index-1,1)<=x && x<=locationmark(Popt_road_index,1)) || (locationmark(Popt_road_index-1,1)>=x && x>=locationmark(Popt_road_index,1))%如果求出的x在Popt_road_index与Popt_road_index-1构成的路段上
        X = x;
        Y = y;
        plot3([locationmark(Popt_road_index-1,1) locationmark(Popt_road_index,1)],[locationmark(Popt_road_index-1,2) locationmark(Popt_road_index,2)],[0 0]);%画前向路段的线段
        hold on
    else %如果K_min直线与路段直线的交点不是我们求所的点，则我们求K_max直线与路段直线的交点
        syms x y;
        eq2 = K_max*(y-relay(2))-(x-relay(1));%以K_max为斜率，以relay为点，做出直线eq2
        eq2=subs(eq2);
        %以0和+1路段点，做出直线eq3，用0和+1路段求其与Kmax线的交点原因是Kmax所交路段是后向路段
        eq3 = (y-locationmark(Popt_road_index,2))*(locationmark(Popt_road_index+1,1)-locationmark(Popt_road_index,1))-(locationmark(Popt_road_index+1,2)-locationmark(Popt_road_index,2))*(x-locationmark(Popt_road_index,1));%以路段标记点编号为Popt_road_index和Popt_road_index+1的两点，做出直线eq3
        eq3=subs(eq3);
        [x,y]=solve(eq2,eq3);%求K_max直线与路段直线的交点
        x = eval([x]);
        y = eval([y]);
        if (locationmark(Popt_road_index,1)<=x && x<=locationmark(Popt_road_index+1,1)) || (locationmark(Popt_road_index,1)>=x && x>=locationmark(Popt_road_index+1,1))%如果这回求出的x在Popt_road_index与Popt_road_index+1构成的路段上
            X = x;
            Y = y;
            plot3([locationmark(Popt_road_index,1) locationmark(Popt_road_index+1,1)],[locationmark(Popt_road_index,2) locationmark(Popt_road_index+1,2)],[0 0]);%画后向路段的线段
            hold on
        else%表示扇形区域内的路段与扇形区域的半径边界没有交点，那么我们就求出路段与弧边界的交点
            syms x y;
            eq4=(x-relay(1))^2+(y-relay(2))^2-(R*Linmap1m)^2;%建立以relay为圆心以R为半径的圆eq4,R*Linmap1m表示将R化为图像中的对应大小
            eq4=subs(eq4);
            eq5 = (y-locationmark(Popt_road_index-1,2))*(locationmark(Popt_road_index,1)-locationmark(Popt_road_index-1,1))-(locationmark(Popt_road_index,2)-locationmark(Popt_road_index-1,2))*(x-locationmark(Popt_road_index-1,1));
            % eq5 = (x-locationmark(Popt_road_index-1,1))*(locationmark(Popt_road_index,2)-locationmark(Popt_road_index-1,2))-(locationmark(Popt_road_index,1)-locationmark(Popt_road_index-1,1))*(y-locationmark(Popt_road_index-1,2));%以路段标记点编号为Popt_road_index-1和Popt_road_index的两点，做出直线eq5
            eq5=subs(eq5);
            [x,y]=solve(eq4,eq5);%求圆eq4与路段直线eq5的交点
            x = eval([x]);
            y = eval([y]);
            %由于圆与直线的交点有两个，我们需要的是在路段Popt_road_index-1（路段标记点Popt_road_index与Popt_road_index-1构成的路段为Popt_road_index-1路段）上的那个交点，故通过以下两行代码选出
            X = x((locationmark(Popt_road_index-1,1)<=x & x<=locationmark(Popt_road_index,1))|(locationmark(Popt_road_index-1,1)>=x & x>=locationmark(Popt_road_index,1)));
            Y = y((locationmark(Popt_road_index-1,2)<=y & y<=locationmark(Popt_road_index,2))|(locationmark(Popt_road_index-1,2)>=y & y>=locationmark(Popt_road_index,2)));
            % straight_sign = 1;%并且标记进入直道中继节点选择
            if isempty(X) || isempty(Y)%如果发现-1和0路段与圆弧的交点不是我们要求的点，则求0和+1路段与圆弧的交点
                syms x y;
                eq4=(x-relay(1))^2+(y-relay(2))^2-(R*Linmap1m)^2;%建立以relay为圆心以R为半径的圆eq4,R*Linmap1m表示将R化为图像中的对应大小
                eq4=subs(eq4);
                eq6 = (y-locationmark(Popt_road_index,2))*(locationmark(Popt_road_index+1,1)-locationmark(Popt_road_index,1))-(locationmark(Popt_road_index+1,2)-locationmark(Popt_road_index,2))*(x-locationmark(Popt_road_index,1));
                eq6=subs(eq6);
                [x,y]=solve(eq4,eq6);%求圆eq4与路段直线eq5的交点
                x = eval([x]);
                y = eval([y]);
                %由于圆与直线的交点有两个，我们需要的是在路段Popt_road_index（路段标记点Popt_road_index与Popt_road_index+1构成的路段为Popt_road_index-1路段）上的那个交点，故通过以下两行代码选出
                X = x((locationmark(Popt_road_index,1)<=x & x<=locationmark(Popt_road_index+1,1))|(locationmark(Popt_road_index,1)>=x & x>=locationmark(Popt_road_index+1,1)));
                Y = y((locationmark(Popt_road_index,2)<=y & y<=locationmark(Popt_road_index+1,2))|(locationmark(Popt_road_index,2)>=y & y>=locationmark(Popt_road_index+1,2)));
                % straight_sign = 1;%并且标记进入直道中继节点选择
                %求出后向路段与圆弧边界的交点后，画出后向路段的图
                plot3([locationmark(Popt_road_index,1) locationmark(Popt_road_index+1,1)],[locationmark(Popt_road_index,2) locationmark(Popt_road_index+1,2)],[0 0]);%画后向路段的线段
                hold on
            else%如果X和Y都不是空，则说明我们的前向路段与弧边界的交点正确，则做出前向路段的图
                plot3([locationmark(Popt_road_index-1,1) locationmark(Popt_road_index,1)],[locationmark(Popt_road_index-1,2) locationmark(Popt_road_index,2)],[0 0]);%画前向路段的线段
                hold on
            end
        end
    end
    posi_opt(1) = X;
    posi_opt(2) = Y;
end