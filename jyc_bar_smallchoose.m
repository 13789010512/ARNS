function[posi_opt] = jyc_bar_smallchoose(Popt_road_index,locationmark,relay,K,K_min,K_max,R,Linmap1m,straight_sign)
    %由于我们默认正常消息广播是整体从左至右，从编号大到编号小的路段，也称作从后至前
    %故我们将向小广播也称作前向路段广播
        plot3([locationmark(Popt_road_index-1,1) locationmark(Popt_road_index,1)],[locationmark(Popt_road_index-1,2) locationmark(Popt_road_index,2)],[0 0]);%画Popt点所在路段的线段
        hold on
        syms x y;        
        if straight_sign == 1 %如果是直道模式
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
            if (locationmark(Popt_road_index-1,1)<=x(1) && x(1)<=locationmark(Popt_road_index,1)) || (locationmark(Popt_road_index-1,1)>=x(1) && x(1)>=locationmark(Popt_road_index,1))%如果这回求出的x在Popt_road_index与Popt_road_index+1构成的路段上
                X = x(1);
                Y = y(1);
            elseif (locationmark(Popt_road_index-1,1)<=x(2) && x(2)<=locationmark(Popt_road_index,1)) || (locationmark(Popt_road_index-1,1)>=x(2) && x(2)>=locationmark(Popt_road_index,1))
                X = x(2);
                Y = y(2);
            end
        else
            eq1 = K_min*(y-relay(2))-(x-relay(1));%以K_min为斜率，以relay为点，做出直线eq1
            eq1=subs(eq1);
            %以路段标记点编号为Popt_road_index和Popt_road_index-1的两点（后简写为0和-1路段点），做出直线eq3，用0和-1路段求其与Kmin线的交点原因是Kmin所交路段是前向路段
            eq3 = (y-locationmark(Popt_road_index-1,2))*(locationmark(Popt_road_index,1)-locationmark(Popt_road_index-1,1))-(locationmark(Popt_road_index,2)-locationmark(Popt_road_index-1,2))*(x-locationmark(Popt_road_index-1,1));%以路段标记点编号为Popt_road_index-1和Popt_road_index的两点，做出直线eq3
            eq3=subs(eq3);
            [x,y]=solve(eq1,eq3);%求K_min直线与路段直线的交点
            x = eval([x]);
            y = eval([y]);
            if (locationmark(Popt_road_index-1,1)<=x && x<=locationmark(Popt_road_index,1)) || (locationmark(Popt_road_index-1,1)>=x && x>=locationmark(Popt_road_index,1))%如果求出的x在Popt_road_index与Popt_road_index-1构成的路段上
                X = x;
                Y = y;
            else %如果K_min直线与路段直线的交点不是我们求所的点，则我们求K_max直线与路段直线的交点
                syms x y;
                eq2 = K_max*(y-relay(2))-(x-relay(1));%以K_max为斜率，以relay为点，做出直线eq2
                eq2=subs(eq2);
                eq3 = (y-locationmark(Popt_road_index-1,2))*(locationmark(Popt_road_index,1)-locationmark(Popt_road_index-1,1))-(locationmark(Popt_road_index,2)-locationmark(Popt_road_index-1,2))*(x-locationmark(Popt_road_index-1,1));%以路段标记点编号为Popt_road_index-1和Popt_road_index的两点，做出直线eq3
                eq3=subs(eq3);
                [x,y]=solve(eq2,eq3);%求K_max直线与路段直线的交点
                x = eval([x]);
                y = eval([y]);
                if (locationmark(Popt_road_index-1,1)<=x && x<=locationmark(Popt_road_index,1)) || (locationmark(Popt_road_index-1,1)>=x && x>=locationmark(Popt_road_index,1))%如果这回求出的x在Popt_road_index与Popt_road_index+1构成的路段上
                    X = x;
                    Y = y;
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
                    plot (x(1),y(1),'*','LineWidth',1,'MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',8);
                    hold on;
                    plot (x(2),y(2),'*','LineWidth',1,'MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',8);
                    hold on;
                    %由于圆与直线的交点有两个，我们需要的是在路段Popt_road_index-1（路段标记点Popt_road_index与Popt_road_index-1构成的路段为Popt_road_index-1路段）上的那个交点，故通过以下两行代码选出
                    if (locationmark(Popt_road_index-1,1)<=x(1) && x(1)<=locationmark(Popt_road_index,1)) || (locationmark(Popt_road_index-1,1)>=x(1) && x(1)>=locationmark(Popt_road_index,1))%如果这回求出的x在Popt_road_index与Popt_road_index+1构成的路段上
                        X = x(1);
                        Y = y(1);
                    elseif (locationmark(Popt_road_index-1,1)<=x(2) && x(2)<=locationmark(Popt_road_index,1)) || (locationmark(Popt_road_index-1,1)>=x(2) && x(2)>=locationmark(Popt_road_index,1))
                        X = x(2);
                        Y = y(2);
                    end
                    % straight_sign = 1;%并且标记进入直道中继节点选择
                end
            end
        end
        posi_opt(1) = X;
        posi_opt(2) = Y;
end