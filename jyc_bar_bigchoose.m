function [posi_opt] = jyc_bar_bigchoose(Popt_road_index,locationmark,relay,K,K_min,K_max,R,Linmap1m,straight_sign)
    %��������Ĭ��������Ϣ�㲥������������ң��ӱ�Ŵ󵽱��С��·�Σ�Ҳ�����Ӻ���ǰ
    %�����ǽ����㲥Ҳ��������·�ι㲥
    plot3([locationmark(Popt_road_index,1) locationmark(Popt_road_index+1,1)],[locationmark(Popt_road_index,2) locationmark(Popt_road_index+1,2)],[0 0]);%������·�ε��߶�
    hold on
    syms x y;
    if straight_sign == 1 %�����ֱ��ģʽ
        syms x y;
        eq4=(x-relay(1))^2+(y-relay(2))^2-(R*Linmap1m)^2;%������relayΪԲ����RΪ�뾶��Բeq4
        eq4=subs(eq4);
        %��Բ���߽������·�εĽ��㣬������Ҫ��ĵ�ɸѡ����
        eq5 = (y-locationmark(Popt_road_index,2))*(locationmark(Popt_road_index+1,1)-locationmark(Popt_road_index,1))-(locationmark(Popt_road_index+1,2)-locationmark(Popt_road_index,2))*(x-locationmark(Popt_road_index,1));%��·�α�ǵ���ΪPopt_road_index��Popt_road_index+1�����㣬����ֱ��eq5
        eq5=subs(eq5);
        [x,y]=solve(eq4,eq5);%��Բeq4��·��ֱ��eq5�Ľ���
        x = eval([x]);
        y = eval([y]);
        %����Բ��ֱ�ߵĽ�����������������Ҫ������·��Popt_road_index��·�α�ǵ�Popt_road_index��Popt_road_index+1���ɵ�·��ΪPopt_road_index·�Σ��ϵ��Ǹ����㣬��ͨ���������д���ѡ��
        if (locationmark(Popt_road_index,1)<=x(1) && x(1)<=locationmark(Popt_road_index+1,1)) || (locationmark(Popt_road_index,1)>=x(1) && x(1)>=locationmark(Popt_road_index+1,1))%�����������x��Popt_road_index��Popt_road_index+1���ɵ�·����
            X = x;
            Y = y;
        elseif (locationmark(Popt_road_index,1)<=x(2) && x(2)<=locationmark(Popt_road_index+1,1)) || (locationmark(Popt_road_index,1)>=x(2) && x(2)>=locationmark(Popt_road_index+1,1))
            X = x;
            Y = y;
        end
    else
        eq1 = K_min*(y-relay(2))-(x-relay(1));%��K_minΪб�ʣ���relayΪ�㣬����ֱ��eq1
        eq1=subs(eq1);
        %��·�α�ǵ���ΪPopt_road_index��Popt_road_index+1�����㣨���дΪ0��+1·�ε㣩������ֱ��eq3����0��+1·��������Kmin�ߵĽ���ԭ����Kmin����·���Ǻ���·��
        eq3 = (y-locationmark(Popt_road_index,2))*(locationmark(Popt_road_index+1,1)-locationmark(Popt_road_index,1))-(locationmark(Popt_road_index+1,2)-locationmark(Popt_road_index,2))*(x-locationmark(Popt_road_index,1));
        eq3=subs(eq3);
        [x,y]=solve(eq1,eq3);%��K_minֱ����·��ֱ�ߵĽ���
        x = eval([x]);
        y = eval([y]);
        if (locationmark(Popt_road_index,1)<=x && x<=locationmark(Popt_road_index+1,1)) || (locationmark(Popt_road_index,1)>=x && x>=locationmark(Popt_road_index+1,1))%��������x��Popt_road_index��Popt_road_index+1���ɵ�·����
            X = x;
            Y = y;
        else %���K_minֱ����ǰ��·��ֱ�ߵĽ��㲻�����������ĵ㣬��������K_maxֱ�������·��ֱ�ߵĽ���
            syms x y;
            eq2 = K_max*(y-relay(2))-(x-relay(1));%��K_maxΪб�ʣ���relayΪ�㣬����ֱ��eq2
            eq2=subs(eq2);
            eq3 = (y-locationmark(Popt_road_index,2))*(locationmark(Popt_road_index+1,1)-locationmark(Popt_road_index,1))-(locationmark(Popt_road_index+1,2)-locationmark(Popt_road_index,2))*(x-locationmark(Popt_road_index,1));%��·�α�ǵ���ΪPopt_road_index��Popt_road_index+1�����㣬����ֱ��eq3
            eq3=subs(eq3);
            [x,y]=solve(eq2,eq3);%��K_maxֱ����·��ֱ�ߵĽ���
            x = eval([x]);
            y = eval([y]);
            if (locationmark(Popt_road_index,1)<=x && x<=locationmark(Popt_road_index+1,1)) || (locationmark(Popt_road_index,1)>=x && x>=locationmark(Popt_road_index+1,1))%�����������x��Popt_road_index��Popt_road_index+1���ɵ�·����
                X = x;
                Y = y;
            else%��ʾ���������ڵ�·������������İ뾶�߽�û�н��㣬��ô���Ǿ����·���뻡�߽�Ľ���
                syms x y;
                eq4=(x-relay(1))^2+(y-relay(2))^2-(R*Linmap1m)^2;%������relayΪԲ����RΪ�뾶��Բeq4
                eq4=subs(eq4);
                %��Բ���߽������·�εĽ��㣬������Ҫ��ĵ�ɸѡ����
                eq5 = (y-locationmark(Popt_road_index,2))*(locationmark(Popt_road_index+1,1)-locationmark(Popt_road_index,1))-(locationmark(Popt_road_index+1,2)-locationmark(Popt_road_index,2))*(x-locationmark(Popt_road_index,1));%��·�α�ǵ���ΪPopt_road_index��Popt_road_index+1�����㣬����ֱ��eq5
                eq5=subs(eq5);
                [x,y]=solve(eq4,eq5);%��Բeq4��·��ֱ��eq5�Ľ���
                x = eval([x]);
                y = eval([y]);
                %����Բ��ֱ�ߵĽ�����������������Ҫ������·��Popt_road_index��·�α�ǵ�Popt_road_index��Popt_road_index+1���ɵ�·��ΪPopt_road_index·�Σ��ϵ��Ǹ����㣬��ͨ���������д���ѡ��
                if (locationmark(Popt_road_index,1)<=x(1) && x(1)<=locationmark(Popt_road_index+1,1)) || (locationmark(Popt_road_index,1)>=x(1) && x(1)>=locationmark(Popt_road_index+1,1))%�����������x��Popt_road_index��Popt_road_index+1���ɵ�·����
                    X = x;
                    Y = y;
                elseif (locationmark(Popt_road_index,1)<=x(2) && x(2)<=locationmark(Popt_road_index+1,1)) || (locationmark(Popt_road_index,1)>=x(2) && x(2)>=locationmark(Popt_road_index+1,1))
                    X = x;
                    Y = y;
                end
                %straight_sign = 1;%���ұ�ǽ���ֱ���м̽ڵ�ѡ��
            end
        end
    end
    posi_opt(1) = X;
    posi_opt(2) = Y;
end