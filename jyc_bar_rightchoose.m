function[posi_opt] = jyc_bar_rightchoose(road_in_relay_R,locationmark,relay,K,K_min,K_max,R,Linmap1m,send_end)
    %��������Ĭ�������㲥��Ϣ�����Ǵ�·�α�Ŵ�ĵط��㲥��·�α��С�ĵط�
    %�����������㲥ʱֻ��Ҫѡ�� road_in_relay_R �����д洢��·�α����С��·����Ϊ���ǵ�Popt����·�μ���
    if length(send_end) ==3%������ڷ���㲥����ѡ�����send_end���·�ε���Ϊ����㲥Popt����·�ε�
        for i = 1:length(road_in_relay_R)%��ʼ��road_in_relay_R�е�·�α�Ž����ж�
           dec_num(i) = abs(road_in_relay_R(i)-send_end(3));%ע������road_in_relay_R��Ȼ����һ�ж��е���ʽ�Ű�ģ�����dec_numȴ����һ�ж��е���ʽ�Ű棬abs�����������ֵ����˼
        end
        [~,rank] = min(dec_num);%��Ϊdec_num����һ�ж��е���ʽ�����Ű�ģ���ͨ��min�������dec_num�����о���ֵ��С���������±�
        Popt_road_index = road_in_relay_R(rank);%rank��Ӧroad_in_relay_R�����е�·�ε��ż�Ϊ���send_end���·�ε���
    else
        Popt_road_index = min(road_in_relay_R); %ȡ���洢����С��·�α�ţ���Ϊ��С�ı��·������������߽��н��㣬�����ǰ뾶�߽����·���н��㣬�����ǻ��߽����·���н���
    end
    syms x y;
    eq1 = K_min*(y-relay(2))-(x-relay(1));%��K_minΪб�ʣ���relayΪ�㣬����ֱ��eq1
    eq1=subs(eq1);
    %��·�α�ǵ���ΪPopt_road_index��Popt_road_index-1�����㣨���дΪ0��-1·�ε㣩������ֱ��eq3����0��-1·��������Kmin�ߵĽ���ԭ����Kmin����·����ǰ��·��
    eq3 = (y-locationmark(Popt_road_index-1,2))*(locationmark(Popt_road_index,1)-locationmark(Popt_road_index-1,1))-(locationmark(Popt_road_index,2)-locationmark(Popt_road_index-1,2))*(x-locationmark(Popt_road_index-1,1));%��·�α�ǵ���ΪPopt_road_index-1��Popt_road_index�����㣬����ֱ��eq3
    eq3=subs(eq3);
    [x,y]=solve(eq1,eq3);%��K_minֱ����·��ֱ�ߵĽ���
    x = eval([x]);
    y = eval([y]);
    %�����ǻ�ͼ��Kmin�ߡ�K�ߡ�Kmax���Ƿ���ȷ
   %  b3 = relay(1)-K*relay(2);
   %  y3 = 0:600;
   %  x3 = K*y3+b3;
   %   plot(x3,y3,'MarkerEdgeColor','g');%Kֱ�ߣ����Խ�X��Y����λ�ÿ��ܲ��ܽ������Ȼ��ɹ�������󣬴������֪��X��Y�÷�����
   %   hold on
   %  b1 = relay(1)-K_min*relay(2);
   %  y1 = 0:600;
   %  x1 = K_min*y1+b1;
   %   plot(x1,y1,'MarkerEdgeColor','b');%K_minֱ�ߣ���X��Y���˹���
   %   hold on
   %  b2 = relay(1)-K_max*relay(2);
   %  y2 = 0:600;
   %  x2 = K_max*y2+b2;
   %   plot(x2,y2,'MarkerEdgeColor','r');%K_maxֱ�ߣ���X��Y���˹���
   %   hold on
    %��ͼ����
    if (locationmark(Popt_road_index-1,1)<=x && x<=locationmark(Popt_road_index,1)) || (locationmark(Popt_road_index-1,1)>=x && x>=locationmark(Popt_road_index,1))%��������x��Popt_road_index��Popt_road_index-1���ɵ�·����
        X = x;
        Y = y;
        plot3([locationmark(Popt_road_index-1,1) locationmark(Popt_road_index,1)],[locationmark(Popt_road_index-1,2) locationmark(Popt_road_index,2)],[0 0]);%��ǰ��·�ε��߶�
        hold on
    else %���K_minֱ����·��ֱ�ߵĽ��㲻�����������ĵ㣬��������K_maxֱ����·��ֱ�ߵĽ���
        syms x y;
        eq2 = K_max*(y-relay(2))-(x-relay(1));%��K_maxΪб�ʣ���relayΪ�㣬����ֱ��eq2
        eq2=subs(eq2);
        %��0��+1·�ε㣬����ֱ��eq3����0��+1·��������Kmax�ߵĽ���ԭ����Kmax����·���Ǻ���·��
        eq3 = (y-locationmark(Popt_road_index,2))*(locationmark(Popt_road_index+1,1)-locationmark(Popt_road_index,1))-(locationmark(Popt_road_index+1,2)-locationmark(Popt_road_index,2))*(x-locationmark(Popt_road_index,1));%��·�α�ǵ���ΪPopt_road_index��Popt_road_index+1�����㣬����ֱ��eq3
        eq3=subs(eq3);
        [x,y]=solve(eq2,eq3);%��K_maxֱ����·��ֱ�ߵĽ���
        x = eval([x]);
        y = eval([y]);
        if (locationmark(Popt_road_index,1)<=x && x<=locationmark(Popt_road_index+1,1)) || (locationmark(Popt_road_index,1)>=x && x>=locationmark(Popt_road_index+1,1))%�����������x��Popt_road_index��Popt_road_index+1���ɵ�·����
            X = x;
            Y = y;
            plot3([locationmark(Popt_road_index,1) locationmark(Popt_road_index+1,1)],[locationmark(Popt_road_index,2) locationmark(Popt_road_index+1,2)],[0 0]);%������·�ε��߶�
            hold on
        else%��ʾ���������ڵ�·������������İ뾶�߽�û�н��㣬��ô���Ǿ����·���뻡�߽�Ľ���
            syms x y;
            eq4=(x-relay(1))^2+(y-relay(2))^2-(R*Linmap1m)^2;%������relayΪԲ����RΪ�뾶��Բeq4,R*Linmap1m��ʾ��R��Ϊͼ���еĶ�Ӧ��С
            eq4=subs(eq4);
            eq5 = (y-locationmark(Popt_road_index-1,2))*(locationmark(Popt_road_index,1)-locationmark(Popt_road_index-1,1))-(locationmark(Popt_road_index,2)-locationmark(Popt_road_index-1,2))*(x-locationmark(Popt_road_index-1,1));
            % eq5 = (x-locationmark(Popt_road_index-1,1))*(locationmark(Popt_road_index,2)-locationmark(Popt_road_index-1,2))-(locationmark(Popt_road_index,1)-locationmark(Popt_road_index-1,1))*(y-locationmark(Popt_road_index-1,2));%��·�α�ǵ���ΪPopt_road_index-1��Popt_road_index�����㣬����ֱ��eq5
            eq5=subs(eq5);
            [x,y]=solve(eq4,eq5);%��Բeq4��·��ֱ��eq5�Ľ���
            x = eval([x]);
            y = eval([y]);
            %����Բ��ֱ�ߵĽ�����������������Ҫ������·��Popt_road_index-1��·�α�ǵ�Popt_road_index��Popt_road_index-1���ɵ�·��ΪPopt_road_index-1·�Σ��ϵ��Ǹ����㣬��ͨ���������д���ѡ��
            X = x((locationmark(Popt_road_index-1,1)<=x & x<=locationmark(Popt_road_index,1))|(locationmark(Popt_road_index-1,1)>=x & x>=locationmark(Popt_road_index,1)));
            Y = y((locationmark(Popt_road_index-1,2)<=y & y<=locationmark(Popt_road_index,2))|(locationmark(Popt_road_index-1,2)>=y & y>=locationmark(Popt_road_index,2)));
            % straight_sign = 1;%���ұ�ǽ���ֱ���м̽ڵ�ѡ��
            if isempty(X) || isempty(Y)%�������-1��0·����Բ���Ľ��㲻������Ҫ��ĵ㣬����0��+1·����Բ���Ľ���
                syms x y;
                eq4=(x-relay(1))^2+(y-relay(2))^2-(R*Linmap1m)^2;%������relayΪԲ����RΪ�뾶��Բeq4,R*Linmap1m��ʾ��R��Ϊͼ���еĶ�Ӧ��С
                eq4=subs(eq4);
                eq6 = (y-locationmark(Popt_road_index,2))*(locationmark(Popt_road_index+1,1)-locationmark(Popt_road_index,1))-(locationmark(Popt_road_index+1,2)-locationmark(Popt_road_index,2))*(x-locationmark(Popt_road_index,1));
                eq6=subs(eq6);
                [x,y]=solve(eq4,eq6);%��Բeq4��·��ֱ��eq5�Ľ���
                x = eval([x]);
                y = eval([y]);
                %����Բ��ֱ�ߵĽ�����������������Ҫ������·��Popt_road_index��·�α�ǵ�Popt_road_index��Popt_road_index+1���ɵ�·��ΪPopt_road_index-1·�Σ��ϵ��Ǹ����㣬��ͨ���������д���ѡ��
                X = x((locationmark(Popt_road_index,1)<=x & x<=locationmark(Popt_road_index+1,1))|(locationmark(Popt_road_index,1)>=x & x>=locationmark(Popt_road_index+1,1)));
                Y = y((locationmark(Popt_road_index,2)<=y & y<=locationmark(Popt_road_index+1,2))|(locationmark(Popt_road_index,2)>=y & y>=locationmark(Popt_road_index+1,2)));
                % straight_sign = 1;%���ұ�ǽ���ֱ���м̽ڵ�ѡ��
                %�������·����Բ���߽�Ľ���󣬻�������·�ε�ͼ
                plot3([locationmark(Popt_road_index,1) locationmark(Popt_road_index+1,1)],[locationmark(Popt_road_index,2) locationmark(Popt_road_index+1,2)],[0 0]);%������·�ε��߶�
                hold on
            else%���X��Y�����ǿգ���˵�����ǵ�ǰ��·���뻡�߽�Ľ�����ȷ��������ǰ��·�ε�ͼ
                plot3([locationmark(Popt_road_index-1,1) locationmark(Popt_road_index,1)],[locationmark(Popt_road_index-1,2) locationmark(Popt_road_index,2)],[0 0]);%��ǰ��·�ε��߶�
                hold on
            end
        end
    end
    posi_opt(1) = X;
    posi_opt(2) = Y;
end