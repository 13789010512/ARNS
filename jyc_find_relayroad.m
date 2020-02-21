 function  [relay_road_index]=jyc_find_relayroad(locationmark,relay)

     %������ȷ��relay����·�Σ�����֮��ѡ����relay��·��ǰ���Popt�㣬ǰ�����˼�Ǹ�������Ϣ�����յ�send_end
    %locationmark(:,2) = locationmark(:,2)-sizey;
    relay_road_indexby_x = zeros(length(locationmark(:,1)),1);
    relay_road_indexby_y = zeros(length(locationmark(:,1)),1);
    for m = 1:length(locationmark(:,1))-1
        if (locationmark(m,1)<relay(1) && relay(1)<locationmark(m+1,1)) || (locationmark(m,1)>relay(1) && relay(1)>locationmark(m+1,1))
            relay_road_indexby_x(m) = m;
        elseif locationmark(m,1)==relay(1) %���locationmark(m,1)����relay(1)����ô��¼relay�����ڵ�m-1·�Σ���ǰ��������m-1·���ɱ�ǵ�m-1��m���߹���
            if m==1
                relay_road_indexby_x(m) = m;
            else
                relay_road_indexby_x(m-1) = m-1;
            end
        elseif locationmark(m+1,1)==relay(1)%���locationmark(m+1,1)����relay(1)����ô��¼relay�����ڵ�m·�Σ���ǰ��������m·���ɱ�ǵ�m��m+1���߹���
            relay_road_indexby_x(m) = m;
        end
    end
    for m = 1:length(locationmark(:,1))-1
        if (locationmark(m,2)<relay(2) && relay(2)<locationmark(m+1,2)) || (locationmark(m,2)>relay(2) && relay(2)>locationmark(m+1,2))
            relay_road_indexby_y(m) = m;
        elseif locationmark(m,2)==relay(2)
            if m==1
                relay_road_indexby_y(m) = m;
            else
                relay_road_indexby_y(m-1) = m-1;
            end
        elseif locationmark(m+1,2)==relay(2)
            relay_road_indexby_y(m) = m;
        end
    end
    relay_road_index = intersect(relay_road_indexby_x,relay_road_indexby_y);%relay_road_index��¼����relay����·�Σ���������·�α�ǵ�ȷ��һ��·�Σ����������ｫ·�κż�Ϊ��Ӧ������·�α�ǵ�����С���Ǹ���ǵ�ı��
    relay_road_index(relay_road_index==0)=[];
    if isempty(relay_road_index)
        %�������ѡ����relay_road_index������������������һ����relay����locationmark����߾���locationmark�㸽������
        for i = 1:length(locationmark(:,1))
            if length(locationmark(:,1))==45 && abs(locationmark(i,1)-relay(1))<2 && abs(locationmark(i,2)-relay(2))<2 %length(locationmark(:,1))==45��ʾ���ڴ���������������Ƕ����������һ���İ��ݣ�ֻҪrelay��������locationmark���2��Χ�ڲ��������Ǿ��϶�relay�������Ǹ�mark������·��
                relay_road_index = i;%relay����locationmark�����relay����locationmark�㸽�����ƣ���ô���ǿ��Խ�relay����·�θ�ֵΪlocationmark���ţ���������Ӧ�ø�ֵΪ���-1�����������ﲻ-1��ԭ�����ڼ�����һ��ֱ··�ηǳ��ĳ������relay�ڸ�ֱ·��β�ʹ���ȴ�����ڸ�ֱ·�ϣ���ô����ѡrelayǰ��·���ϵ��м̽ڵ㣬��ѡ����������ֱ··���ϵĽڵ㣬��Ϊ���ǽ�relay����·���ų���
                break;
            elseif abs(locationmark(i,1)-relay(1))<1 && abs(locationmark(i,2)-relay(2))<1%���ڷ���������������ϸ�㣬��Ҫrelay��������locationmark���1��Χ�ڲ��������ǲŻ��϶�relay���ڸ�mark������·��
                relay_road_index = i;
                break;
            else
                relay_road_index = length(locationmark(:,1));
            end
        end
    end
    error_sign = length(relay_road_index);
    if error_sign~=1%���������relay_road_index���в�ֹһ���м̽ڵ�·�α�ǵ�����
        for i = 1:error_sign%���relay_road_index�е�ÿһ���м̽ڵ�·�α�ǵ���������жϣ�����relay_yΪ����relay��X��������������Y���꣬�����Ƿ����ʵ��Y������ȣ������˵��relay�ڸ�·����
            relay_y = locationmark(relay_road_index(i),2)+(locationmark(relay_road_index(i)+1,2)-locationmark(relay_road_index(i),2))*((relay(1)-locationmark(relay_road_index(i),1))/(locationmark(relay_road_index(i)+1,1)-locationmark(relay_road_index(i),1)));
            if relay_y ~= relay(2)%�����relay_y�Ǹ���relay_road_index�е��м̽ڵ�·�α�ǵ�����ֱ�߷��̣�����relay(1)��ֵ�����������relay_yֵ���������relay_y���겻����relay(2)����˵���м̽ڵ�relay��ʵ�����ڸ�·����
                relay_road_index(i)=0;%���м̽ڵ㲻�ڵ�·�α�ǵ��relay_road_index������ȥ��
            end
        end
        relay_road_index(relay_road_index==0) = [];%��relay_road_indexһά�����е���Ԫ��ȥ��
    end
    error_sign = length(relay_road_index);
    if error_sign~=1
        for i = 1:error_sign
            if relay_road_index(i)~=1 && locationmark(relay_road_index(i)+1,1) == relay(1) && locationmark(relay_road_index(i)+1,2) == relay(2)%�������Ϊrelay�պ���·�α�ǵ㣨����relay_road_index(i)+1����Ϊ���������·�α�ǵ����relay�Ļ���relay_road_index�д洢��relay����·��Ϊ��·�α�ǵ�����-1��
                temp = relay_road_index(i);
                relay_road_index = [];
                relay_road_index = temp;
                break;
            end
        end
    end
    error_sign = length(relay_road_index);
    if error_sign~=1
        for i = 1:error_sign-1
            if abs(relay_road_index(i)-relay_road_index(i+1))==1%���·�α��֮��ֻ��1����ô���Ǿ����ѡһ��·�α����Ϊrelay����·�α��
                temp = relay_road_index(i);
                relay_road_index = [];
                relay_road_index = temp;
            end
        end
    end
 end