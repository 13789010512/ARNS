 function  [relay_road_index]=jyc_find_relayroad(locationmark,relay)

     %接下来确定relay所在路段，便于之后选出在relay点路段前面的Popt点，前面的意思是更靠近消息传播终点send_end
    %locationmark(:,2) = locationmark(:,2)-sizey;
    relay_road_indexby_x = zeros(length(locationmark(:,1)),1);
    relay_road_indexby_y = zeros(length(locationmark(:,1)),1);
    for m = 1:length(locationmark(:,1))-1
        if (locationmark(m,1)<relay(1) && relay(1)<locationmark(m+1,1)) || (locationmark(m,1)>relay(1) && relay(1)>locationmark(m+1,1))
            relay_road_indexby_x(m) = m;
        elseif locationmark(m,1)==relay(1) %如果locationmark(m,1)等于relay(1)，那么记录relay可能在第m-1路段（向前看），第m-1路段由标记点m-1和m连线构成
            if m==1
                relay_road_indexby_x(m) = m;
            else
                relay_road_indexby_x(m-1) = m-1;
            end
        elseif locationmark(m+1,1)==relay(1)%如果locationmark(m+1,1)等于relay(1)，那么记录relay可能在第m路段（向前看），第m路段由标记点m和m+1连线构成
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
    relay_road_index = intersect(relay_road_indexby_x,relay_road_indexby_y);%relay_road_index记录的是relay所在路段，由于两个路段标记点确定一条路段，故我们这里将路段号记为对应的两个路段标记点中最小的那个标记点的编号
    relay_road_index(relay_road_index==0)=[];
    if isempty(relay_road_index)
        %用来解决选不出relay_road_index的情况，出现这种情况一般是relay就是locationmark点或者就在locationmark点附近环绕
        for i = 1:length(locationmark(:,1))
            if length(locationmark(:,1))==45 && abs(locationmark(i,1)-relay(1))<2 && abs(locationmark(i,2)-relay(2))<2 %length(locationmark(:,1))==45表示现在处于弯道场景，我们对弯道场景有一定的包容，只要relay的坐标在locationmark点的2范围内波动，我们就认定relay点属于那个mark点所在路段
                relay_road_index = i;%relay就是locationmark点或者relay就在locationmark点附近环绕，那么我们可以将relay所在路段赋值为locationmark点编号，本来我们应该赋值为编号-1，但我们这里不-1，原因在于假如有一条直路路段非常的长，如果relay在该直路的尾巴处，却被归于该直路上，那么我们选relay前向路段上的中继节点，就选不到这条长直路路段上的节点，因为我们将relay所在路段排除了
                break;
            elseif abs(locationmark(i,1)-relay(1))<1 && abs(locationmark(i,2)-relay(2))<1%对于非弯道场景，我们严格点，需要relay的坐标在locationmark点的1范围内波动，我们才会认定relay属于该mark点所在路段
                relay_road_index = i;
                break;
            else
                relay_road_index = length(locationmark(:,1));
            end
        end
    end
    error_sign = length(relay_road_index);
    if error_sign~=1%如果出现了relay_road_index中有不止一个中继节点路段标记点的情况
        for i = 1:error_sign%则对relay_road_index中的每一个中继节点路段标记点进行以下判断，以下relay_y为带入relay的X坐标后求出的理论Y坐标，看其是否和真实的Y坐标相等，相等则说明relay在该路段上
            relay_y = locationmark(relay_road_index(i),2)+(locationmark(relay_road_index(i)+1,2)-locationmark(relay_road_index(i),2))*((relay(1)-locationmark(relay_road_index(i),1))/(locationmark(relay_road_index(i)+1,1)-locationmark(relay_road_index(i),1)));
            if relay_y ~= relay(2)%求出的relay_y是根据relay_road_index中的中继节点路段标记点所在直线方程，带入relay(1)的值后求出的理论relay_y值，如果理论relay_y坐标不等于relay(2)，则说明中继节点relay其实并不在该路段上
                relay_road_index(i)=0;%将中继节点不在的路段标记点从relay_road_index数组中去掉
            end
        end
        relay_road_index(relay_road_index==0) = [];%将relay_road_index一维数组中的零元素去掉
    end
    error_sign = length(relay_road_index);
    if error_sign~=1
        for i = 1:error_sign
            if relay_road_index(i)~=1 && locationmark(relay_road_index(i)+1,1) == relay(1) && locationmark(relay_road_index(i)+1,2) == relay(2)%如果是因为relay刚好是路段标记点（这里relay_road_index(i)+1是因为，如果我们路段标记点等于relay的话，relay_road_index中存储的relay所在路段为该路段标记点的序号-1）
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
            if abs(relay_road_index(i)-relay_road_index(i+1))==1%如果路段编号之间只差1，那么我们就随便选一个路段编号作为relay所在路段编号
                temp = relay_road_index(i);
                relay_road_index = [];
                relay_road_index = temp;
            end
        end
    end
 end