function [location_vehi,num_vehi]=generatetvehi(density,locationjucntion,branchset,R,Linmap1m,locationbar)
%%%生成道路像素信息，及表示出道路轮廓及十字路口
%在仿真图中，道路分为两车道、四车道、六车道三种，本函数考虑车道数，根据单车道车辆节点密度*车道数，得出多车道的车辆节点密度，然后依据该密度生成车辆节点


%初始化一个原始消息发送车辆，位于10-12路段的10位置
location_mark = branchset{2,1};
num_vehi=0;
location_vehi = zeros(1000,4);%1:序号；2：x坐标；3：y坐标；4：seg备用
location_vehi(1,1) = 1;
location_vehi(1,2:3) = location_mark(1,2:3);
num_vehi = num_vehi+1;
%以上初始化原始消息发送车辆完毕
for r = 1:length(branchset(1,:))
    location_mark = branchset{2,r};
    roadname = branchset{1,r}(1:5);
    if strcmp(roadname,'10-12') || strcmp(roadname,'12-13') || strcmp(roadname,'13-14') || strcmp(roadname,'14-03')
        roadnum = 6;%车道数为六车道
    elseif strcmp(roadname,'12-15') || strcmp(roadname,'15-02') || strcmp(roadname,'13-17') || strcmp(roadname,'17-18') || strcmp(roadname,'18-19') || strcmp(roadname,'19-06')
        roadnum = 4;%车道数为四车道
    else
        roadnum = 2;%车道数为二车道
    end
    l_road = sum((location_mark(2:length(location_mark(:,1)),2:3)-location_mark(1:length(location_mark(:,1))-1,2:3)).^2,2).^0.5/Linmap1m;%计算在整个道路上的由mark确定的第i条路段的现实长度
    l_road_all = sum(l_road);%路段的现实总长度
    
    %绘制关心路段
    for i=1:length(location_mark(:,1))-1
        n_seg_vehi = random('Poisson',density*l_road(i)*roadnum,1,1); %随机产生在整个道路上的由mark确定的第i条路段上的车辆数
        A = location_mark(i,2:3);
        B = location_mark(i+1,2:3);
        
        %可能底下这段函数里面有问题,很大可能是条件问题（已解决）
        %思路：城市情景下，当l_road路段到了第4段的时候，明明现实路长为284米左右，但是在底下的deltaX（deltaX为每米对应的图像像素值）折算下，第4段的长度竟然到了8000多米（即X数组的列数有8000多列）？（已解决）
        if B(1)~= A(1)&& B(2)~= A(2)
            deltaX = (B(1)-A(1))/l_road(i);%以道路长度1米为间隔确定车辆图上像素点位置,即算出每个路段每米的X、Y坐标的变化
            X = A(1): deltaX:B(1);
            Y = (B(2)-A(2))/(B(1)-A(1))*(X-A(1))+A(2);
        else
            if B(1)== A(1)
                deltaX = (B(2)-A(2))/l_road(i);
                Y = A(2):deltaX:B(2);
                X = A(1)*ones(1,length(Y));
            else
                deltaX = (B(1)-A(1))/l_road(i);%deltaX值对应现实中的每米
                X = A(1):deltaX:B(1);
                Y = A(2)*ones(1,length(X));
            end
        end
        
        
        if n_seg_vehi ~= 0
            if isempty(X) %如果X是空的，则会执行下面a=1的操作
                a=1;
            end
            location_index=randi(length(X),1,n_seg_vehi);    %将数目为n_seg_vehi的车辆随机的放入第i段的某一列（米）中，该列下标存入location_index中
           
            %如果随机生成的车辆之间间距小于4米，那么再重新分配一次，这里设置4米间隔，主要是因为考虑了车身所占空间，和车辆间安全距离
            [rank_ar,~] = sort(location_index);%rank_ar是对location_index中的数排序后的数组，方便我们进行之后的间隔判断
            if length(rank_ar)>1%如果路段车辆多于一台，那么就要进行4米间隔判断
                if abs(rank_ar(2:length(rank_ar))-rank_ar(1:length(rank_ar)-1))<=4%如果车辆间间隔小于4米
                    location_index=randi(length(X),1,n_seg_vehi);%重新分配车辆分布
                end
            end
            %以上if语句完成4米间隔设置
            
            if length(location_index) ~= 1
                while isempty(find(location_index-R/2<0,1)) || isempty(find(location_index+R/2>length(X),1))
                    location_index=randi(length(X),1,n_seg_vehi);
                    for j = 1:length(location_index)
                        if location_index(j)~=min(location_index) && location_index(j)~=max(location_index)
                            while isempty(find(location_index>=location_index(j)-R & location_index<location_index(j),1))
                                location_index=randi(length(X),1,n_seg_vehi);
                            end
                        elseif location_index(j) == min(location_index)
                            while isempty(find(location_index<=location_index(j)+R & location_index>location_index(j),1))
                                location_index=randi(length(X),1,n_seg_vehi);
                            end
                        else
                            while isempty(find(location_index>=location_index(j)-R & location_index<location_index(j),1))
                                location_index=randi(length(X),1,n_seg_vehi);
                            end
                        end
                    end
                end
            end
            
            
            X_vehi = X(location_index);
            Y_vehi = Y(location_index);                      %将第i段对应location_index列的X、Y轴图像值分别赋予X_vehi和Y_vehi
            location_vehi(num_vehi+1:num_vehi+n_seg_vehi,1) = num_vehi+1:num_vehi+n_seg_vehi;
            location_vehi(num_vehi+1:num_vehi+n_seg_vehi,2) = X_vehi;
            location_vehi(num_vehi+1:num_vehi+n_seg_vehi,3) = Y_vehi;
            num_vehi = num_vehi+n_seg_vehi;                  %车辆数，这里不仅仅代表车辆数，同时也是循环分配车辆位置算法进行到第多少辆车的标志
       
        end
    end
    %绘制分叉路段

end




