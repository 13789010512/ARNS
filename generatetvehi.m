function [location_vehi,num_vehi]=generatetvehi(density,locationjucntion,branchset,R,Linmap1m,locationbar)
%%%���ɵ�·������Ϣ������ʾ����·������ʮ��·��
%�ڷ���ͼ�У���·��Ϊ���������ĳ��������������֣����������ǳ����������ݵ����������ڵ��ܶ�*���������ó��೵���ĳ����ڵ��ܶȣ�Ȼ�����ݸ��ܶ����ɳ����ڵ�


%��ʼ��һ��ԭʼ��Ϣ���ͳ�����λ��10-12·�ε�10λ��
location_mark = branchset{2,1};
num_vehi=0;
location_vehi = zeros(1000,4);%1:��ţ�2��x���ꣻ3��y���ꣻ4��seg����
location_vehi(1,1) = 1;
location_vehi(1,2:3) = location_mark(1,2:3);
num_vehi = num_vehi+1;
%���ϳ�ʼ��ԭʼ��Ϣ���ͳ������
for r = 1:length(branchset(1,:))
    location_mark = branchset{2,r};
    roadname = branchset{1,r}(1:5);
    if strcmp(roadname,'10-12') || strcmp(roadname,'12-13') || strcmp(roadname,'13-14') || strcmp(roadname,'14-03')
        roadnum = 6;%������Ϊ������
    elseif strcmp(roadname,'12-15') || strcmp(roadname,'15-02') || strcmp(roadname,'13-17') || strcmp(roadname,'17-18') || strcmp(roadname,'18-19') || strcmp(roadname,'19-06')
        roadnum = 4;%������Ϊ�ĳ���
    else
        roadnum = 2;%������Ϊ������
    end
    l_road = sum((location_mark(2:length(location_mark(:,1)),2:3)-location_mark(1:length(location_mark(:,1))-1,2:3)).^2,2).^0.5/Linmap1m;%������������·�ϵ���markȷ���ĵ�i��·�ε���ʵ����
    l_road_all = sum(l_road);%·�ε���ʵ�ܳ���
    
    %���ƹ���·��
    for i=1:length(location_mark(:,1))-1
        n_seg_vehi = random('Poisson',density*l_road(i)*roadnum,1,1); %���������������·�ϵ���markȷ���ĵ�i��·���ϵĳ�����
        A = location_mark(i,2:3);
        B = location_mark(i+1,2:3);
        
        %���ܵ�����κ�������������,�ܴ�������������⣨�ѽ����
        %˼·�������龰�£���l_road·�ε��˵�4�ε�ʱ��������ʵ·��Ϊ284�����ң������ڵ��µ�deltaX��deltaXΪÿ�׶�Ӧ��ͼ������ֵ�������£���4�εĳ��Ⱦ�Ȼ����8000���ף���X�����������8000���У������ѽ����
        if B(1)~= A(1)&& B(2)~= A(2)
            deltaX = (B(1)-A(1))/l_road(i);%�Ե�·����1��Ϊ���ȷ������ͼ�����ص�λ��,�����ÿ��·��ÿ�׵�X��Y����ı仯
            X = A(1): deltaX:B(1);
            Y = (B(2)-A(2))/(B(1)-A(1))*(X-A(1))+A(2);
        else
            if B(1)== A(1)
                deltaX = (B(2)-A(2))/l_road(i);
                Y = A(2):deltaX:B(2);
                X = A(1)*ones(1,length(Y));
            else
                deltaX = (B(1)-A(1))/l_road(i);%deltaXֵ��Ӧ��ʵ�е�ÿ��
                X = A(1):deltaX:B(1);
                Y = A(2)*ones(1,length(X));
            end
        end
        
        
        if n_seg_vehi ~= 0
            if isempty(X) %���X�ǿյģ����ִ������a=1�Ĳ���
                a=1;
            end
            location_index=randi(length(X),1,n_seg_vehi);    %����ĿΪn_seg_vehi�ĳ�������ķ����i�ε�ĳһ�У��ף��У������±����location_index��
           
            %���������ɵĳ���֮����С��4�ף���ô�����·���һ�Σ���������4�׼������Ҫ����Ϊ�����˳�����ռ�ռ䣬�ͳ����䰲ȫ����
            [rank_ar,~] = sort(location_index);%rank_ar�Ƕ�location_index�е������������飬�������ǽ���֮��ļ���ж�
            if length(rank_ar)>1%���·�γ�������һ̨����ô��Ҫ����4�׼���ж�
                if abs(rank_ar(2:length(rank_ar))-rank_ar(1:length(rank_ar)-1))<=4%�����������С��4��
                    location_index=randi(length(X),1,n_seg_vehi);%���·��䳵���ֲ�
                end
            end
            %����if������4�׼������
            
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
            Y_vehi = Y(location_index);                      %����i�ζ�Ӧlocation_index�е�X��Y��ͼ��ֵ�ֱ���X_vehi��Y_vehi
            location_vehi(num_vehi+1:num_vehi+n_seg_vehi,1) = num_vehi+1:num_vehi+n_seg_vehi;
            location_vehi(num_vehi+1:num_vehi+n_seg_vehi,2) = X_vehi;
            location_vehi(num_vehi+1:num_vehi+n_seg_vehi,3) = Y_vehi;
            num_vehi = num_vehi+n_seg_vehi;                  %�����������ﲻ��������������ͬʱҲ��ѭ�����䳵��λ���㷨���е��ڶ��������ı�־
       
        end
    end
    %���Ʒֲ�·��

end




