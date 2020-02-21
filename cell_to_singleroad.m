function [nextjunction,junction_time,last_relay,t_relay,store_wait,store_end,end_relay_junction,store_road,all_location_relay,PDR,location_vehi,num_vehi] = cell_to_singleroad(tree,endtree,waittree,locationjunction,last_relay,R,Linmap1m,roadname,junction_time,t_relay,store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part)
%�˺��������ǰ�Ԫ�������еĸ���·����Ϣ��������ȡ����������֮����м̽ڵ�ѡ���������м̽ڵ�ѡ��  
    global isbeacon;
    end_relay_junction = [];
    start_relay_junction = [];%��ʼ������
    roadname1 = str2num(roadname(1:2));%roadname���ַ�����ʽ��·��������������Ϊ��1012�����ַ�����
    roadname2 = str2num(roadname(3:4));%����roadname1��roadname2Ŀ���ǰ�·�����ַ�����ɹ���·�ε�����junction�����ţ��ֱ����roadname1��roadname2��
    PDR = [];
    if isempty(all_location_relay)==0
        for i = 1:length(all_location_relay(:,1))
            if all_location_relay(i,3) == last_relay(1,1) && all_location_relay(i,4) == last_relay(1,2) && (roadname1==all_location_relay(i,1) || roadname1==all_location_relay(i,2)) && (roadname2==all_location_relay(i,1) || roadname2==all_location_relay(i,2))
                start_relay_junction = all_location_relay(i,1);
                end_relay_junction = all_location_relay(i,2);
            end
        end
    else
        dis1 = (sum((locationjunction(roadname1,2:3)-last_relay).^2))^0.5/Linmap1m;%���roadname1��last_relay�ľ���
        dis2 = (sum((locationjunction(roadname2,2:3)-last_relay).^2))^0.5/Linmap1m;%���roadname2��last_relay�ľ���
        if  dis1 < dis2  %���roadname1��last_relay�ľ���С��roadname2��last_relay�ľ��룬��˵��roadname1��last_relay��������last_relay����·�ε�end_relay_junction��·���յ�junction��
            end_relay_junction = roadname1;%end_relay_junction�洢last_relay�����ǵ�junction����ţ����Ǹ�·���յ�junction���
            start_relay_junction = roadname2;%start_relay_junction�洢last_relay����·�ε���ʼjunction���
        else
            end_relay_junction = roadname2;
            start_relay_junction = roadname1;
        end
    end
    
    if isempty(end_relay_junction)
        dis1 = (sum((locationjunction(roadname1,2:3)-last_relay).^2))^0.5/Linmap1m;%���roadname1��last_relay�ľ���
        dis2 = (sum((locationjunction(roadname2,2:3)-last_relay).^2))^0.5/Linmap1m;%���roadname2��last_relay�ľ���
        if  dis1 < dis2  %���roadname1��last_relay�ľ���С��roadname2��last_relay�ľ��룬��˵��roadname1��last_relay��������last_relay����·�ε�end_relay_junction��·���յ�junction��
            end_relay_junction = roadname1;%end_relay_junction�洢last_relay�����ǵ�junction����ţ����Ǹ�·���յ�junction���
            start_relay_junction = roadname2;%start_relay_junction�洢last_relay����·�ε���ʼjunction���
        else
            end_relay_junction = roadname2;
            start_relay_junction = roadname1;
        end
    end
    
    store_road = [store_road;[roadname1 roadname2]];  
    if junction_time(end_relay_junction,4)==0 %�����ǰlast_relay�����ǵ�junctionδ�洢��ʱ��
        junction_time(end_relay_junction,4) = t_relay;%����ʱ��Ϊ��ǰʱ��
        junction_time(end_relay_junction,5) = start_relay_junction;%����ʱ��Ϊ��ǰrelay����ʱ·��
        if isempty(find(end_relay_junction == endtree(:), 1))==0%��Ϊ1����Ϊ0��isempty(...)==0��ʾ��Ϊ�գ�������ʾ�����ǰlast_relay�����ǵ�junction���ս�junction
            if end_relay_junction == 10 && junction_time(12,4) == 0 %���12��junction���ʱ����0��Ȼ��last_relay���ǵ�junction���Ϊ10�ţ�˵�����Ǹտ�ʼ��Ϣ�㲥
                store_end(end_relay_junction,1) = end_relay_junction;
                nextjunction=12;%nextjunction�����洢�뱻����junction����������junction�����ţ������ʼ���洢����12��junction��
            else
                store_end(end_relay_junction,1) = end_relay_junction;
                store_end(end_relay_junction,2:3) = last_relay;
                store_end(end_relay_junction,4) = t_relay;
                nextjunction = 0;%nextjunction��0����ʾ����Ϣ֧·�ս�
            end
        elseif isempty(find(end_relay_junction == waittree(:), 1))==0%��Ϊ1����Ϊ0��isempty(...)==0��ʾ��Ϊ�գ�������ʾ�����ǰlast_relay�����ǵ�junction�ǵȴ�junction
            %��һ�θ��ǵȴ�junction������������������ȥ
            store_wait(end_relay_junction,1) = end_relay_junction;
            store_wait(end_relay_junction,2:3) = last_relay;
            store_wait(end_relay_junction,4) = t_relay;
            store_wait(end_relay_junction,5) = start_relay_junction;
            nextjunction = tree(end_relay_junction,2:5);%��tree���뵱ǰ����junction������junction��ȡ��������nextjunction������
            nextjunction(:,nextjunction(1,:)==start_relay_junction)=[];%��nextjunction�а�����·����ʼjunction���ɾ������Ϊ�����Ǵ�����·���ģ�û��Ҫ����ȥ
        else    %������ʾ�����ǰlast_relay�����ǵ�junction����ͨjunction
            nextjunction = tree(end_relay_junction,2:5);%��tree���뵱ǰ����junction������junction��ȡ��������nextjunction������
            nextjunction(:,nextjunction(1,:)==start_relay_junction)=[];%��nextjunction�а�����·����ʼjunction���ɾ������Ϊ�����Ǵ�����·���ģ�û��Ҫ����ȥ            
        end
        

%����Ĭ��ÿ���ȴ�junction��waittree(i)������ڸ��ǹ����г����˸���ʱ���last_relay�ĸ��£���ôֻ���ǣ��ڶ��θ���ʱ��С�ڵ�һ�θ���ʱ�䣬�Ӷ������ĸ��¡�
%���ǻ��������ǵ����θ��ǻ���Ĵθ��Ƕ�ǰ�渲��ʱ��ĸ��£���Ϊ���ֿ�����̫С��
    elseif junction_time(end_relay_junction,4)~=0%�����ǰlast_relay�����ǵ�junction���д���ʱ��
        if junction_time(end_relay_junction,4)>t_relay%���junction�����ʱ�����t_relay�������ڵ�ǰlast_relay�������˵�ʱ�䣬˵����ǰlast_relay����junction��ʱ��ǰ���ģ��ʸ��´�ʱ�䣬���Դ�last_relay��ϢΪ��������֧·��Ϣ����            
            junction_time(end_relay_junction,4) = t_relay;%����junction_time��Ӧλ�ô洢��ʱ��     
            junction_time(end_relay_junction,5) = start_relay_junction;%����ʱ��Ϊ��ǰrelay����ʱ·��
            if isempty(find(end_relay_junction == endtree(:), 1))==0%������ǵ����ս�junction����ô����ʱ���last_relay�����ս����Ϣ֧·
                store_end(end_relay_junction,1) = end_relay_junction;
                store_end(end_relay_junction,2:3) = last_relay;%������Ϣ
                store_end(end_relay_junction,4) = t_relay;
                nextjunction = 0;%�����ʾֱ���ս���
            elseif isempty(find(end_relay_junction == waittree(:), 1))==0%������ǵ��ǵȴ�junction����ô����ʱ���last_relay�����Դ�last_relay��ϢΪ��������֧·��Ϣ����
                store_wait(end_relay_junction,1) = end_relay_junction;%������Ϣ
                store_wait(end_relay_junction,2:3) = last_relay;%������Ϣ
                store_wait(end_relay_junction,4) = t_relay;
                store_wait(end_relay_junction,5) = start_relay_junction;
                %ֱ�ӽ��뷴���������
                [all_location_relay,junction_time,store_road,PDR,location_vehi,num_vehi]...
                    = reverse_traversal(store_road,end_relay_junction,start_relay_junction,tree,all_location_relay,location_vehi,num_vehi,locationjunction,last_relay,R,t_relay,junction_time,endtree,N_iter,N_part);               
                nextjunction = 0; %�������֮��ֱ�Ӹ�nextjunction���㣬��ʾ����ֱ���ս��ˣ���Ϊ�����ڷ�������������Ѿ������˲���
            else  %������ʾ�����ǰlast_relay�����ǵ�junction����ͨjunction
                nextjunction = tree(end_relay_junction,2:5);%��tree���뵱ǰ����junction������junction��ȡ��������nextjunction������
                nextjunction(:,nextjunction(1,:)==start_relay_junction)=[];%��nextjunction�а�����·����ʼjunction���ɾ������Ϊ�����Ǵ�����·���ģ�û��Ҫ����ȥ
            end
        else%���junction�����ʱ��С��t_relay��˵��������Ϣ֧·��
            %��û��Ҫ���������ˣ���Ϊ�Ѿ�����
            nextjunction = 0;%�����ʾֱ���ս���,��Ϊ�����ˣ�û��Ҫ��˳�Ŵ�֧·��������
        end
    end
    nextjunction(:,nextjunction(1,:)==0)=[];%��nextjunction�е���Ԫ��ȥ��
    if isempty(nextjunction)%���nextjunctionΪ�գ����������ս�ڵ㡢�ȴ��ڵ���ߴ��ʱ��������ʱ��ĺ��棬��ô������ֹ�˷����ϵ���Ϣ����
        t_relay=[];
        end_relay_junction=[];
        last_relay = [];
    elseif length(nextjunction)<3%����nextjunction��Ϊ�գ����Ǿͽ�nextjunction����Ԫ�ص�3�У�������������nextjunction���鼯�ϵĽṹ����
        for j = 1:3-length(nextjunction)
            nextjunction = [nextjunction,0];
        end
    end
    %����nextjunction��Ϣ����������ͬ��ѭ������������ѭ������ͬһ�����junction�㣬Ȼ���ٿ�ʼ��һ���junction�������Դ����ƣ�
    %����������Ҫ�ü����������洢������Ϣ����������ʵ�ֱ���ÿ����Ϣ��֧��ֹ�����Ϣ��������������󣬿�������һ���ÿ����Ӧ��Ϣ��֧����ʼ����Ϣʹ�á�
    
    


