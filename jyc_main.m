function [time,PDR,location_vehi,num_vehi,maxhop]=jyc_main(R,N_iter,N_part,location_vehi,num_vehi)
%��cell_to_singleroad�����reverse_traversal������Ҳ������jyc_bar_complete_relay_selection������findroad����
%����PDR������Ĭ��ͨ�ŷ�Χ��������һ�������ڵ�ɹ�ͨ�ţ���ô���������ͨ�ŷ�Χ��û�г����ڵ㣬����ѡ������һ��P_opt����sender��֮��ĳ����ڵ���Ϊnew_relay���ǲ���PDR=0��Ҳ����ͨ��ʧ�ܣ������
%PDR=0�����ֻ���ڣ�1.���ô��������������������ӳ����޴�  or  2.�����ű�ķ�����ѡ�е�relay������ʻ��ͨ�ŷ�Χ  ����������Ż����
    global branchset A Linmap1m T ima2 locationjunction isbeacon;    
   
%     ima=imread('system_map.jpg');
%     imshow(ima);                  %��ʾͼƬ
%     hold on;
% 
%     for i = 1:length(location_vehi(:,1))
%         plot(location_vehi(i,2),location_vehi(i,3),'o','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor',[0.7,0.7,0.7],'MarkerSize',4);%����X_vehi��Y_vehi�������ڵ���ͼ�б�ʶ����
%         hold on
%     end
%     %���ƿ̶ȳ�
%     LinmapRm=R*Linmap1m;    
%     plot([20,20+LinmapRm],[80,80],'Linewidth',2);
%     text(30+LinmapRm/20,65,'200')
%     hold on
    
    PDR_set = [];%%��ʼ��PDR
    store_road = [];%��ʼ��store_road�����洢�ѱ���·�ε�junction����ţ�junction����ŷֱ����1��2����
    all_location_relay = [];%��ʼ��all_location_relay�����洢�����м̽ڵ���Ϣ����1�д��м̽ڵ�����·����ʱ��junction����ţ���2�д��м̽ڵ�����·��ȥ����junction����ţ���3��4�д��м̽ڵ������
    nowlayer = 0;%��ʼ����ǰ����Ϊ0
    last_relay = branchset{2,1}(1,2:3);
    t_relay = 0;
    roadname = '1012';
    junction_time = locationjunction;%junction_timeǰ3�е���locationjunction
    junction_time(:,4) = 0;%junction_time��4�д洢��junction��Ĵ��ʱ��
    junction_time(:,5) = 0;%junction_time��5�д洢�ڸ�junction�㴦���Ĺ㲥��ʱ·��
    %������Ϣ�������,��������ʼ��ȡ��·��Ϣ
    store_end = zeros(24,4);%�����洢�ս�junction�㴦��last_relay��Ϣ������last_relay����������˵�ʱ�䣬��1���Ǵ���ս�junction����ţ���2��3�д����last_relay�����꣬��4�д����ʱ��  
    store_wait = zeros(24,5);%�����洢�ȴ�junction�㴦��last_relay��Ϣ������last_relay����������˵�ʱ�䣬��1���Ǵ�ĵȴ�junction����ţ���2��3�д���Ǵ���Ϣ��last_relay�����꣬��4�д���Ǵ�ʱ��,��5�д���Ǵ���Ϣ��ʱ��junction�����
    tree_root = cell(2,1);
    tree_layer1 = cell(2,2);
    tree_layer2 = cell(2,4);
    tree_layer3 = cell(2,6);
    tree_layer4 = cell(2,5);
    tree_layer5 = cell(2,7);
    tree_layer6 = cell(2,4);          
    relayinf = zeros(1,5);%��1��2�д�last_relay�����꣬��3��4�д�������·�Σ���5�д����ʱ��    
    [tree,endtree,waittree] = newtree;
    
    while(1)
        for i = 1:length(nowlayer)
            if nowlayer(i)==0 && isempty(tree_root{1,1})%����ڵ�0�㣨���㣩����tree_root{1,1}Ϊ�գ�˵����Ÿս�����Ϣ�㲥
                [nextjunction,junction_time,last_relay,t_relay,store_wait,store_end,end_relay_junction,store_road,all_location_relay,~,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,last_relay,R,Linmap1m,roadname,junction_time,t_relay,store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
            elseif nowlayer(i)==0 && isempty(tree_root{1,1})==0%����ڵ�0�㣨���㣩����tree_root{1,1}��Ϊ�գ�˵������·���ѱ��㲥
                roadname = [num2str(tree_root{1,1}(1,3),'%02d'),num2str(tree_root{1,1}(1,4),'%02d')];
                [nextjunction,junction_time,last_relay,t_relay,store_wait,store_end,end_relay_junction,store_road,all_location_relay,~,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,tree_root{1,1}(1,1:2),R,Linmap1m,roadname,junction_time,tree_root{1,1}(1,5),store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
            elseif nowlayer(i)==1%�ӵ�һ�㿪ʼ���ַ�֧�ˣ���Ҫ���Ƿ�֧����������������Լ�������ͼ����֪��֧Ϊ2
                roadname = [num2str(tree_layer1{1,i}(1,3),'%02d'),num2str(tree_layer1{1,i}(1,4),'%02d')];
                [nextjunction_part,junction_time,last_relay_part,t_relay_part,store_wait,store_end,end_relay_junction_part,store_road,all_location_relay,PDR_part,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,tree_layer1{1,i}(1,1:2),R,Linmap1m,roadname,junction_time,tree_layer1{1,i}(1,5),store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
                end_relay_junction = [end_relay_junction,end_relay_junction_part];
                nextjunction = [nextjunction;nextjunction_part];
                t_relay = [t_relay,t_relay_part];
                last_relay = [last_relay;last_relay_part];
                PDR_set = [PDR_set;PDR_part];
            elseif nowlayer(i)==2
                roadname = [num2str(tree_layer2{1,i}(1,3),'%02d'),num2str(tree_layer2{1,i}(1,4),'%02d')];
                [nextjunction_part,junction_time,last_relay_part,t_relay_part,store_wait,store_end,end_relay_junction_part,store_road,all_location_relay,PDR_part,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,tree_layer2{1,i}(1,1:2),R,Linmap1m,roadname,junction_time,tree_layer2{1,i}(1,5),store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
                end_relay_junction = [end_relay_junction,end_relay_junction_part];
                nextjunction = [nextjunction;nextjunction_part];
                t_relay = [t_relay,t_relay_part];
                last_relay = [last_relay;last_relay_part];
                PDR_set = [PDR_set;PDR_part];
            elseif nowlayer(i)==3
                roadname = [num2str(tree_layer3{1,i}(1,3),'%02d'),num2str(tree_layer3{1,i}(1,4),'%02d')];
                [nextjunction_part,junction_time,last_relay_part,t_relay_part,store_wait,store_end,end_relay_junction_part,store_road,all_location_relay,PDR_part,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,tree_layer3{1,i}(1,1:2),R,Linmap1m,roadname,junction_time,tree_layer3{1,i}(1,5),store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
                end_relay_junction = [end_relay_junction,end_relay_junction_part];
                nextjunction = [nextjunction;nextjunction_part];
                t_relay = [t_relay,t_relay_part];
                last_relay = [last_relay;last_relay_part];
                PDR_set = [PDR_set;PDR_part];
            elseif nowlayer(i)==4
                roadname = [num2str(tree_layer4{1,i}(1,3),'%02d'),num2str(tree_layer4{1,i}(1,4),'%02d')];
                [nextjunction_part,junction_time,last_relay_part,t_relay_part,store_wait,store_end,end_relay_junction_part,store_road,all_location_relay,PDR_part,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,tree_layer4{1,i}(1,1:2),R,Linmap1m,roadname,junction_time,tree_layer4{1,i}(1,5),store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
                end_relay_junction = [end_relay_junction,end_relay_junction_part];
                nextjunction = [nextjunction;nextjunction_part];
                t_relay = [t_relay,t_relay_part];
                last_relay = [last_relay;last_relay_part];
                PDR_set = [PDR_set;PDR_part];
            elseif nowlayer(i)==5
                roadname = [num2str(tree_layer5{1,i}(1,3),'%02d'),num2str(tree_layer5{1,i}(1,4),'%02d')];
                [nextjunction_part,junction_time,last_relay_part,t_relay_part,store_wait,store_end,end_relay_junction_part,store_road,all_location_relay,PDR_part,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,tree_layer5{1,i}(1,1:2),R,Linmap1m,roadname,junction_time,tree_layer5{1,i}(1,5),store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
                end_relay_junction = [end_relay_junction,end_relay_junction_part];
                nextjunction = [nextjunction;nextjunction_part];
                t_relay = [t_relay,t_relay_part];
                last_relay = [last_relay;last_relay_part];
                PDR_set = [PDR_set;PDR_part];
            elseif nowlayer(i)==6
                roadname = [num2str(tree_layer6{1,i}(1,3),'%02d'),num2str(tree_layer6{1,i}(1,4),'%02d')];
                [nextjunction_part,junction_time,last_relay_part,t_relay_part,store_wait,store_end,end_relay_junction_part,store_road,all_location_relay,PDR_part,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,tree_layer6{1,i}(1,1:2),R,Linmap1m,roadname,junction_time,tree_layer6{1,i}(1,5),store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
                end_relay_junction = [end_relay_junction,end_relay_junction_part];
                nextjunction = [nextjunction;nextjunction_part];
                t_relay = [t_relay,t_relay_part];
                last_relay = [last_relay;last_relay_part];
                PDR_set = [PDR_set;PDR_part];
            end
        end
        nowlayer = [];%����Ʋ����������ոò���
        
        if any(any(nextjunction))==0 || isempty(nextjunction)%���nextjunction������Ԫ�ض�Ϊ0����nextjunctionΪ��          
            break;%����while(1)ѭ��
        end        
        
        for j = 1:length(nextjunction(:,1))
            for i = 1:length(nextjunction(j,:))
                if nextjunction(j,i)==0
                    break;
                end
                [locationmark] = findroad(nextjunction(j,i),end_relay_junction(j),branchset,last_relay(j,1:2),locationjunction);
                if isbeacon == 1
                    [t,location_relay,relay,location_vehi,num_vehi,PDR_part]=jyc_beacon_relay_selection(location_vehi(1:num_vehi,1:4),locationmark,R,num_vehi,N_iter,N_part,A,Linmap1m,T,ima2,t_relay(j));
                elseif N_part == 4
                    [t,location_relay,relay,PDR_part,location_vehi,num_vehi]=jyc_bar_complete_relay_selection(location_vehi(1:num_vehi,1:4),locationmark,R,num_vehi,N_iter,N_part,A,Linmap1m,T,ima2,t_relay(j));%�����ϰ�����м̽ڵ�ѡ��
                elseif N_part == 3
                    [t,location_relay,relay,PDR_part,location_vehi,num_vehi]=jyc_bar_3P3B_based_relay_selection(location_vehi(1:num_vehi,1:4),locationmark,R,num_vehi,N_iter,N_part,A,Linmap1m,T,ima2,t_relay(j));%�����ϰ�����м̽ڵ�ѡ��
                end
                %���½�location_relay�е��м̽ڵ���Ϣ����·������junction��ţ�������λ�ô��location_relay������                
                PDR_set = [PDR_set;PDR_part];
                location_relay(:,3:4)=location_relay(:,2:3);
                location_relay(:,1) = end_relay_junction(j);
                location_relay(:,2) = nextjunction(j,i);
                location_relay(location_relay(:,3)==0,:)=[];
                %���ϵ���location_relay�������
                all_location_relay = [all_location_relay;location_relay];%��location_relay�������ݸ��¼ӽ�all_location_relay������
                relayinf(1,1:2) = relay;
                relayinf(1,3) = end_relay_junction(j);
                relayinf(1,4) = nextjunction(j,i);
                relayinf(1,5) = t;
                
                if PDR_part==0%�����Ϣ����ʧ�ܣ����ü���������9.16
                    break;
                end
                
                if end_relay_junction(j)==10 && nextjunction(j,i)==12
                    tree_root{1,1} = relayinf;
                    nowlayer = [nowlayer 0];%��ʾ�����ĸ���
                elseif end_relay_junction(j)==12 && nextjunction(j,i)==15
                    tree_layer1{1,1} = relayinf;
                    nowlayer = [nowlayer 1];%��ʾ�����ĵ�һ��
                elseif end_relay_junction(j)==12 && nextjunction(j,i)==13
                    tree_layer1{1,2} = relayinf;
                    nowlayer = [nowlayer 1];%��ʾ�����ĵ�һ��
                elseif end_relay_junction(j)==15 && nextjunction(j,i)==2
                    tree_layer2{1,1} = relayinf;
                    nowlayer = [nowlayer 2];%��ʾ�����ĵڶ���
                elseif end_relay_junction(j)==15 && nextjunction(j,i)==16
                    tree_layer2{1,2} = relayinf;
                    nowlayer = [nowlayer 2];%��ʾ�����ĵڶ���
                elseif end_relay_junction(j)==13 && nextjunction(j,i)==14
                    tree_layer2{1,3} = relayinf;
                    nowlayer = [nowlayer 2];%��ʾ�����ĵڶ���
                elseif end_relay_junction(j)==13 && nextjunction(j,i)==17
                    tree_layer2{1,4} = relayinf;
                    nowlayer = [nowlayer 2];%��ʾ�����ĵڶ���
                elseif end_relay_junction(j)==16 && nextjunction(j,i)==1
                    tree_layer3{1,1} = relayinf;
                    nowlayer = [nowlayer 3];%��ʾ�����ĵ�����
                elseif end_relay_junction(j)==16 && nextjunction(j,i)==11
                    tree_layer3{1,2} = relayinf;
                    nowlayer = [nowlayer 3];%��ʾ�����ĵ�����
                elseif end_relay_junction(j)==14 && nextjunction(j,i)==3
                    tree_layer3{1,3} = relayinf;
                    nowlayer = [nowlayer 3];%��ʾ�����ĵ�����
                elseif end_relay_junction(j)==14 && nextjunction(j,i)==20
                    tree_layer3{1,4} = relayinf;
                    nowlayer = [nowlayer 3];%��ʾ�����ĵ�����
                elseif end_relay_junction(j)==17 && nextjunction(j,i)==9
                    tree_layer3{1,5} = relayinf;
                    nowlayer = [nowlayer 3];%��ʾ�����ĵ�����
                elseif end_relay_junction(j)==17 && nextjunction(j,i)==18
                    tree_layer3{1,6} = relayinf;
                    nowlayer = [nowlayer 3];%��ʾ�����ĵ�����
                elseif end_relay_junction(j)==20 && nextjunction(j,i)==21
                    tree_layer4{1,1} = relayinf;
                    nowlayer = [nowlayer 4];%��ʾ�����ĵ��Ĳ�
                elseif end_relay_junction(j)==20 && nextjunction(j,i)==18
                    tree_layer4{1,2} = relayinf;
                    nowlayer = [nowlayer 4];%��ʾ�����ĵ��Ĳ�
                elseif end_relay_junction(j)==18 && nextjunction(j,i)==19
                    tree_layer4{1,3} = relayinf;
                    nowlayer = [nowlayer 4];%��ʾ�����ĵ��Ĳ�
                elseif end_relay_junction(j)==18 && nextjunction(j,i)==20
                    tree_layer4{1,4} = relayinf;
                    nowlayer = [nowlayer 4];%��ʾ�����ĵ��Ĳ�
                elseif end_relay_junction(j)==18 && nextjunction(j,i)==22
                    tree_layer4{1,5} = relayinf;
                    nowlayer = [nowlayer 4];%��ʾ�����ĵ��Ĳ�
                elseif end_relay_junction(j)==21 && nextjunction(j,i)==4
                    tree_layer5{1,1} = relayinf;
                    nowlayer = [nowlayer 5];%��ʾ�����ĵ����
                elseif end_relay_junction(j)==21 && nextjunction(j,i)==23
                    tree_layer5{1,2} = relayinf;
                    nowlayer = [nowlayer 5];%��ʾ�����ĵ����
                elseif end_relay_junction(j)==19 && nextjunction(j,i)==6
                    tree_layer5{1,3} = relayinf;
                    nowlayer = [nowlayer 5];%��ʾ�����ĵ����
                elseif end_relay_junction(j)==19 && nextjunction(j,i)==23
                    tree_layer5{1,4} = relayinf;
                    nowlayer = [nowlayer 5];%��ʾ�����ĵ����
                elseif end_relay_junction(j)==19 && nextjunction(j,i)==24
                    tree_layer5{1,5} = relayinf;
                    nowlayer = [nowlayer 5];%��ʾ�����ĵ����
                elseif end_relay_junction(j)==22 && nextjunction(j,i)==8
                    tree_layer5{1,6} = relayinf;
                    nowlayer = [nowlayer 5];%��ʾ�����ĵ����
                elseif end_relay_junction(j)==22 && nextjunction(j,i)==24
                    tree_layer5{1,7} = relayinf;
                    nowlayer = [nowlayer 5];%��ʾ�����ĵ����
                elseif end_relay_junction(j)==23 && nextjunction(j,i)==5
                    tree_layer6{1,1} = relayinf;
                    nowlayer = [nowlayer 6];%��ʾ�����ĵ�����
                elseif end_relay_junction(j)==23 && nextjunction(j,i)==19
                    tree_layer6{1,2} = relayinf;
                    nowlayer = [nowlayer 6];%��ʾ�����ĵ�����
                elseif end_relay_junction(j)==24 && nextjunction(j,i)==7
                    tree_layer6{1,3} = relayinf;
                    nowlayer = [nowlayer 6];%��ʾ�����ĵ�����
                elseif end_relay_junction(j)==24 && nextjunction(j,i)==22
                    tree_layer6{1,4} = relayinf;
                    nowlayer = [nowlayer 6];%��ʾ�����ĵ�����
                end               
            end   
            
            if PDR_part==0%�����Ϣ����ʧ�ܣ����ü���������9.16
                break;
            end
            
        end       
        end_relay_junction = [];
        nextjunction = [];
        t_relay = [];
        last_relay = [];     
        
        if PDR_part==0%�����Ϣ����ʧ�ܣ����ü���������9.16
            break;
        end
   
    end

    for j = 1:length(all_location_relay(:,1))%forѭ���еĴ���Ŀ���ǣ�����Ϊ�෴�������Ϣ����������ҳ�������ͳһΪһ������        
        %˼�룺��һ������all_location_relay���������ҳ����ڶ��е���all_location_relay(j,1)���ҵ�һ�е���all_location_relay(j,2)���У�
        %�ڶ�����������Juntion_time�еĴ��ʱ������ж����϶���Ϣ���������ǴӴ����ĵ�㲥�������ĵ㣬�ʸ���������Ĵ��ʱ�䣬���ս���Ϊ�෴��������Ϣ��������ͳһΪһ����Ϣ��������
        [index_a,~] = find(all_location_relay(:,2)==all_location_relay(j,1));%�ҳ�all_location_relay�еڶ��е���all_location_relay(j,1)�����±�
        [index_b,~] = find(all_location_relay(:,1)==all_location_relay(j,2));%�ҳ�all_location_relay�е�һ�е���all_location_relay(j,2)�����±�
        index = intersect(index_a,index_b);
        if isempty(index)==0 && any(all_location_relay(index(1),:))~=0%���index��Ϊ��,��index��Ӧ�����治ȫΪ0
            index_a_used_in_amend = [];
            index_b_used_in_amend = [];
            index_used_in_amend = [];
            if all_location_relay(j,1)~=0
                if junction_time(all_location_relay(j,1),4) > junction_time(all_location_relay(j,2),4)%�����all_location_relay(j,1)��junction��Ĵ��ʱ�����ڵ�all_location_relay(j,2)��junction�Ĵ��ʱ��
                    [index_a_used_in_amend,~] = find(all_location_relay(:,1)==all_location_relay(j,1));
                    [index_b_used_in_amend,~] = find(all_location_relay(:,2)==all_location_relay(j,2));
                    index_used_in_amend = intersect(index_a_used_in_amend,index_b_used_in_amend);
                    all_location_relay(index_used_in_amend,:)=0;
                else                                                                                  %�����all_location_relay(j,1)��junction��Ĵ��ʱ�����ڵ�all_location_relay(j,2)��junction�Ĵ��ʱ��
                    [index_a_used_in_amend,~] = find(all_location_relay(:,1)==all_location_relay(j,2));
                    [index_b_used_in_amend,~] = find(all_location_relay(:,2)==all_location_relay(j,1));
                    index_used_in_amend = intersect(index_a_used_in_amend,index_b_used_in_amend);
                    all_location_relay(index_used_in_amend,:)=0;
                end
            end
        end
    end
    all_location_relay(all_location_relay(:,1)==0,:)=[];
    %        ͨ�������forѭ����ͼ�л���relay��
%     for j =1:length(all_location_relay(:,1))%������ѡ�����м̽ڵ���ͼ�л���
%         plot(all_location_relay(j,3),all_location_relay(j,4),'^','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);
%         hold on
%     end
    PDR = mean(PDR_set(:,1));%����PDR
    if PDR~=1%���Ƕ˵��˵�PDR
        PDR = 0;
        time = NaN;%�˵�����Ϣ����ʧ�ܣ���ʱ����������
        maxhop = NaN;
    else%���PDR����1��˵���˵��˴���ɹ��ˣ������else�������maxhop��time
        %���¿�ʼ��������
        hop = 0;
        temp_all_relay = all_location_relay;
        temp_all_relay(:,5) = 0;
        while(1)
            while(1)
                [sameroad_relay_index_a,~] = find(temp_all_relay(:,1)==temp_all_relay(1,1));
                [sameroad_relay_index_b,~] = find(temp_all_relay(:,2)==temp_all_relay(1,2));
                sameroad_relay_index = intersect(sameroad_relay_index_a,sameroad_relay_index_b);
                for n = 1:length(sameroad_relay_index)
                    if hop > 0
                    %�ڸ�if����У����ǽ������������������������
                    %һ����ĳ·����û�н����м̽ڵ�ѡ�������˸�·�εĸ���ʱ�������ں�����ͨ���Ὣ��һ·�����ѡ����м̽ڵ���Ϊ��·�ε��м̽ڵ㣬Ȼ����ʵ�и�·����ʵ����û�����м̽ڵ�ѡ�񣨴����������21-04·�Σ�
                    %������ĳjunction�㴦������Ϣ�㲥��ͻ����ʱ�Ӹ�junction��������������Ϣ�㲥�����ظ���ѡ��һ�������junction��ĳ�����Ϊ�м̽ڵ㣬Ȼ����ʵ�ظ�����һ������ʵ�в�û�б�Ҫ���������������21��junction�㣩
                    %���������������ʵ���ᵼ������������һ����������if����У������ж���������������������Ǿͽ�������һ����Ӧ�Դ������⡣
                        [index_last_hop,~] = find(temp_all_relay(:,5) == hop);
                        if isempty(index_last_hop)==0
                            for i = 1:length(index_last_hop)
                                if index_last_hop(i) ~= 1 && temp_all_relay(index_last_hop(i),3) == temp_all_relay(1,3) && temp_all_relay(index_last_hop(i),4) == temp_all_relay(1,4)
                                    hop = hop-1;
                                    break;
                                end
                            end
                        end
                    end
                    hop = hop+1;%��������
                    if temp_all_relay(sameroad_relay_index(n),5)<hop
                        temp_all_relay(sameroad_relay_index(n),5)=hop;
                    end
                end                
                [index_a_used_in_amend,~] = find(temp_all_relay(:,1) == temp_all_relay(1,2),1,'first');
                [index_b_used_in_amend,~] = find(temp_all_relay(:,2) == temp_all_relay(1,1));
                index = setdiff(index_a_used_in_amend,index_b_used_in_amend);
                if isempty(index) || junction_time(temp_all_relay(1,2),5) ~= temp_all_relay(1,1)%���������·�ϵ�relay���Ѿ����ǵ��յ���߸��ǵ��Ѿ������ǹ���junction�㣬��ô�����������¼�������
                    break;
                end
                if temp_all_relay(index,5)<=hop
                    temp_one_relay = temp_all_relay(1,:);
                    temp_all_relay(1,:) = temp_all_relay(index,:);
                    temp_all_relay(index,:) = temp_one_relay;
                else
                    break;
                end
            end
            count = 1;%��ʼ�����������������������while���м������������ѭ��
            while(1)               
                [index,~] = find(temp_all_relay(:,5) == 0,1,'first');%�ҳ���Ӧ��δ����������·��
                if isempty(index)%����Ҳ�����˵�������ж�����������
                    break;
                end
                [pro_index,~] = find(temp_all_relay(:,2)==temp_all_relay(index,1));%�ҳ���Ӧ·�ε�ǰ��·��
                
                if isempty(pro_index)%����Ҳ�����Ӧ·�ε�ǰ��·�Σ���˵��������Ϣ�㲥�Ǵ���ģ���¼һ��
                    PDR = 0;
                    time = NaN;%�˵�����Ϣ����ʧ�ܣ���ʱ����������
                    maxhop = NaN;
                    look_sign = 1;
                end
                
                if temp_all_relay(pro_index(1),5)==0%���ǰ��·��Ҳδ����������
                    temp_one_relay = temp_all_relay(index,:);%������Ӧ·�κ���ǰ��·���������е�λ��
                    temp_all_relay(index,:) = temp_all_relay(pro_index(1),:);
                    temp_all_relay(pro_index(1),:) = temp_one_relay;
                else
                    hop = max(temp_all_relay(pro_index,5));%��������Ϊ��Ӧ·��ǰ��·�εĴ�������
                    temp_one_relay = temp_all_relay(1,:);%������Ӧ·���������е�һ��·�ε�λ��
                    temp_all_relay(1,:) = temp_all_relay(index,:);
                    temp_all_relay(index,:) = temp_one_relay;
                    break;
                end
                
                count = count+1;
                if count>50%���count������50����ζ�ų�������ѭ����������Ҫ��temp_all_relay�����д洢������յ�λ�ý����޸�                   
                    for k = 1:length(temp_all_relay(:,1))
                        if temp_all_relay(k,5)==0
                            if temp_all_relay(k,2)==junction_time(temp_all_relay(k,1),5)%���temp_all_relay������k�д洢��junctionȥ�����յ����junction_time�����и�junction�洢����ʱ�ĵ㣬��˵�������㹹�ɵ�·�γ�����ѭ���������������junction_time�����д洢��Ϊ׼
                                %�ڴ�if����е���temp_junction�����и��������յ��λ��
                                temp_junction = temp_all_relay(k,1);
                                temp_all_relay(k,1) = temp_all_relay(k,2);
                                temp_all_relay(k,2) = temp_junction;
                            end
                        end
                    end
                    count = 1;%��ԭcount�ļ���
                end
                
            end
            if isempty(find(temp_all_relay(:,5)==0, 1))
                break;
            end
        end
        maxhop = max(temp_all_relay(:,5));
        %�����������
        time=max(junction_time(:,4));%����˵���ʱ��
    end    
        
end