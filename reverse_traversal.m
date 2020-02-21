function [all_location_relay,junction_time,store_road,PDR,location_vehi,num_vehi] = reverse_traversal(store_road,end_relay_junction,start_relay_junction,tree,all_location_relay,location_vehi,num_vehi,locationjunction,last_relay,R,t_relay,junction_time,endtree,N_iter,N_part)
%���ǳ������ؽڵ�ʱ�������һ������ķ�������������������������Ϊ���ݲ�����ǰ������
    global branchset A Linmap1m T ima2 isbeacon
    %�������ҳ��ؽڵ�ʱ�����֮ǰ�ĸ��׽ڵ㣬����nextjunction�����У���Ϊ���ݷ�֧���л��ݲ���,nextjunction�����Junction����������Ϣ�㲥��Ŀ�꣬��last_relay����������Ϣ�㲥�����
    %store_road��ֻ���»��ݲ�����·��
    nextjunction=[];%��ʼ��nextjunction
    PDR_set = [];
    while(1)
        temp = [];%����һ��temp����Ŀ����Ϊ����ʱ���ÿ��junction��ʼ���������֧�ӽڵ�·��ѡ����relay��Ϣ��tempͨ��relayinf���������и��£�relayinf��1��2�д�relay���꣬��3��4�д�·������յ�Junction����5�д��ʱ��
        for j = 1:length(end_relay_junction)
            nextjunction_part = tree(end_relay_junction(j),2:5);
            nextjunction_part(:,nextjunction_part(1,:)==start_relay_junction(j))=[];%�����nextjunction�а�����start_relay_junction���¸��׽ڵ㣩
            nextjunction = [nextjunction;nextjunction_part];
            for n = 1:length(all_location_relay(:,1))%ͨ���˴����ҳ��ϸ���junction��
                if all_location_relay(n,2) == end_relay_junction(j)%�ҳ�ԭ��Ŀ��Junction�����������junction����
                    old_father = all_location_relay(n,1);%���е�һ�д洢��junction�������ǵ��ϸ���junction���������old_father��
                    break;
                end
            end
        end%����end_relay_junction��ͬʱҪ����start_relay_junction
        
        for j = 1:length(nextjunction(:,1))
            for i = 1:length(nextjunction(j,:))
                if nextjunction(j,i)==0%�����nextjunction�ĵ�j�е�i��������0Ԫ�أ�˵��nextjunction�ĵ�j������Ԫ�ض�����ɺ���Ĳ�������������nextjunction��j��Ԫ�ص�ѭ����
                    break;
                end
                [locationmark] = findroad (nextjunction(j,i),end_relay_junction(j),branchset,last_relay(j,1:2),locationjunction);
                if isbeacon == 1                    
                    [t,location_relay,relay,location_vehi,num_vehi,PDR_part]=jyc_beacon_relay_selection(location_vehi(1:num_vehi,1:4),locationmark,R,num_vehi,N_iter,N_part,A,Linmap1m,T,ima2,t_relay(j));
                else
                    [t,location_relay,relay,PDR_part,location_vehi,num_vehi]=jyc_bar_complete_relay_selection(location_vehi(1:num_vehi,1:4),locationmark,R,num_vehi,N_iter,N_part,A,Linmap1m,T,ima2,t_relay(j));%�����ϰ�����м̽ڵ�ѡ��
                end
                location_relay(:,3:4)=location_relay(:,2:3);%��3��4�д�relay������
                location_relay(:,1) = end_relay_junction(j);%��1�д�·�����junction
                location_relay(:,2) = nextjunction(j,i);%��2�д�·���յ�junction
                location_relay(location_relay(:,3)==0,:)=[];%�����location_relay�е�ȫ����
                PDR_set = [PDR_set;PDR_part];
                
                %���ϵ���location_relay�������               

                if junction_time(nextjunction(j,i),4)~=0 && isempty(find(nextjunction(j,i) == endtree(:), 1)) && nextjunction(j,i)~=old_father%�����ǰ������
                    %������η����·���յ���junction_time�еĴ��ʱ�䲻Ϊ0���Ҳ�������ֹ�ڵ㣬�Ҳ����ϸ��׽ڵ�
                    junction_time(nextjunction(j,i),4) = t;%����junction_time�еĴ��ʱ��
                    for n = 1:length(all_location_relay(:,1))
                        if all_location_relay(n,1)==end_relay_junction(j) && all_location_relay(n,2)==nextjunction(j,i)%��all_location_relay�������ҳ�Ŀ�ĵ������ǵ�Ŀ�ģ��յ�������ǵ��յ����
                            all_location_relay(n,:)=0; %�ҵ��󣬽�����ȫ������
                        end
                    end
                    all_location_relay(all_location_relay(:,1)==0,:)=[];%�����all_location_relay�е�ȫ����
                    all_location_relay = [all_location_relay;location_relay];%��location_relay�������ݸ��¼ӽ�all_location_relay������                    
                elseif nextjunction(j,i)==old_father %����ǻ��ݲ���
                    %ֱ�Ӿ͸���all_location_relay�����д洢��relay�ڵ���Ϣ                 
                    for n = 1:length(all_location_relay(:,1))
                        if all_location_relay(n,2) == end_relay_junction(j) && nextjunction(j,i)==all_location_relay(n,1)%�ҳ�ԭ��Ŀ��Junction�����������junction�ģ�������Ŀ��junction����ԭ�����junction����
                            all_location_relay(n,:)=0;%��ԭ��·���е�relay��ȫ������
                        end
                    end
                    all_location_relay(all_location_relay(:,1)==0,:)=[];%�����all_location_relay�е�ȫ����
                    all_location_relay = [all_location_relay;location_relay];%��location_relay�������ݸ��¼ӽ�all_location_relay������
                    for n = 1:length(store_road(:,1))
                        if store_road(n,1) == nextjunction(j,i) && store_road(n,2) == end_relay_junction(j)%��store_road���ҵ�һ�������������յ㣨�ϸ���Junction�㣩���յ�������ǵ����
                            store_road(n,1) = end_relay_junction(j);%��������Ϊ���ǵ���㣬ע�⣺end_relay_junction(j)����㣬nextjunction(j,i)���յ�
                            store_road(n,2) = nextjunction(j,i);%�����յ��Ϊ���ǵ��յ�
                        end
                    end
                    if junction_time(nextjunction(j,i),4)>t%������ݹ������ϸ���junction��Ĵ��ʱ���������ڵĴ��tʱ��
                        junction_time(nextjunction(j,i),4)=t;%���´��ʱ��
                    else%������ݹ������ϸ���junction��Ĵ��ʱ���������ڵĴ��tʱ��
                        relay = [0 0];%���������·�ε�relay��Ϣ��ʹ���޷��ټ�����һ�λ��ݱ���                       
                    end
                elseif junction_time(nextjunction(j,i),4)==0%���junction_time�е�ʱ��δ��
                    relay = [0 0];%�����relay��Ϣ,ʹ���޷�������һ�α���
                elseif isempty(find(nextjunction(j,i) == endtree(:), 1))==0%���������ֹ��
                    junction_time(nextjunction(j,i),4) = t;%����junction_time�еĴ��ʱ��
                    for n = 1:length(all_location_relay(:,1))
                        if all_location_relay(n,1)==end_relay_junction(j) && all_location_relay(n,2)==nextjunction(j,i)%��all_location_relay�������ҳ�Ŀ�ĵ������ǵ�Ŀ�ģ��յ�������ǵ��յ����
                            all_location_relay(n,:)=0; %�ҵ��󣬽�����ȫ������
                        end
                    end
                    all_location_relay(all_location_relay(:,1)==0,:)=[];%�����all_location_relay�е�ȫ����
                    all_location_relay = [all_location_relay;location_relay];%��location_relay�������ݸ��¼ӽ�all_location_relay������                    
                    relay = [0 0];%�����relay��Ϣ,ʹ���޷�������һ�α���                   
                end
                %������Ӧ�ñ�дǰ��������������������ֹ�ڵ�򸲸�junction���ʱ��Ϊ0����������е�һ����Ӧ����ô�����������Ϣ��������
                %����Ĵ��뻹�����Ҫ��Ҫ
                relayinf(1,1:2) = relay;
                relayinf(1,3) = end_relay_junction(j);%end_relay_junction(j)��·�����
                relayinf(1,4) = nextjunction(j,i);%nextjunction(j,i)��·���յ�
                relayinf(1,5) = t;
                temp = [temp;relayinf];
            end
        end
        start_relay_junction = [];
        end_relay_junction = [];
        nextjunction = [];
        t_relay = [];
        last_relay = [];
        for j = 1:length(temp(:,1))
            if temp(j,1)~=0
                last_relay = [last_relay;temp(j,1:2)];
                start_relay_junction = [start_relay_junction,temp(j,3)];
                end_relay_junction = [end_relay_junction,temp(j,4)];
                t_relay = [t_relay,temp(j,5)];
            end
        end
        
        if isempty(last_relay)%���������������Ϣ��������ȫ���������ô�Ϳ�������while(1)ѭ��
            break;
        end
    
    end
    PDR = mean(PDR_set(:,1));%����PDR

