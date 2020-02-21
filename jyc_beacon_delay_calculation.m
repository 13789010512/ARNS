function [delay,PDR,newrelay,route_table,PDR_channel] = jyc_beacon_delay_calculation(route_table,density_EM,sender,relay,R,location_vehi,ima2,send_end,locationmark,posi_opt,fail_time)

%% ����log-partition���ƣ�������inform_vehi�п���l_way=R,ʹ��ֻ��one-hop
%% m_p_one_hop��massege_progress_one_hop
%��Ҫע��������������location_vehi_in_Rp��sender��ͨ�ŷ�Χ�ڵĳ����ڵ���Ϣ��Ȼ��relay���Ѿ�ѡ�������м̽ڵ㣬����������������þ������ѡ���relay������Ҫ���ѵ���ʱ
%����������֪sender��location_vehi_in_Rp��posi_opt���Ѿ�������ѡ������relay����ѡ����relay���ѵ�ʱ��
%    ������ʼ��
global t_SIFS s_EM s_RTB s_CTB rate_bit n_cont_mini CW_max_DIFS Linmap1m vehi_speed beacon_period s_ACK CW_min_DIFS;
PDR_channel = 1;%�ŵ����õ�PDR����ʼ��Ϊ1
relay_index = [];
for i = 1:length(route_table(:,1))
    route_table(i,5) = beacon_period*(randi(100)/100);%��route_table�ĵ�5�и���Ϊ�ó����ڵ���ű����ʱ��
    if route_table(i,2) == relay(1) && route_table(i,3) == relay(2)%�ҳ�relay��·�ɱ��е�λ��
        relay_index = i;
    end
end
if isempty(relay_index)%���û���ҳ�relay_index����ô������relay_index�������ű���£�������Ҫ�ҳ�route_table(:,3)����relay(2)��ֵ��С���������У���Ϊ���ǵ�relay_index
    D_value = zeros(length(route_table(:,1)),1);
    for i = 1:length(route_table(:,1))
        D_value(i) = abs(route_table(i,3)-relay(2));
    end
    [relay_index,~] = find(D_value(:,1)==min(D_value(:,1)));
end
if route_table(relay_index,6)>R%���������ѡrelay��sender�ľ������R��˵����relayӦ����Popt�㣬������Popt��ʱ������ּ����ϵ�һ��ƫ�Ϊ�˹�ƽ���������ΪR����
    route_table(relay_index,6)=R;
end

if route_table(relay_index,6)+route_table(relay_index,5)*vehi_speed>R%����м̽ڵ���sender�ľ���+�м̽ڵ��ű����ʱ��*���ٵĽ������ͨ�Ű뾶
    %��ô˵������ѡ�е�relay��һ���ĸ������ű���¼�϶��ʻ��ͨ�ŷ�Χ
    %�������һ��0��1��������ʾ�����������ڵ���������ʻ����������ʻ
    %0�����⣬�˳��Ὺ��ͨ�ŷ�Χ��������sender��Ҳ����old_relay���Ҳ�������Ϊnew_relay��ͨ��ʧ�ܣ�
    %1�����ڣ�ͨ�ųɹ�
    PDR = round(rand(1,1));%PDR��0��1��ֵ�����һ��
else
    PDR = 1;     
end

delay = 0;%��ʼ���ӳ�Ϊ0

%���ǽ�ͨ�ŷ�Χ�����������ĳ���λ�ö�����sender�ľ������
%����inform_vehi���飬�����е�1��Ϊ��������С����ţ���2��Ϊ���٣���3��ΪX���꣨����location_vehi_in_Rp�еĳ���X������0���������д泵����sender��ֱ�߾���
inform_vehi = [];%��ʼ��inform_vehiΪ�գ����ں���ʼ����route_table������������һ�£���1�д洢���ǳ�����ţ���2�д洢���ǳ�����ʻ����0���⣬1���ڣ���3�д洢�����ű���¼�϶(<beacon_period)����4�д洢���ǳ�����sender��ֱ�߾���
%�������inform_vehi�����е�ĳһ�����ڵ㣬���ű���¼�϶>beacon_period����������sender��ֱ�߾��룬�����ű���¼�϶���ã����ã�ԭ�ű���¼�϶-beacon_period��
newrelay = zeros(1,2);%��ʼ��newrelay���飬ֻ���ڳ���ԭrelay��ʻ��ͨ�ŷ�Χʱ�Ż���и�ֵ��newrelay�д������[0 ��sender�ľ���]

%��������ʼ������һ������ʧ��������ʱ
n_EM = max(1,random('Poisson',density_EM)); %�������n_EM��EMs�����Ӳ��ɷֲ�
if n_EM == 0
    n_EM=1;
end
[delay_contention,n_collision] = jyc_contention(n_EM,CW_min_DIFS,n_cont_mini,1);%�ŵ�������ϲ��ɹ��㲥RTS����ʱ������룬delay_contentionΪDIFS����ʱ�����飬n_collisionΪ�㲥RTS��ͻ����
t_accsee_channel = sum(delay_contention)+(s_RTB/rate_bit+t_SIFS)*(n_collision+1);%���㾺���ŵ����ܺ�ʱ����DIFS�������ʱ�Ӻͳɹ��㲥RTSʱ�ӹ��ɣ����гɹ��㲥RTSʱ���ɣ�RTS�㲥ʱ��+ģʽ�л�ʱ��SIFS��*���㲥RTS��ͻ����+1������
if t_accsee_channel==Inf      %����ŵ������ӳ����޴���ô��ʾ�㲥RTS��ͻ�����ﵽ����
    PDR_channel = 0;                  %������Ϣ�㲥�����ŵ����ù㲥RTS��ͻ���������ޣ�����ͨ��ʧ�ܣ����ټ����ش���
end
t_cont_CTB = s_CTB/rate_bit;  %����㲥CTSʱ��
t_data = t_SIFS+s_EM/rate_bit;%���㴫������ʱ�ӣ���ģʽ�л�ʱ��SIFS+���ݴ���ʱ�ӹ���
t_ACK = t_SIFS+s_ACK/rate_bit;%���㴫��ACK�ź�ʱ�ӣ���ģʽ�л�ʱ��SIFS+ACK�źŴ���ʱ�ӹ���
delay = t_accsee_channel+t_cont_CTB+t_data+t_ACK;

if delay==Inf                 %����������ŵ����ù㲥RTS��ͻ�����ﵽ���޵��µ�ʧ�ܣ����ǽ��˴�delay������ȱʧ����
    delay = NaN;              %NaN��ʾ����ȱʧ
    
    %�����������ش�ǰ�ĳ����ڵ�λ����Ϣ���¸�ֵ��inform_vehi�����У���ѡһ���µ�relay��ֵ��newrelay�����У�ֻ���ǡ�����ѡ�е�relay���ű���¼�϶��ʻ��ͨ�ŷ�Χ����һ��PDR=0�������
    %��Ϊ�������ŵ�����ʧ�ܶ�Σ��������������µ�PDR=0�����������contention�������Ѿ�������Ӧ���������ó��˶�Ӧ����ʱ��PDR����
elseif PDR==0 && delay~=Inf   %���PDR����0���Ҳ����ŵ����ô����������������ô���ǽ���һ����Χ�ڵĳ����ڵ�λ����Ϣ����
    route_table(:,5) = route_table(:,5)+delay;%����·�ɱ����ڵ���ű���¼��ʱ��
    R_expand = beacon_period*vehi_speed+R;%�����չ����뾶Ϊ�ű�������ڳ��Գ��٣��ڴ���RС��R_expand�ķ�Χ�ڻ���һ�����ܴ��ڴ�ͨ�ŷ�Χ����ʻ��ͨ�ŷ�Χ�ڵĳ����ڵ㣬������ű겻����������beacon_period��sender�ǲ�֪������ڵģ���֮��sender֪������ڣ�����ѡ����Ϊnew_relay
    if fail_time == 0%����ǵ�һ�ν�����ʱ���㺯������ô���������չ�����ڵĳ����ڵ㽫�����·�ɱ��м��㣬������ǵ�һ�ν��룬��ô��չ�����ڵĳ����ڵ��Ѿ��������·�ɱ��ˣ�û��Ҫ�ٴμ���
        [~,location_vehi_in_RE]=jyc_bar_posi_opt(location_vehi,locationmark,R_expand,Linmap1m,sender,send_end,ima2); %���location_vehi_in_RE��ͨ����չ�����ڵĳ����ڵ㣩
    else
        location_vehi_in_RE = [];
    end
    if isempty(location_vehi_in_RE)==0%���location_vehi_in_RE��Ϊ��
        for i = 1:length(route_table(:,1))
            for j = 1:length(location_vehi_in_RE(:,1))
                if route_table(i,2) == location_vehi_in_RE(j,2) && route_table(i,3) == location_vehi_in_RE(j,3)
                    location_vehi_in_RE(j,:)=0;%���Ѿ���·�ɱ��еĳ����ڵ���ų�
                end
            end
        end
        location_vehi_in_RE(location_vehi_in_RE(:,1)==0,:)=[];
        rank_num = length(route_table(:,1));%rank_num1��������route_table��Ϣ
        if isempty(location_vehi_in_RE)==0%���location_vehi_in_RE��Ϊ�㣬��ʾ���ڴ���RС��R_expand�ķ�Χ�ڵĳ����ڵ㣬��location_vehi_in_RE����չ�����ڵĳ����ڵ����·�ɱ��У���Ϊ����ڵ���һ�����ܴ�ͨ�ŷ�Χ����ʻ��ͨ�ŷ�Χ��
            for n = 1:length(location_vehi_in_RE(:,1))%����չ�����ڵĳ����ڵ�������·�ɱ���
                route_table(rank_num+1,1:4) = location_vehi_in_RE(n,1:4);
                route_table(rank_num+1,5) = beacon_period*(randi(100)/100)+delay;%��route_table�����ĵ�5�и�ֵΪ�ó����ڵ���ű����ʱ��������ǵ�delay�ӳ�
                route_table(rank_num+1,6) = (sum((route_table(rank_num+1,2:3)-sender(1,1:2)).^2))^0.5/Linmap1m;%��route_table�����ĵ�6�и�ֵΪ�ó����ڵ���sender�ľ���
                rank_num = rank_num+1;
            end
        end
    end
    %��������newrelay��inform_vehi���и�ֵ
    inform_vehi(:,1) = route_table(:,1);                                         %inform_vehi(:,1)�洢���ǳ������
    for i = 1:length(route_table(:,1))                                           %����inform_vehi������ϸ��Ϣ
        if route_table(i,5)-beacon_period>=0                                     %���·�ɱ��еĳ����ڵ��ű���¼�϶���ȴﵽ���ű��������
            if i~=relay_index                                                    %����õ㲻��relay�㣬��������һ����������
                inform_vehi(i,2) = round(rand(1,1));                             %inform_vehi(:,2)�洢���ǳ�����ʻ����0���⣬1����
            else                                                                 %����õ���relay��
                inform_vehi(i,2) = 0;                                            %����������ķ�������
            end
            inform_vehi(i,3) = route_table(i,5)-beacon_period;                   %inform_vehi(:,3)�洢�����µ��ű���¼�϶(<beacon_period)
            if inform_vehi(i,2)==0                                               %��������ڵ�������ʻ������ԭ�������beacon_period*vehi_speed
                inform_vehi(i,4) = route_table(i,6)+beacon_period*vehi_speed;    %inform_vehi(:,4)�洢���ǳ�����sender��ֱ�߾���
            else                                                                 %��������ڵ�������ʻ������ԭ�����ȥbeacon_period*vehi_speed
                inform_vehi(i,4) = route_table(i,6)-beacon_period*vehi_speed;    %inform_vehi(:,4)�洢���ǳ�����sender��ֱ�߾���
            end
        else                                                                     %���·�ɱ��еĳ����ڵ��ű���¼�϶����δ�ﵽ�ű��������
            if i~=relay_index                                                    %����õ㲻��relay�㣬��������һ����������
                inform_vehi(i,2) = round(rand(1,1));                             %inform_vehi(:,2)�洢���ǳ�����ʻ����0���⣬1����
            else                                                                 %����õ���relay��
                inform_vehi(i,2) = 0;                                            %����������ķ�������
            end
            inform_vehi(i,3) = route_table(i,5);
            inform_vehi(i,4) = route_table(i,6);
        end
    end
    dis_sender_newrelay = max(inform_vehi(inform_vehi(:,4)<R,4));%�ҳ�С��R�Ҿ���ֵ���ľ���
    if isempty(dis_sender_newrelay)%����Ҳ���        
        newrelay_index = length(inform_vehi(:,1))+1;
        inform_vehi(newrelay_index,1) = length(location_vehi(:,1))+1;
        inform_vehi(newrelay_index,2) = round(rand(1,1));
        inform_vehi(newrelay_index,3) = beacon_period*(randi(100)/100);
        inform_vehi(newrelay_index,4) = (sum((posi_opt(1,1:2)-sender(1,1:2)).^2))^0.5/Linmap1m;
        if inform_vehi(newrelay_index,4)>R%������Popt��ʱ������ּ����ϵ�һ��ƫ�Ϊ�˹�ƽ���������ΪR����
            inform_vehi(newrelay_index,4)=R;
        end
        newrelay = [0 inform_vehi(newrelay_index,4)];%��newrelay��ֵΪposi_opt
    else                   %����ҵ���
        [newrelay_index,~] = find(inform_vehi(:,4)==dis_sender_newrelay);
        if length(newrelay_index)>1
            newrelay_index = newrelay_index(1,1);
        end
        newrelay = [0 route_table(newrelay_index,6)];%��newrelay��ֵΪ�ó����ڵ�
    end
    if length(inform_vehi(:,1))-length(route_table(:,1))>0
        [posi_opt_road_index]=jyc_find_relayroad(locationmark,posi_opt);
        route_table(end+1,4) = posi_opt_road_index;
    end
    route_table(:,1) = inform_vehi(:,1);
    route_table(:,2) = 0;
    route_table(:,3) = inform_vehi(:,4);  
    route_table(:,5) = inform_vehi(:,3);
    route_table(:,6) = inform_vehi(:,4);
end


