function [delay_contention,n_collision] = jyc_contention(n_contention,cw_initial,n_collision_max,mini_DIFS_mark)

%% ����ʱ��
%% delay_contention:������ʱ��n_collision:��ͻ�Ĵ�����n_contention������Դ�ĸ�����cw������n_collision_max�������ͻ���ԵĴ���;mini_DIFS_mark:���ڱ�־�Ƿ���ѭ��ͻ�󴰷Ŵ�Ļ��ƣ�1��ʾ"��"

global n_fail w_mini_slot t_slot CW_max_DIFS  ;

cw=cw_initial;
delay_contention = zeros(n_collision_max+1,1);
n_collision = 0;
while n_collision<n_collision_max %����ͻ����С������ͻ�����������whileѭ��������ͻ������������ͻ����������whileѭ��
    t_backoff =randi(cw,[n_contention,1])-1;%Ϊn_contention�����뾺���Ľڵ�����˱�ʱ��
    
    clear collide_Detect_Martrix_index;
    collide_Detect_Martrix_index=find(t_backoff==min(t_backoff));%�ҳ��˱�ʱ��ͬΪ��Сֵ�Ľڵ�
    m=length(collide_Detect_Martrix_index);%ͨ��length����˱�ʱ��ͬΪ��Сֵ�Ľڵ�ĸ���
    if m==0%���Ϊ0�������
        sprintf('n_contention:%d,t_backoff:%d,cw:%d,mini_DIFS_mark:%d',n_contention,t_backoff,cw,mini_DIFS_mark)
        pause
    end
    if m==1%���Ϊ1����ʾ����һ�γɹ���ֻ��һ���ڵ��˱�ʱ��Ϊ��Сֵ
        if mini_DIFS_mark==1
            delay_contention(n_collision+1) =  delay_contention(n_collision+1)+t_backoff(collide_Detect_Martrix_index)*w_mini_slot;
        else
            delay_contention(n_collision+1) =  delay_contention(n_collision+1)+t_backoff(collide_Detect_Martrix_index)*t_slot/2;
        end
        break;
    else%�����Ϊ1����ʾ����ʧ�ܣ��ж���ڵ��˱�ʱ��Ϊ��Сֵ
        if mini_DIFS_mark==1%�����ǽ�����Ϣ����ʱ�������mini_DIFS_markΪ1
            cw = min((2^(n_collision+2)),CW_max_DIFS);%��һ�����ò�����ͻʧ�ܺ�cw��2�����4���������ʧ����2^t���ƣ�CW_max_DFIS�����ô�������ӵ��cw����Ŀ����reslut�����ж���
            delay_contention(n_collision+1) =  delay_contention(n_collision+1)+t_backoff(collide_Detect_Martrix_index(1))*w_mini_slot;
        else%�����ǽ���С����ڵ�����ʱ�������mini_DIFS_markΪ0
            delay_contention(n_collision+1) =  delay_contention(n_collision+1)+t_slot/2*(cw-1);
            cw=cw*2;%������ָ���˱��㷨ԭ��cwΪ���ô��ڵ�ʱ϶��������һ�����ò�����ͻʧ�ܺ�cw��2�����4���������ʧ���Դ����ơ�
        end
        n_collision=n_collision+1;%��ͻ����+1
  
    end
end
if n_collision>=n_collision_max%����ͻ�������ڵ�������ͻ����
    n_fail=n_fail+1;%ʧ�ܴ���+1
    delay_contention =Inf; %����ͻ�����ﵽ�Ͻ磬����ʧ�ܴ��������Ѷ�Ӧ��delay_contention����Чֵ
end