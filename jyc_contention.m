function [delay_contention,n_collision] = jyc_contention(n_contention,cw_initial,n_collision_max,mini_DIFS_mark)

%% 竞争时间
%% delay_contention:竞争延时；n_collision:冲突的次数；n_contention：竞争源的个数；cw：窗宽；n_collision_max：允许冲突再试的次数;mini_DIFS_mark:用于标志是否遵循冲突后窗放大的机制，1表示"是"

global n_fail w_mini_slot t_slot CW_max_DIFS  ;

cw=cw_initial;
delay_contention = zeros(n_collision_max+1,1);
n_collision = 0;
while n_collision<n_collision_max %当冲突次数小于最大冲突次数，则进行while循环，当冲突次数等于最大冲突次数则跳出while循环
    t_backoff =randi(cw,[n_contention,1])-1;%为n_contention个参与竞争的节点分配退避时间
    
    clear collide_Detect_Martrix_index;
    collide_Detect_Martrix_index=find(t_backoff==min(t_backoff));%找出退避时间同为最小值的节点
    m=length(collide_Detect_Martrix_index);%通过length求出退避时间同为最小值的节点的个数
    if m==0%如果为0，则出错
        sprintf('n_contention:%d,t_backoff:%d,cw:%d,mini_DIFS_mark:%d',n_contention,t_backoff,cw,mini_DIFS_mark)
        pause
    end
    if m==1%如果为1，表示争用一次成功，只有一个节点退避时间为最小值
        if mini_DIFS_mark==1
            delay_contention(n_collision+1) =  delay_contention(n_collision+1)+t_backoff(collide_Detect_Martrix_index)*w_mini_slot;
        else
            delay_contention(n_collision+1) =  delay_contention(n_collision+1)+t_backoff(collide_Detect_Martrix_index)*t_slot/2;
        end
        break;
    else%如果不为1，表示争用失败，有多个节点退避时间为最小值
        if mini_DIFS_mark==1%当我们进行消息争用时，输入的mini_DIFS_mark为1
            cw = min((2^(n_collision+2)),CW_max_DIFS);%第一次争用产生冲突失败后，cw从2变成了4，多次争用失败以2^t类推，CW_max_DFIS是争用窗口最大可拥有cw的数目，在reslut函数中定义
            delay_contention(n_collision+1) =  delay_contention(n_collision+1)+t_backoff(collide_Detect_Martrix_index(1))*w_mini_slot;
        else%当我们进行小区间节点争用时，输入的mini_DIFS_mark为0
            delay_contention(n_collision+1) =  delay_contention(n_collision+1)+t_slot/2*(cw-1);
            cw=cw*2;%二进制指数退避算法原则，cw为争用窗口的时隙个数，第一次争用产生冲突失败后，cw从2变成了4，多次争用失败以此类推。
        end
        n_collision=n_collision+1;%冲突次数+1
  
    end
end
if n_collision>=n_collision_max%当冲突次数大于等于最大冲突次数
    n_fail=n_fail+1;%失败次数+1
    delay_contention =Inf; %当冲突次数达到上界，计数失败次数，并把对应的delay_contention置无效值
end