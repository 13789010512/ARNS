function [width_segm,bound_segm]=width_segment(N_part,N_iter,A)
%% 验证计算width of segment algorithm正确

k=1:N_part;
w_bro=zeros(1,N_part);
w_porti= (1+A).^((k-1)/N_part)*((1+A)^(1/N_part)-1)/A;
for i=1:N_part
    w_bro(i+1) = sum(w_porti(1:i));
end
board_segment=zeros(N_iter+1,N_part^N_iter+1);
board_segment(1,1)=0;
board_segment(1,2)=1;
for i=2:N_iter+1
    for j=1:N_part^(i-2)
        board_segment(i,(2+N_part*(j-1)):(N_part*j+1))= (board_segment(i-1,j+1)-board_segment(i-1,j))*w_bro(2:N_part+1)+ones(1,N_part)*board_segment(i-1,j);%0点从sender开始
    end 
end

bound_segm=board_segment(N_iter+1,N_part^N_iter+1:-1:1);
width_segment1=board_segment(i,2:length(board_segment))-board_segment(i,1:length(board_segment)-1);


width_segment2=zeros(1,N_part^N_iter);
k=1:N_part^N_iter; 
x=zeros(N_iter,length(k));
x(1,1:length(k))=mod(k,N_part);
x(1,find(x(1,1:length(k))==0))=N_part;
y(1,:)=ceil(k/N_part);
for j=1:N_iter-1
    x(1+j,1:length(k))=mod(y(j,1:length(k)),N_part);
    x(1+j,find((x(j+1,1:length(k))==0)))=N_part;
    y(1+j,:)=ceil(y(j,:)/N_part);
end
if N_part<=2
    for j=1:N_iter
      x(j,find(x(j,1:length(k))<=2))=0;
    end
else
    for j=1:N_iter
      x(j,find(x(j,1:length(k))<=2))=0;
      x(j,find(x(j,1:length(k))>=3))=x(j,find(x(j,1:length(k))>=3))-2;
    end
end
for j=1:N_part^N_iter
    width_segment2(j)= 2^sum(x(:,j))/(2^(N_part-1)^N_iter); 
end
width_segm=width_segment1;%insex=0是从border开始
a=sum(width_segment1);
b=sum(width_segment2);