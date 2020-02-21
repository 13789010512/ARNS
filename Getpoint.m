a=imread('Curve.jpg');
imshow(a)
hold on;
markpoint = zeros(1,2);
for i = 1:1
    markpoint(i,1:2) = ginput(1);
    plot(markpoint(i,1),markpoint(i,2),'o');
    hold on;
end
%fdata1 = fopen('jyc_location_markMapofCurve.txt','w');
%fprintf(fdata1,'%d ',markpoint(:,:));
%fprintf(fdata1,'\n');
%fclose(fdata1);
disp(markpoint)
