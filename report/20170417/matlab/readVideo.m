function readVideo( )
fileName = 'v.MOV'; 
obj = VideoReader(fileName);
numFrames = 3 * 30;
a=[];
 for k = 1 : numFrames
     frame = read(obj,k);
     %imwrite(frame,strcat(num2str(k),'.jpg'),'jpg');
     a(end+1) = mean(frame(:));
 end

 x=1:1:numFrames;
 x = x/30;
 figure(x ,a);
end

function figure(a ,b)
plot(a,b);
%axis([0 3 5 55])
%set(gca,'XTick',1:1:3) %改变x轴坐标间隔显示 这里间隔为10
%set(gca,'YTick') %改变y轴坐标间隔显示 这里间隔为10
title('The Mean Pixel Values of First 3 Seconds in a Video')
xlabel('Time (s)');
ylabel('Mean of Pixel Value');
end

