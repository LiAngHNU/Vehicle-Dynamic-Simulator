%% 清空环境变量
clc
clear

%% 训练数据预测数据提取及归一化
%导入输入输出数据，数据和m文件在一个文件夹中。
data=xlsread('工作簿123.xlsx', 'Sheet1', 'A2:U1975');

%找出训练数据和预测数据，1700条训练数据，275条测试数据。
input_train=data(1:1700,1:20)';
output_train=data(1:1700,21)';
input_test=data(1701:end,1:20)';
output_test=data(1701:end,21)';

%选连样本输入输出数据归一化

[inputn,inputps]=mapminmax(input_train)
[outputn,outputps]=mapminmax(output_train); 

%% BP网络训练
% %初始化网络结构
net=newff(inputn,outputn,8);%隐含层节点数量经验公式p=sqrt(m+n)+a ，故分别取2~13进行试验

net.trainParam.epochs=100 % 对整个训练组训练100次
net.trainParam.lr=0.001;% 设置学习率
net.trainParam.goal=0.0001;  %网络输出和目标值的差的平方再求平均值

%网络训练
net=train(net,inputn,outputn);

%% BP网络预测
%预测数据归一化
inputn_test=mapminmax('apply',input_test,inputps);
 
%网络预测输出
an=sim(net,inputn_test);
 
%将输出的结果
BPoutput=mapminmax('reverse',an,outputps);

%% 结果分析

figure (1)
plot(BPoutput,'bo')
hold on
plot(output_test,'gp');
legend('预测输出','期望输出')
title('BP网络预测输出','fontsize',11)
ylabel('函数输出','fontsize',11)
xlabel('样本','fontsize',11)
set(gca,'XTick',[1:1:11])
zoom on;
grid on
%预测误差
error=BPoutput-output_test;


figure (2)
x = 1:1:275;
plot(x,error,'-o')
title('BP网络预测误差','fontsize',11)
ylabel('误差','fontsize',11)
xlabel('样本','fontsize',11)
set(gca,'XTick',[1:1:275])
axis([1,300,-5,10])
str1=num2str(error',2);
text(x,error,str1)
grid on

figure (3)
plot((output_test-BPoutput)./BPoutput,'r');
set(gca,'yticklabel',{'0','5%','10%','15%','20%','25%','30%','35%','40%','45%','50%'});
title('神经网络预测误差百分比')
ylabel('百分比','fontsize',11)
xlabel('样本','fontsize',11)
set(gca,'XTick',[0:1:6])
grid on

error=sum(abs(error))/275
w1 = net.iw{1,1};%输入层到中间层的权值
w2 = net.lw{2,1};
b1 = net.b{1};%中间各层神经元阈值
b2 = net.b{2};