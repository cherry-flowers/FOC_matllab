function [b0, b1, b2, a1, a2] = butterworth_2nd_order_coeffs(fs, fc)
% 巴特沃斯二阶低通滤波器系数计算
% 输入:
%   fs - 采样频率 (Hz)
%   fc - 截止频率 (Hz)
% 输出:
%   b0, b1, b2 - 分子系数
%   a1, a2 - 分母系数

% 验证输入参数
if fc >= fs/2
    error('截止频率必须小于奈奎斯特频率 (fs/2 = %.2f Hz)', fs/2);
end

% 计算归一化截止频率 (0~1)
Wn = fc / (fs/2);

% 设计二阶巴特沃斯低通滤波器
[b, a] = butter(2, Wn, 'low');

% 提取系数
b0 = b(1);
b1 = b(2);
b2 = b(3);
% 注意: a(1)是归一化系数(通常为1)，因此我们取a(2)和a(3)
a1 = a(2);
a2 = a(3);

% 显示结果
disp('=== 巴特沃斯二阶低通滤波器系数 ===');
fprintf('采样频率 (fs): %.2f Hz\n', fs);
fprintf('截止频率 (fc): %.2f Hz\n', fc);
fprintf('归一化截止频率 (Wn): %.4f\n', Wn);
fprintf('分子系数: b0 = %.8f, b1 = %.8f, b2 = %.8f\n', b0, b1, b2);
fprintf('分母系数: a1 = %.8f, a2 = %.8f\n', a1, a2);

% 绘制频率响应曲线以验证设计
freqz(b, a, 1024, fs);
title(['巴特沃斯二阶低通滤波器: F_c=', num2str(fc), 'Hz, F_s=', num2str(fs), 'Hz']);
grid on;
hold on;
plot([fc fc], [-200 5], 'r--'); % 标记截止频率点
text(fc, -90, ['F_c = ', num2str(fc), ' Hz'], 'Color', 'red');
hold off;
end