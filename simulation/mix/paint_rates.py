import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 定义文件列表和标签列表
files = ['traceRate_1.txt', 'traceRate_2.txt', 'traceRate_3.txt', 'traceRate_4.txt', 'traceRate_5.txt']
labels = ['flow1', 'flow2', 'flow3', 'flow4', 'flow5']

# 创建图形
plt.figure(figsize=(9, 4.5))

# 对每个文件进行处理
for file, label in zip(files, labels):
    # 读取文件，假设文件是空格分隔的，列名分别是'Time'和'Rate'
    data = pd.read_csv(file, sep=' ', names=['Time', 'Rate'])

    # 将时间从纳秒转换为毫秒
    data['Time'] = data['Time'] / 1e6

    # 绘制折线图，x轴为时间，y轴为速率，标签为对应的流
    plt.plot(data['Time'], data['Rate'], label=label)

# 设置y轴刻度为5，并且只有10的倍数显示刻度值
plt.yticks(np.arange(0, plt.ylim()[1], 5), [str(i) if i%10==0 else '' for i in range(0, int(plt.ylim()[1])+1, 5)])

# 设置x轴每1ms一个刻度，每5ms一个刻度值
plt.xticks(np.arange(0, plt.xlim()[1], 1), [str(i) if i%5==0 else '' for i in range(0, int(plt.xlim()[1])+1, 1)])

plt.grid(True, linestyle='-', alpha=0.4)  # alpha 控制透明度

# 添加图例
plt.legend()

# 保存图形，你可以选择不同的文件格式，如.png, .jpg, .pdf等
plt.savefig('rates.png')

# 显示图形
plt.show()
