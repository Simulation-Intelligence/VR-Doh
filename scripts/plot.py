import matplotlib.pyplot as plt
import seaborn as sns

# 使用 Seaborn 样式
sns.set_theme(style="whitegrid")

# 数据
n_grid = [16, 32, 48, 64, 80, 96, 112, 128]
fps_n_grid = [248.4, 216.0, 167.5, 118.4, 66.6, 38.4, 24.3, 19.2]

substep = [3, 5, 8, 10, 15, 20, 25, 30]
fps_substep = [144.8, 118.4, 100.2, 81.8, 62.4, 50.2, 42.3, 37.4]

particle = [4, 5, 6, 7, 8, 9, 10, 11, 12]
fps_particle = [159.6, 143.7, 138.1, 129.3, 118.4, 106.4, 105.3, 99.7, 88.9]

# 设置图形尺寸
plt.figure(figsize=(15, 5))  # 总体尺寸

# 自定义颜色和标记样式
colors = ['#E37777', '#F4A261', '#2A9D8F']
markers = ['o', 'o', 'o']

# 定义比例
ratio = 3 / 4

# 绘制不同 n_grid 的图
ax1 = plt.subplot(1, 3, 1)
ax1.plot(n_grid, fps_n_grid, marker=markers[0], linestyle='-', color=colors[0], linewidth=1.5, markersize=5)
ax1.set_title("FPS vs Grid Resolution", fontsize=16)
ax1.set_xlabel("Grid Resolution", fontsize=16)
ax1.set_ylabel("FPS", fontsize=16)
ax1.set_ylim(0, 260)  # y轴从0开始
ax1.grid(True, linestyle='--', linewidth=0.5, color='gray', alpha=0.7)
x_left, x_right = ax1.get_xlim()
y_low, y_high = ax1.get_ylim()
ax1.set_aspect(abs((x_right - x_left) / (y_low - y_high)) * ratio)
for spine in ax1.spines.values():
    spine.set_color('black')

# 绘制不同 substeps per frame 的图
ax2 = plt.subplot(1, 3, 2)
ax2.plot(substep, fps_substep, marker=markers[1], linestyle='-', color=colors[1], linewidth=1.5, markersize=5)
ax2.set_title("FPS vs Substeps Per Frame", fontsize=16)
ax2.set_xlabel("Substeps Per Frame", fontsize=16)
ax2.set_ylabel("FPS", fontsize=16)
ax2.set_ylim(20, 160)  # y轴从20开始
ax2.grid(True, linestyle='--', linewidth=0.5, color='gray', alpha=0.7)
x_left, x_right = ax2.get_xlim()
y_low, y_high = ax2.get_ylim()
ax2.set_aspect(abs((x_right - x_left) / (y_low - y_high)) * ratio)
for spine in ax2.spines.values():
    spine.set_color('black')

# 绘制不同 particles per cell 的图
ax3 = plt.subplot(1, 3, 3)
ax3.plot(particle, fps_particle, marker=markers[2], linestyle='-', color=colors[2], linewidth=1.5, markersize=5)
ax3.set_title("FPS vs Particles Per Cell", fontsize=16)
ax3.set_xlabel("Particles Per Cell", fontsize=16)
ax3.set_ylabel("FPS", fontsize=16)
ax3.set_ylim(80, 170)  # y轴从80开始
ax3.grid(True, linestyle='--', linewidth=0.5, color='gray', alpha=0.7)
x_left, x_right = ax3.get_xlim()
y_low, y_high = ax3.get_ylim()
ax3.set_aspect(abs((x_right - x_left) / (y_low - y_high)) * ratio)
for spine in ax3.spines.values():
    spine.set_color('black')

# 调整布局
plt.tight_layout(pad=1.5)
plt.gcf().set_facecolor('white')  # 设置白色背景

# 保存和显示图像
plt.savefig("img/performance.pdf", dpi=300, bbox_inches='tight')
plt.show()
