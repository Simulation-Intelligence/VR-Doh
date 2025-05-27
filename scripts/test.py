import taichi as ti
import time

ti.init(arch=ti.gpu)  # 使用 GPU 进行加速

# 定义一个大的范围
N = 1000000
n=10000
# 创建一个 Taichi field
data = ti.field(dtype=ti.f32, shape=N)

# 初始化数据
@ti.kernel
def initialize():
    for i in range(N):
        data[i] = ti.random()

# 定义一个耗时较长的计算函数
# @ti.func
# def complex_computation(x):
#     sum = 0.0
    
#     return sum

# 定义内核函数
@ti.kernel
def compute1():
    for i in range(N):
        if i % 100 == 0:  # 每隔1000个进行复杂计算
            sum = 0.0
            for j in range(100000):  # 模拟耗时计算
                sum += ti.sin(data[j])
            data[i] = sum


@ti.kernel
def compute2():
    for i in range(n):
        sum = 0.0
        for j in range(100000):  # 模拟耗时计算
            sum += ti.sin(data[j])
        data[i] = sum

# 初始化数据
initialize()

# 测量计算时间
start_time = time.time()

# 计算
compute1()
ti.sync()
end_time = time.time()

compute2()
ti.sync()
end_time2 = time.time()

# 打印结果

print(data.to_numpy())
print(f"Execution1 time: {end_time - start_time} seconds")

print(f"Execution2 time: {end_time2 - end_time} seconds")
