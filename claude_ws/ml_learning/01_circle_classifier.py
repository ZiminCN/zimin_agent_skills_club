"""
小白带你学神经网络 第1课：判断点是否在圆内
=============================================
场景：输入一个点的坐标 (x, y)，判断它在单位圆内部还是外部
      圆内=0，圆外=1（单位圆：x² + y² >= 1.0 为圆外）

流程：生成数据 → 定义神经网络 → 训练 → 测试 → 导出ONNX

运行方式：
    python3 ml_learning/01_circle_classifier.py

依赖（一般都有，没有的话 pip3 install onnx）：
    - PyTorch (torch)
    - onnx (仅导出需要)
"""

import numpy as np
import torch
import torch.nn as nn

# ============================
# 第1步：生成训练数据
# ============================
# 我们要教会神经网络判断"点在不在圆内"
# 所以先制造 10000 个随机点，以及每个点的正确答案（标签）
np.random.seed(42)  # 固定随机种子，确保每次运行结果一致
n = 10000

# 在 [-1.5, 1.5] 范围内随机生成 10000 个 (x, y) 坐标点
X = np.random.uniform(-1.5, 1.5, (n, 2)).astype(np.float32)

# 计算每个点的标签：
#   - 如果 x² + y² >= 1.0 → 在圆外 → 标签为 1
#   - 如果 x² + y² < 1.0  → 在圆内 → 标签为 0
y = (X[:, 0]**2 + X[:, 1]**2 >= 1.0).astype(np.float32)

# 转成 PyTorch 的张量（Tensor），神经网络只认这个格式
X_t = torch.from_numpy(X)
y_t = torch.from_numpy(y).reshape(-1, 1)

print("=" * 50)
print("第1步：数据准备完成")
print(f"  训练样本数: {len(X_t)}")
print(f"  每个样本: (x, y) 坐标 → 标签(0=圆内, 1=圆外)")
print(f"  示例前3个:")
for i in range(3):
    label = "圆内" if y[i] == 0 else "圆外"
    print(f"    ({X[i,0]:+.3f}, {X[i,1]:+.3f}) → {label}")


# ============================
# 第2步：定义神经网络结构
# ============================
# 这是一个"多层感知机"（MLP），由3层组成：
#
#   输入层(2个神经元)          ← 接收 (x, y) 两个值
#       ↓
#   隐藏层1(32个神经元)        ← 学习非线性的特征组合
#       ↓  ReLU激活函数         ← 引入非线性（不然多层=一层没区别）
#   隐藏层2(32个神经元)        ← 进一步组合特征
#       ↓  ReLU激活函数
#   输出层(1个神经元)          ← 输出 0~1 之间的概率
#       ↓  Sigmoid激活函数      ← 把输出压缩到 [0,1]，表示"是圆外的概率"
#
# nn.Sequential 就是把各层按顺序串起来
model = nn.Sequential(
    nn.Linear(2, 32),    # 线性层：y = xW^T + b，输入2维，输出32维
    nn.ReLU(),           # 激活函数：max(0, x)，把负数干掉，引入非线性
    nn.Linear(32, 32),   # 第二层：32维 → 32维
    nn.ReLU(),           # 再来一次非线性
    nn.Linear(32, 1),    # 输出层：32维 → 1维（最终得分）
    nn.Sigmoid()         # 把得分压缩到 0~1，解释为概率
)

total_params = sum(p.numel() for p in model.parameters())
print(f"\n第2步：神经网络定义完成")
print(f"  结构: {model}")
print(f"  总参数量: {total_params} 个")
print(f"  说明: 这 {total_params} 个参数就是模型要学习的东西")


# ============================
# 第3步：训练模型
# ============================
# 训练 = 不断调整那 1185 个参数，让预测越来越准
#
# 关键概念：
#   - 损失函数 (loss_fn)：衡量预测值和真实值差距有多大
#     BCELoss = 二分类交叉熵损失，值越小说明预测越准
#   - 优化器 (optimizer)：决定"怎么调参数"
#     Adam = 自适应学习率优化器，比普通SGD更好用
#   - 反向传播 (backward)：自动计算每个参数对损失的影响方向
#   - epoch：完整过一遍所有训练数据

loss_fn = nn.BCELoss()         # 二分类损失函数
optimizer = torch.optim.Adam(model.parameters(), lr=0.01)  # Adam优化器，lr=学习率

print(f"\n第3步：开始训练...")
for epoch in range(200):      # 训练 200 轮
    # 前向传播：把数据送进模型，得到预测结果
    pred = model(X_t)

    # 计算损失：预测值 vs 真实值
    loss = loss_fn(pred, y_t)

    # 反向传播：三大步，记住这个模式
    optimizer.zero_grad()  # 1) 清空上一次的梯度
    loss.backward()        # 2) 自动计算每个参数的梯度（核心魔法！）
    optimizer.step()       # 3) 根据梯度更新参数

    # 每 40 轮打印一次进度
    if epoch % 40 == 0:
        # 准确率：预测值 > 0.5 判为圆外(1)，否则判为圆内(0)
        acc = ((pred > 0.5).float() == y_t).float().mean()
        print(f"  第{epoch:3d}轮  loss={loss.item():.4f}  准确率={acc.item()*100:.0f}%")

print(f"  训练完成！")


# ============================
# 第4步：测试模型
# ============================
# 用一些特定位置的点来检验模型学得怎么样
with torch.no_grad():  # 测试时不需要计算梯度，省内存加速
    test_points = torch.tensor([
        [0.0, 0.0],   # 圆心
        [0.5, 0.0],   # 圆内
        [0.7, 0.7],   # 圆内 (0.7²+0.7²=0.98 < 1)
        [1.0, 0.0],   # 圆上 (边界情况)
        [0.0, 1.5],   # 圆外
        [-1.3, 0.0],  # 圆外
    ])

    probs = model(test_points)  # 模型预测概率

    print(f"\n第4步：测试结果")
    for pt, p in zip(test_points, probs):
        # 真实答案（用数学公式算的）
        truth = "圆内" if pt[0]**2 + pt[1]**2 < 1 else "圆外"
        # 模型预测
        guess = "圆内" if p < 0.5 else "圆外"
        # 对还是错
        mark = "✓" if truth == guess else "✗"
        print(f"  ({pt[0]:.1f}, {pt[1]:.1f}) 真实={truth}  预测={guess} {mark}  (概率={p.item():.3f})")


# ============================
# 第5步：导出 ONNX 模型
# ============================
# ONNX = Open Neural Network Exchange
# 导出的 .onnx 文件可以在任何框架/语言中加载推理
# 相当于模型的"通用格式"
dummy_input = torch.randn(1, 2)  # 一个假输入，用来"追踪"模型结构
torch.onnx.export(
    model,
    dummy_input,
    "./onnx_model/circle_model.onnx",          # 导出路径
    input_names=["input"],             # 输入节点名
    output_names=["output"],           # 输出节点名
    dynamic_axes={"input": {0: "batch"}}  # 允许批量大小可变
)

import os
size_kb = os.path.getsize("./onnx_model/circle_model.onnx") / 1024
print(f"\n第5步：ONNX 导出完成")
print(f"  文件: ./onnx_model/circle_model.onnx")
print(f"  大小: {size_kb:.1f} KB")
print(f"  查看: 用 https://netron.app 打开这个文件看结构")
print()
print("=" * 50)
print("总结：神经网络就是一个超大参数组合的数学函数")
print("      输入张量 → [一堆矩阵乘法 + 激活函数] → 输出张量")
print("      训练 = 用数据调整参数，让输出接近正确答案")
print("=" * 50)
