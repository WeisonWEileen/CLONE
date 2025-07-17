import numpy as np

def mat_update(prev_mat, mat):
    if np.linalg.det(mat) == 0:
        return prev_mat
    else:
        return mat


def fast_mat_inv(mat):
    ret = np.eye(4)
    ret[:3, :3] = mat[:3, :3].T
    ret[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return ret

import torch

def quaternion_conjugate(q):
    # q: [B, 4] = [x, y, z, w]
    # 共轭：q* = [-x, -y, -z, w]
    x, y, z, w = q.unbind(dim=-1)
    return torch.stack([-x, -y, -z, w], dim=-1)

def quaternion_mul(q1, q2):
    # q1, q2: [B,4] = [x, y, z, w]
    x1, y1, z1, w1 = q1.unbind(dim=-1)
    x2, y2, z2, w2 = q2.unbind(dim=-1)

    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

    return torch.stack([x, y, z, w], dim=-1)

def rotate_quaternion(A, B):
    # 使用B表示的旋转作用于A: Q = B * A * B*
    B_conj = quaternion_conjugate(B)
    return quaternion_mul(quaternion_mul(B, A), B_conj)

# 示例
if __name__ == "__main__":
    # 假设A和B为[B,4], 且为单位四元数
    A = torch.tensor([[0.0, 0.0, 0.0, 1.0],    # 单位四元数 (无旋转)
                      [0.0, 0.7071, 0.0, 0.7071]]) # 约90度绕y轴旋转
    B = torch.tensor([[0.7071, 0.0, 0.0, 0.7071],  # 约90度绕x轴旋转
                      [0.0, 0.0, 0.7071, 0.7071]]) # 约90度绕z轴旋转

    Q = rotate_quaternion(A, B)
    print(Q)