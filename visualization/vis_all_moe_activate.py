import seaborn as sns
import pandas as pd
import numpy as np
import json
import os   
import os.path as osp

def run_single(expert_path):
    if "_moel" in expert_path:
        num_layers = 3
        num_experts = 4
    elif "_moes" in expert_path or "_moee" in expert_path:
        num_layers = 3
        num_experts = 8
    elif "smoe" in expert_path:
        num_layers = 1
        num_experts = 4
    elif "a1" in expert_path:
        num_layers = 4
        num_experts = 4
    elif "a2" in expert_path:
        num_layers = 5
        num_experts = 4
    elif "m1" in expert_path:
        num_layers = 2
        num_experts = 4
    else:
        return

    expert_files = [x for x in os.listdir(expert_path) if not x == 'walk.json']
    expert_category = [x.replace('.json', '') for x in expert_files]
    expert_files = [osp.join(expert_path, file) for file in expert_files if file.endswith('.json')]
    

    expert_data = {}
    for i, expert_file in enumerate(expert_files):
        with open(expert_file, 'r') as f:
            data = json.load(f)
            print('Processing: ', expert_category[i])

            keys_to_remove = []
            for key, value in data.items():
                v = dict(eval(value.replace('Counter(', '').replace(')', '')))
                new_v = []
                for j in range(num_experts):
                    if j not in v:
                        new_v.append(0)
                        continue
                    new_v.append(v[j])
                data[key] = new_v
            
            for key, value in data.items():
                if 'Expert 0' in key:
                    nk = key.replace('Expert 0', 'Expert 1')
                    nv = data[nk]
                    for j in range(num_experts):
                        value[j] += nv[j]
                    data[key] = value
                else:
                    keys_to_remove.append(key)
                    continue
                
            for key in keys_to_remove:
                del data[key]
            print(data)
            expert_data[expert_category[i]] = data

    category_order = list(expert_data.keys())
    category_order.sort()
    category_order[0], category_order[1], category_order[2], category_order[3] = category_order[3], category_order[0], category_order[1], category_order[2]
    category_order[2], category_order[-1] = category_order[-1], category_order[2]
    category_order[5], category_order[-1] = category_order[-1], category_order[5]
    category_order[-3], category_order[-2] = category_order[-2], category_order[-3]
    category_order[5], category_order[4] = category_order[4], category_order[5]

    activate_value = [[expert_data[c][key] for key in expert_data[c].keys()] for c in category_order]
    activate_value = np.array(activate_value)

    assert activate_value.shape[1] == num_layers
    assert activate_value.shape[2] == num_experts

    print(activate_value.shape)

    activate_value_norm = activate_value / (activate_value.sum(axis=2, keepdims=True) + 1e-8)
    activate_value_norm = activate_value_norm.reshape(activate_value_norm.shape[0], -1)
    rows = [x.replace('still', 'stand').replace('stright', '').replace('straight', '') for x in category_order]
    cols = []
    for l in range(num_layers):
        for e in range(num_experts):
            cols.append(f'l{l}.e{e}')

    v_min = activate_value_norm.min()   
    v_max = activate_value_norm.max()
    df_norm_row = pd.DataFrame(activate_value_norm, index=rows, columns=cols)
    # df_norm_row=df.divide(df.sum(axis=0), axis=1)
    sns.heatmap(df_norm_row, annot=True, annot_kws={'size': 7}, square=True, cbar=True, vmin=v_min, vmax=v_max)

    activate_value_norm = activate_value_norm.reshape(-1, num_layers, num_experts)
    import matplotlib.pyplot as plt
    plt.figure(figsize=(15, 5 * ((num_layers + 2) // num_layers)))
    for layer_idx in range(num_layers):
        plt.subplot(((num_layers + 2) // num_layers), num_layers, layer_idx + 1)
        plt.xticks(size=18)
        plt.yticks(size=18)
        
        # 获取当前层的激活值 (n x e)
        layer_data = activate_value_norm[:, layer_idx, :]
        
        # 创建热力图
        sns.heatmap(layer_data, cmap='RdBu_r',
                    xticklabels=[f'E{i+1}' for i in range(num_experts)],
                    yticklabels=rows if layer_idx == 0 else [],
                    annot=True,
                    annot_kws={'size': 7},
                    square=True,
                    cbar=False,
                    fmt='.2f',
                    vmin=v_min,
                    vmax=v_max)
        
        plt.title(f'Layer {layer_idx + 1}', size=18)
        # plt.xlabel('Expert')
        if layer_idx == 0:
            plt.ylabel('Category', size=18)

    wspace = -0.77 if num_experts == 4 else -0.2
    if num_layers == 2:
        wspace = -0.82
    elif num_layers == 4:
        wspace = -0.70
    elif num_layers == 5:
        wspace = -0.50
    plt.subplots_adjust(wspace=wspace, hspace=0.2)

    expert_name = expert_path.split('/')[-1]
    plt.savefig(f"{expert_name}_activate_value_heatmap.pdf", dpi=600)

dirs = os.listdir('/home/yutang/Desktop/CLONE/visualization')
dirs = [osp.join('/home/yutang/Desktop/CLONE/visualization', dir) for dir in dirs if osp.isdir(osp.join('/home/yutang/Desktop/CLONE/visualization', dir))]
for dir in dirs:
    run_single(dir)