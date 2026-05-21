# JSON to JSONL Conversion Script

这个脚本用于将停车数据集从多个JSON文件转换为单个JSONL文件，用于大模型训练。

## 文件结构

数据集位于 `dataset/` 目录下，包含以下子文件夹：
- `T1/`: 50个JSON文件
- `T2/`: 12个JSON文件
- `T3/`: (空文件夹)

每个JSON文件包含多行，每行是一个独立的对话数据（包含system、user、assistant消息）。

## 使用方法

### 基本用法

```bash
# 在dataset_collector目录下运行
python3 convert_to_jsonl.py
```

### 自定义参数

```bash
# 指定数据集目录和输出文件
python3 convert_to_jsonl.py --dataset_dir /path/to/dataset --output my_dataset.jsonl
```

### 参数说明

- `--dataset_dir`: 数据集目录路径（默认: dataset）
- `--output`: 输出JSONL文件路径（默认: parking_dataset.jsonl）

## 输出格式

生成的JSONL文件每行包含一个完整的JSON对象，格式如下：

```json
{"messages": [{"role": "system", "content": "..."}, {"role": "user", "content": "..."}, {"role": "assistant", "content": "..."}]}
```

## 统计信息

- 总文件数: 62个 (T1: 50个, T2: 12个, T3: 0个)
- 成功处理: 62个文件
- 输出数据: 67行对话数据（某些文件包含多个对话）
- 输出文件: `parking_dataset.jsonl` (约750KB)

## 验证

可以使用以下命令验证JSONL文件：

```bash
# 检查行数
wc -l parking_dataset.jsonl

# 验证JSON格式
python3 -c "import json; count=0; [count:=count+1 for line in open('parking_dataset.jsonl') if json.loads(line.strip())]; print(f'All {count} lines are valid JSON')"
```

## 注意事项

- 脚本会自动跳过空的JSON行和解析错误的行
- 输出文件使用UTF-8编码，支持中文字符
- 如果目标文件夹不存在，脚本会跳过该文件夹
