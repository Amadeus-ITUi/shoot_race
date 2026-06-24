# Moonshot 任务板视觉识别

程序在 shoot3 面向任务板时，从 `/camera/image` 获取一帧图片，调用
Moonshot `kimi-k2.6` 视觉模型，并将识别出的物品映射到移动靶区域：

| 物品 | 区域 |
|---|---:|
| `ak47`、`helmet`、`pack` | 1 |
| `aid`、`gauze`、`iv` | 2 |
| `mag`、`box`、`belt` | 3 |

## 配置

`run_reference.sh` 会自动加载 `reference/.env`。手动运行主程序时先加载：

```bash
cd ~/shoot_race/reference
source .env
python3 shoot_race_shoot1_only.py
```

可选参数：

```bash
export MOONSHOT_VISION_MODEL="kimi-k2.6"
```

不要将密钥写入 Python 文件、启动脚本或 Git 仓库。已经在聊天、终端历史或
其他公开位置出现过的密钥应立即撤销并重新生成。

## 返回格式与容错

模型被要求严格返回：

```json
{"object_name":"helmet","confidence":0.95,"description":"军用头盔"}
```

程序会再次进行本地校验：

- 顶层必须是 JSON 对象；
- 只能包含 `object_name`、`confidence`、`description`；
- `object_name` 必须是九个规范名称之一；
- `confidence` 必须位于 0 到 1；
- 区域由本地映射计算，不采用模型返回的区域号。

请求失败、返回格式错误、未知类别或置信度低于 `0.55` 时会自动重试三次；
最终失败则回退到区域 2，保证流程继续执行。

## 测试

```bash
cd ~/shoot_race/reference
source ~/shoot_race/devel/setup.bash
python3 -m unittest -v test_moonshot_vision.py
```
