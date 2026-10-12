# 高低工位托座

由 `scripts/export_task_fixtures.py` 按 `config/tasks/higher.json`、`lower.json` 导出，复用 `cad/build.py` 的托座建模函数。STEP/STL 的长度单位为毫米。

| 目录 | 拾取托座总高 / mm | 放置托座总高 / mm |
|---|---:|---:|
| `higher/` | 55 | 85 |
| `lower/` | 55 | 35 |

底板为 60 × 45 × 6 mm，顶部支承面 24 × 14 mm，四个 Ø4.5 mm 通孔，孔距 44 × 29 mm。两工位共用孔位，调整的是立柱长度；默认的 55 mm 托座也保留在 `cad/generated/`。

各目录的 `metrics.json` 记录高度、实体体积和有效性。支承点在世界坐标系中的位置由任务配置设置；零件文件本身以底板中心为原点、底面 z = 0，不含工位安装位移。强度、加工及实物装配尚未验证。
