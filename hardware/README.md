> 状态：现行交付快照｜2026-10-23｜未完成整车标定

# 结构与接线参考源件

本目录是原文件只读快照，未修改CAD/图纸几何，也未在本次发布中重新做SolidWorks验收。

| 文件 | 来源与适用边界 |
|---|---|
| [baseplate.SLDPRT](reference/baseplate.SLDPRT) | 本地阿克曼底板源件；不是“已确认最新实车底板”的声明。当前实车结构与历史300×200等参考几何须另核，不直接据此加工 |
| [component-layout.svg](reference/component-layout.svg) | 历史元件布局；不能自动套用当前双520/新板/供电走线 |
| [system-flow-v2.5.svg](reference/system-flow-v2.5.svg) | v2.5并联一拖二系统结构参考；主控供电后来改D057方案，当前以接线说明为准 |
| [v2.6-original-reference.xlsx](wiring/v2.6-original-reference.xlsx) | 原58编号接线表存档；左右XSH与C2/C3是交换前规划，当前映射已更正 |

**施工与测试先读[当前接线](../docs/WIRING.md)**。原图/表中的未核额定、旧GPIO顺序、旧元件尺寸不因上传而成为验收结论。保留源件是为了后续复核和设计溯源。
