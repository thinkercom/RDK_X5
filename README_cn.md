# 🤖 基于RDK X5的农业巡检机器人及智慧终端

## 📖 项目简介

本项目聚焦于农业场景下的智能巡检与自动化作业，基于地平线RDK X5平台，集成了目标检测（YOLO11）、多设备联动、Web可视化、底盘/设备嵌入式控制等多项功能。
系统可实现作物/果实等目标的自动识别、精准定位与联动控制（如水泵💧、滑台🛤️、底盘🚜），并通过网页端进行远程管理与监控。
适用于智能农业🌱、温室自动化、田间巡检等场景，具备良好的扩展性和二次开发基础。

---

## 🗂️ 项目结构导航

```
Project20250627/
├── app5.py                # 单文件全集成版（快速体验/调试用）
├── run_in_here/           # 推荐的模块化运行目录（主力开发/部署用）
├── models/                # AI模型文件（如Chili.bin等）
├── serial/                # 串口动态库等（如libserialcomm.so）
├── templates/             # 全局HTML模板
├── imgs/                  # 全局图片保存目录
├── test_yolov11/          # YOLOv11模型测试脚本与数据
├── others/                # 其它辅助脚本或工具
├── history/               # 代码历史归档
├── stm32.zip              # 机器人底盘/设备控制嵌入式固件（压缩包，需解压后用Keil5打开）
└── README.md              # 项目总览说明（本文件）
```

---

## 🌟 项目亮点

- **软硬件一体化**：集成摄像头、地平线AI芯片、串口外设、嵌入式底盘，软硬件协同
- **多场景适配**：支持多摄像头、滑台/水泵/底盘等多种设备，适应不同农业应用
- **Web可视化**：网页端实时视频流、设备状态、远程控制，操作直观
- **模块化设计**：`run_in_here/` 目录下多文件分工，嵌入式固件以zip包形式提供，便于维护和二次开发
- **可扩展性强**：支持自定义目标检测模型、设备类型、业务逻辑扩展
- **丰富的日志与图片归档**：便于溯源、调试和数据分析

---

## 🚩 快速上手

1. **推荐使用 `run_in_here/` 目录下的多文件结构进行开发和部署。**
2. **详细功能、依赖、启动方法请参见 `run_in_here/README.md`。**
3. **如需快速体验，可直接运行 `app5.py` 单文件版。**
4. **如需底盘/设备控制固件开发：**
   - 解压 `stm32.zip` 到本地目录。
   - 用 Keil5（MDK-ARM 5.x）打开解压后的工程文件（如 `USER/dlrobot.uvprojx`）。
   - 按照 `stm32` 文件夹内的说明进行编译、下载和调试。

---

## 📦 各子目录简要说明

- **run_in_here/**  
  主力开发与部署目录，所有功能模块化拆分，详见其内 `README.md`。

- **models/**  
  存放AI模型文件（如YOLO11的bin文件）。

- **serial/**  
  存放串口通信相关的动态库和配置。

- **templates/**  
  全局HTML模板，Web端页面资源。

- **imgs/**  
  全局图片归档目录，自动生成。

- **test_yolov11/**  
  YOLOv11模型的独立测试与验证脚本。

- **others/**  
  其它辅助脚本、工具或临时文件。

- **history/**  
  代码历史版本归档，便于追溯和对比。

- **stm32.zip**  
  机器人底盘/设备控制的嵌入式固件工程（压缩包，需解压后用Keil5打开，包含平衡控制、外设驱动、FreeRTOS等）。

---

## 🧭 推荐开发/部署流程

1. 进入 `run_in_here/` 目录，按其 `README.md` 配置依赖、模型、动态库等。
2. 启动主服务：  
   ```bash
   cd run_in_here
   python main.py
   ```
3. 浏览器访问 `http://<设备IP>:5000`，体验Web端管理与控制。
4. 如需底盘/设备控制固件开发，先解压 `stm32.zip`，再用 Keil5 打开工程文件进行编译和下载。

---

## 🛠️ 适用与扩展建议

- **适用场景**：温室巡检、果蔬采摘、田间自动化、分拣流水线、底盘运动控制等。
- **扩展建议**：可集成更多传感器、机械臂、云端数据分析等模块，打造更完整的智慧农业解决方案。

---

## ❓ 常见问题

- **功能/依赖/启动等细节问题**  
  请优先查阅 `run_in_here/README.md` 和解压后的 `stm32/README.md`，这两份文档更详细。

- **模型/动态库/硬件路径问题**  
  请根据实际环境修改相关配置，确保文件路径、串口号等正确。

- **嵌入式固件无法编译/下载**  
  请确认已用 Keil5 解压并打开工程，相关外设库和驱动已配置好。

---

## 📬 联系与支持

如需定制开发、技术支持或有任何建议，欢迎联系项目维护者。

---

**🌱 让AI与嵌入式赋能农业，让自动化提升效率！**  
**✨ Enjoy your smart agri-inspection & control system! ✨** 
