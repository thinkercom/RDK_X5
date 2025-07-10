CXX = g++
CXXFLAGS = -Wall -fPIC -std=c++11

# 源文件和目标文件
SRCS = control.cpp WzSerialportPlus.cpp
OBJS = $(SRCS:.cpp=.o)

# 动态库名称
LIB_NAME = libserialcomm.so

all: $(LIB_NAME)

$(LIB_NAME): $(OBJS)
	$(CXX) -shared -o $@ $^ -lpthread -lrt

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(LIB_NAME)



# # 在RDKX5设备上直接编译的设置
# TARGET = rdkx5_app
# CC = g++
# CFLAGS = -O2 -g -Wall -Wextra -std=c++17 -flto -pthread
# LDFLAGS = -pthread -lrt -ldl -latomic

# # 自动包含当前目录和所有子目录
# INCLUDEPATH = $(addprefix -I,$(ALL_DIRS))

# # 构建目录
# BUILD_PATH = build
# LOG_FILE = build.log

# # 排除的目录
# EXCLUDE_DIRS = ./.git ./.svn ./build
# EXCLUDE_DIRS := $(foreach dir,$(EXCLUDE_DIRS),-path $(dir) -prune -o)

# # 查找源文件和目录
# ALL_DIRS := $(shell find . -maxdepth 8 $(EXCLUDE_DIRS) -type d -print)
# SOURCE_FILES = $(foreach dir,$(ALL_DIRS),$(wildcard $(dir)/*.cpp))
# OBJECT_FILES = $(notdir $(patsubst %.cpp,%.o, $(SOURCE_FILES)))
# HEDAER_DEPENDS = $(patsubst %o,%d,$(OBJECT_FILES))
# HEDAER_DEPENDS := $(addprefix $(BUILD_PATH)/,$(HEDAER_DEPENDS))

# # VPATH update
# VPATH += $(ALL_DIRS)

# # .PHONY声明
# .PHONY: all pre clean install run debug configure help

# # 默认目标
# all: pre compile link
# 	@echo "编译完成！ 输出: $(BUILD_PATH)/$(TARGET)"

# # 链接目标
# link: $(TARGET)
# $(TARGET): compile
# 	@echo "正在链接可执行文件..."
# 	@$(CC) $(BUILD_PATH)/*.o -o $(BUILD_PATH)/$(TARGET) $(LDFLAGS) > $(LOG_FILE) 2>&1 || (cat $(LOG_FILE); exit 1)

# # 编译
# compile: $(OBJECT_FILES) $(HEDAER_DEPENDS)
# 	@echo "编译完成，共 $(words $(OBJECT_FILES)) 个源文件"

# $(OBJECT_FILES): %.o : %.cpp %.d
# 	@echo "编译: $<"
# 	@$(CC) -c $< -o $(BUILD_PATH)/$@ $(INCLUDEPATH) $(CFLAGS) >> $(LOG_FILE) 2>&1 || (cat $(LOG_FILE); exit 1)

# # 头文件依赖处理
# $(HEDAER_DEPENDS): pre
# -include $(HEDAER_DEPENDS)
# %.d: %.cpp
# 	@set -e;
# 	@mkdir -p $(BUILD_PATH);
# 	@echo "生成依赖: $<"
# 	@$(CC) $(CFLAGS) $(INCLUDEPATH) -MM $< | \
# 		sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' > $(BUILD_PATH)/$@ || true

# # 准备工作
# pre:
# 	@if [ ! -d "$(BUILD_PATH)" ]; then \
# 		echo "创建构建目录: $(BUILD_PATH)"; \
# 		mkdir -p $(BUILD_PATH); \
# 	fi
# 	@> $(LOG_FILE)  # 清空日志文件

# # 配置脚本（可选）
# configure:
# 	@./configure --prefix=/usr/local

# # 运行程序
# run: all
# 	@echo "正在运行程序..."
# 	@$(BUILD_PATH)/$(TARGET) || true

# # 调试运行
# debug: all
# 	@echo "正在调试模式运行程序..."
# 	@gdb -ex run --args $(BUILD_PATH)/$(TARGET)

# # 安装到系统
# install: all
# 	@echo "正在安装到系统目录..."
# 	@sudo cp $(BUILD_PATH)/$(TARGET) /usr/local/bin/$(TARGET)
# 	@sudo chmod 755 /usr/local/bin/$(TARGET)
# 	@echo "安装完成！输入 '$(TARGET)' 运行程序"

# # 清理
# clean:
# 	@rm -rf $(BUILD_PATH) $(LOG_FILE)
# 	@echo "清理完成."

# # 显示帮助
# help:
# 	@echo "RDKX5 本地编译 Makefile"
# 	@echo "可用目标:"
# 	@echo "  all       - 默认目标, 构建程序 (默认)"
# 	@echo "  run       - 构建并运行程序"
# 	@echo "  debug     - 构建并在GDB中运行程序"
# 	@echo "  install   - 安装到系统目录 (/usr/local/bin)"
# 	@echo "  clean     - 清理构建文件"
# 	@echo "  help      - 显示此帮助信息"
