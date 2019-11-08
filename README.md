
# 1.环境配置

## gpstk安装
mkdir build

cd build

cmake .. -DBUILD_EXT=ON

make

注意:在我电脑上总是遇到编译失败的情况， 后来把某个测试文件给注释掉就可以了
FFBinaryStream_T.cpp

## 编译example
mkdir build

cd build

cmake ..

make

# 2.案例学习
读取Rinex文件

## 2.1时间转换
系统时间 system time
通用时间 common time
民用时间 civil time
YDS时间 YDS time
GPS时间 gps time

## 2.2文件输入输出
读取一个文件然后输出到另一个文件

## 2.3
