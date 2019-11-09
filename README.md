
# 1.环境配置

## gpstk安装
```
mkdir build
cd build
cmake .. -DBUILD_EXT=ON
make
```

注意:在我电脑上总是遇到编译失败的情况， 后来把某个测试文件给注释掉就可以了
FFBinaryStream_T.cpp

## 编译example
```
mkdir build
cd build
cmake ..
make
```

# 2.案例学习

## 2.1时间转换 example1
系统时间 system time

通用时间 common time

民用时间 civil time

YDS时间 YDS time

GPS时间 gps time

## 2.2文件输入输出 example2
读取一个文件然后输出到另一个文件


## 3.Rinex数据格式
file types:

1. Observation Data File
2. Navigation Data File
3. Meteorological Data File
4. GLONASS Navigation Message File

每个文件分成2部分
1. header section
2. data section


## GPS observables
1. time  时间
2. psedu-range 伪距
3. phase 相位
4. Doppler 多普勒 -->(方向定义: 朝向卫星为正)

## Rinex version features
1. satellite numbers 卫星个数

## 文件头的记录顺序
顺序无关，但是有部分例外
1. "RINEX VERSION / TYPE"放在最前面
2. "WAVELENGTH FACT L1/2"如果存在，需要放在为单个卫星定义wavelength factor的记录之前
3. "# OF SATELLITES"如果存在，后面需要紧跟"PRN / # OF OBS"

## 缺失项

## 数据段
```
 08  5 27  0  0  0.0000000  0  8G 6G21G29G30G31G 3G24G16
 114882249.33248  89518626.24848  21861373.4054   21861376.2644   21861373.5894
       238.000         204.000
 110254184.33348  85912345.81348  20980679.0014   20980680.5424   20980679.8634
       249.000         223.000
 120713732.89848  94062638.76348  22971066.7834   22971069.3844   22971066.3794
       231.000         194.000
 127596858.63848  99426108.12048  24280886.5204   24280890.7184   24280887.1834
       218.000         159.000
 113742340.75548  88630389.25948  21644452.3124   21644453.7974   21644451.9784
       245.000         216.000
 122838795.24448  95718531.73348  23375451.5884   23375454.5834   23375451.0024
       216.000         162.000
 117616989.64848  91649585.97448  22381782.3054   22381787.0184   22381782.7914
       238.000         201.000
 111208337.76148  86655834.69148  21162252.8314   21162255.1284   21162253.5234
       248.000         220.000
```

08  5 27  0  0  0.0000000 历元时刻
0 历元标志， 0:正常，1：异常
8G 6G21G29G30G31G 3G24G16 8代表有8个观测，后面是每个卫星的编号，前缀G代表GPS(G 3,代表G03？ 不带补0的？)
下面每行就是每个卫星的观测数据

卫星的位置怎么来的？
根据历元时刻+Navigation msg计算出来


# 4.如果根据导航信息计算卫星位置？


## example3.cpp
```
LINE: 89
text 0:C1P is not stored in system G.
location 0:/home/libing/source/gnss/GPSTk/core/lib/FileHandling/RINEX3/Rinex3ObsHeader.cpp:2476
```
这段错误码啥意思？是文件格式不对吗？

## example4.cpp
```
./example4 ../bahr1620.04o ../bahr1620.04n ../bahr1620.04m
The observation file doesn't have P1 pseudoranges.
```
为什么example4也不能运行？

## Rinex2 到Rinex3的映射? 这么应该怎么取？
```
The difficulty is that the mapping from RIN2 to RIN3 is not one-to-one:

C2: C2C, C2S, C2L, C2X
P2: C2P, C2W, C2Y
L2: L2C, L2S, L2L, L2X, L2P, L2W, L2Y
S2: S2C, S2S, S2L, S2X, S2P, S2W, S2Y
D2: D2C, D2S, D2L, D2X, D2P, D2W, D2Y

C1: C1C
P1: C1P, C1W, C1Y
L1: L1C, L1P, L1W, L1Y
D1: D1C, D1P, D1W, D1Y
S1: S1C, S1P, S1W, S1Y
```

如果根据三个文件估计接收机的位置?
```
               // In order to compute positions we need the current time, the
               // vector of visible satellites, the vector of corresponding
               // ranges, the object containing satellite ephemerides, and a
               // pointer to the tropospheric model to be applied
            raimSolver.RAIMCompute( rod.time,
                                    prnVec,
                                    rangeVec,
                                    bcestore,
                                    tropModelPtr );
```


## example5.cpp
```
 ./example5 -i ../bahr1620.04o -n ../bahr1620.04n
```
读取文件的地方总是会报错， 把catch try那里删掉就好了
不知道这里报exception是啥意思？