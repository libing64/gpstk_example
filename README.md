
# 1.环境配置

## 1.1 GPSTK编译和安装 (Ubuntu18.04)

GPSTK目前依然在快速更新，试了下最新的版本，不是很稳定，而且example快被删光了，建议大家使用v2.11版本

```
git clone git@github.com:SGL-UT/GPSTk.git
cd GPSTK
git checkout v2.11 -b master_v2.11
mkdir build
cd build
cmake .. -DBUILD_EXT=ON
make
sudo make install
```

注意:在我电脑上总是遇到编译失败的情况， 一个test文件FFBinaryStream_T.cpp总是编译失败，在core/tests/FileHandling/CMakeLists.txt把这个文件注释掉就好了

```
-add_executable(FFBinaryStream_T FFBinaryStream_T.cpp)
-target_link_libraries(FFBinaryStream_T gpstk)
-add_test(FileHandling_FFBinaryStream FFBinaryStream_T)
+# add_executable(FFBinaryStream_T FFBinaryStream_T.cpp)
+# target_link_libraries(FFBinaryStream_T gpstk)
+# add_test(FileHandling_FFBinaryStream FFBinaryStream_T)
```

## 1.2 创建依赖GPSTK的工程

```
mkdir gpstk_example
cd gpstk_example
touch example1.cpp CMakeLists.txt
```
example1.cpp可以从GPSTK的例子中拷贝, CMakeLists.txt如下

```
cmake_minimum_required(VERSION 3.1)
project(example)

find_package(GPSTK)
include_directories(${GPSTK_INCLUDE_DIRS})

message(STATUS "gpstk include dir: ${GPSTK_INCLUDE_DIRS}")
message(STATUS "gpstk library dir: ${GPSTK_LIBRARY_DIRS}")

add_executable(example1 example1.cpp)
target_link_libraries(example1 gpstk)
```

编译代码
```
mkdir build
cd build
cmake ..
make
```

执行代码 
```
./example1
Hello world!
   The current civil time is 11/10/2019 14:36:07 UTC
   The current year is 2019
   The current day of year is 314
   The current second of day is 52567.2
   The current full GPS week is 2079
   The current short GPS week is 31
   The current day of GPS week is 0
   The current second of GPS week is 52567.2
   The current Modified Julian Date is 58797.608416953 UTC

```

example1.cpp 非常简单，几个时间之间的转换.

* 系统时间 system time
* 通用时间 common time
* 民用时间 civil time
* YDS时间 YDS time
* GPS时间 gps time


# 2. GPS数据存储和访问
GPSTK目前支持Rinex， Binex, sp3等格式，这里重点介绍下Rinex.
Rinex全称是'Receiver Independent Exchange Format'.

## 2.1 Rinex(2.0版本)文件类型
Rinex文件一ASCII码的方式存储, 主要分为4类，
* Observation Data File 观测数据文件
* Navigation Data File 导航数据文件(用于星历计算)
* Meteorological Data File 气象数据文件(用于时间补偿)
* GLONASS Navigation Message File 格罗纳斯导航数据文件

Rinex协议一直在更新扩展中， 格罗纳斯导航数据文件是1997年对Rinex格式进行扩展加上的
到了Rinex3.x版本又被精简为观测文件，导航文件和气象文件，格罗纳斯和北斗的观测统一合并到观测文件.

* Observation Data File 观测数据文件 
* Navigation Data File 导航数据文件(用于星历计算)
* Meteorological Data File 气象数据文件(用于时间补偿)

## 2.2 文件命名格式

文件命名格式 'ssssdddf.yyt'

例如 onsa2240.08o

* ssss 4字符基站名
* ddd  一年的第ddd天
* f    一天内的第f个文件
* yy   年份
* t    文件类型
  * o 观测文件
  * n 导航文件
  * m 气象文件
  * g 格罗纳斯导航文件

## 2.3 文件头和数据
Rinex文件分为文件头和数据两部分，文件头指明了基本信息，数据段是一条条的观测记录.

每条观测数据有指明观测类型，主要以下几种
* L1 L2 : 在L1/L2频段的相位测量
* C1    : C/A码在L1频段的伪距测量
* P1 P2 : P码在L1/L2频段的伪距测量
* D1 D2 : 在L1/L2频段的多普勒频率
* T1 T2 : 在150MHz(T1)和400MHz(T2)的传输集成多普勒

数据段的每天观测有记录历元时刻，历元标志，卫星ID等
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

* 08  5 27  0  0  0.0000000 : 历元时刻
* 0                         : 历元标志， 0:正常，1：异常
* 8G 6G21G29G30G31G 3G24G16 : 8代表有8个观测，后面是每个卫星的ID，前缀G代表GPS(G 3,代表G03？ 不带补0的？)
下面每行就是每个卫星的观测数据

## 2.4 观测类型 3字节
为了兼容更多观测类型 改为3字节
* t: observation type
  * C : pseudorange 
  * L : carrier phase
  * D : doppler
  * S : signal strengh
* n: band/frequency
  * 1,2,3,..8
* a: attribute
  * tracking mode or channel
# 3. 案例学习

## 3.1 example1.cpp
多种时间之间的转换
```
./example1
Hello world!
   The current civil time is 11/10/2019 14:36:07 UTC
   The current year is 2019
   The current day of year is 314
   The current second of day is 52567.2
   The current full GPS week is 2079
   The current short GPS week is 31
   The current day of GPS week is 0
   The current second of GPS week is 52567.2
   The current Modified Julian Date is 58797.608416953 UTC

```

## 3.2 example2.cpp
Rinex文件读取和写入
```
    // Create the input file stream
    Rinex3ObsStream rin("bahr1620.04o");

    // Create the output file stream
    Rinex3ObsStream rout("bahr1620.04o.new", ios::out | ios::trunc);

    // Read the RINEX header
    Rinex3ObsHeader head; //RINEX header object
    rin >> head;
    rout.header = rin.header;
    rout << rout.header;

    // Loop over all data epochs
    Rinex3ObsData data; //RINEX data object
    while (rin >> data)
    {
        rout << data;
    }
    return 0;
```

## 3.3 example3.cpp
Rinex文件读取
```
./example3 ../madr1480.08o
text 0:C1P is not stored in system G.
location 0:/home/xxx/source/gnss/GPSTk/core/lib/FileHandling/RINEX3/Rinex3ObsHeader.cpp:2476
```
这个错误是Rinex2映射到Rinex3的问题， 获取观测index的string变了, 而且不是一一映射.

* C2: C2C, C2S, C2L, C2X
* P2: C2P, C2W, C2Y
* L2: L2C, L2S, L2L, L2X, L2P, L2W, L2Y
* S2: S2C, S2S, S2L, S2X, S2P, S2W, S2Y
* D2: D2C, D2S, D2L, D2X, D2P, D2W, D2Y
* C1: C1C
* P1: C1P, C1W, C1Y
* L1: L1C, L1P, L1W, L1Y
* D1: D1C, D1P, D1W, D1Y
* S1: S1C, S1P, S1W, S1Y

获取index这里做下修改, 具体怎么映射的还没搞清楚.
```
    int indexP1( roh.getObsIndex( "P1" ) );
    int indexP2( roh.getObsIndex( "P2" ) );
    ==>
    int indexP1( roh.getObsIndex( "C1W" ) );
    int indexP2( roh.getObsIndex( "C2W" ) );


    dataobj = roe.getObs(prn, "L1", roh);
    ==>
    dataobj = roe.getObs(prn, "L1C", roh);
```
重新执行代码

```
./example3 ../madr1480.08o
...
05/27/2008 19:51:00 GPS    PRN 14 biased multipath -2.588
05/27/2008 19:51:30 GPS    PRN 14 biased multipath -2.237
05/27/2008 19:52:00 GPS    PRN 14 biased multipath -3.180
05/27/2008 19:52:30 GPS    PRN 14 biased multipath -2.592
05/27/2008 19:53:00 GPS    PRN 14 biased multipath -2.613
05/27/2008 19:53:30 GPS    PRN 14 biased multipath -2.500
05/27/2008 19:54:00 GPS    PRN 14 biased multipath -2.211
05/27/2008 19:54:30 GPS    PRN 14 biased multipath -3.501
05/27/2008 19:55:00 GPS    PRN 14 biased multipath -3.405
...
```

但是读取另外一个文件又不行了, 而且onsa2240.05o 后面example8.cpp进行PPP高精度定位还需要用.

```
./example3 ../onsa2240.05o 
Name your PRN of interest (by number: 1 through 32): 3
Reading ../onsa2240.05o.
text 0:Non-text data in file.
text 1:In record 0
text 2:In file ../onsa2240.05o
text 3:Near file line 17
location 0:/home/xxx/source/gnss/GPSTk/core/lib/FileHandling/FFTextStream.cpp:163
location 1:/home/xxx/source/gnss/GPSTk/core/lib/FileHandling/FFStream.cpp:225
location 2:/home/xxx/source/gnss/GPSTk/core/lib/FileHandling/FFStream.hpp:214
location 3:/home/xxx/source/gnss/GPSTk/core/lib/FileHandling/FFStream.hpp:214
```
尝试把onsa2240.05o中COMMENT 那段删除就好了，只是注释而已，也没啥影响 (来来回回发现GPSTK的bug并不少)

```
-MSWin2000|IAx86-PII|bcc32 5.0|MSWin95/98/NT/2000|486/DX+    COMMENT
-teqc  2002Mar14                         20050812 00:00:19UTCCOMMENT
-Forced Modulo Decimation to 30 seconds                      COMMENT
-IMoS GFileSrv 3.34  Lantm<E4>teriet        11.08.2005          COMMENT
-Edited by GPSTK Rinex Editor ver 3.5 6/21/2007 on 2008/03/26COMMENT
-Edited by GPSTK Rinex Editor ver 3.5 6/21/2007 on 2008/03/29COMMENT
```

## 3.4 example4.cpp
使用RAIM(Receiver Autonomous Integrity Monitoring)算法进行位置解算，同样需要修复example3.cpp中类似的读取问题.

```
./example4 ../bahr1620.04o ../bahr1620.04n ../bahr1620.04m
...
3633914.011569504626 4425279.107967041433 2799864.755831988994
3633916.685007087421 4425279.235846237279 2799864.304203432985
3633913.651396216359 4425276.569889308885 2799863.520161014516
3633914.779875373468 4425278.692695946433 2799863.891346025281
3633913.707529108506 4425278.749492554925 2799864.092652271967
3633915.993440963328 4425279.903057985939 2799864.321926375385
3633912.729556117672 4425278.029179973528 2799863.424704938196
3633915.355947569944 4425279.968269264325 2799862.789997876622
3633912.848960178904 4425278.554718625732 2799862.117798298132
3633913.617620099802 4425277.836539197713 2799863.085488767363
3633913.512186058797 4425278.970656199381 2799861.922919052653
3633914.016724057030 4425279.113330357708 2799863.271206538193
...

```


## 3.5 example5.cpp
使用high-level的gpstk库函数进行位置解算.
```
 ./example5 -i ../bahr1620.04o -n ../bahr1620.04n
```
读取文件的地方总是会报错， 把catch try那里删掉就好了， exception的函数有bug.



## 3.6 example6.cpp
示范一种最简单的处理GPS数据的方法

```
./example6
...
3420.00000000 3633913.78745681   4425278.15260870   2799863.52851997   50.60812377   26.20913716   -11.52921599   
3450.00000000 3633914.07529918   4425278.50379979   2799863.77856510   50.60812377   26.20913738   -11.01138913   
3480.00000000 3633914.47325779   4425278.62842339   2799863.45694815   50.60812149   26.20913338   -10.84043349   
3510.00000000 3633914.73389930   4425279.14054682   2799863.65079338   50.60812272   26.20913272   -10.25132938   
3540.00000000 3633913.86945560   4425278.16343019   2799862.64804251   50.60812320   26.20912979   -11.86388659   
3570.00000000 3633914.62948494   4425278.90517968   2799863.52814496   50.60812204   26.20913271   -10.52814381  
...
```

## 3.7 example7.cpp
示范了十几种处理GPS数据的方法


## 3.8 example8.cpp
高精度定位PPP: Precise Point Positioning, PPP算法参考Kouba, J. and P. Heroux. "Precise Point Positioning using IGS Orbit and Clock Products"
example3那里已经修复了onsa2240.05o文件读取失败的问题， 否则这里是不能运行的.

```
./example8
...
80100.000  -0.006  -0.010  -0.021  0.103  0.000  0.000  0.000  0.000  5  2.873  2.538  1.345  1.683  1.900  
81000.000  -0.006  -0.010  -0.021  0.107  0.000  0.000  0.000  0.000  5  3.087  2.723  1.452  1.778  2.063  
81900.000  -0.006  -0.010  -0.021  0.110  0.000  0.000  0.000  0.000  6  2.651  2.385  1.157  1.393  1.937  
82800.000  -0.006  -0.010  -0.020  0.106  0.000  0.000  0.000  0.000  7  2.664  2.362  1.233  1.363  1.929  
83700.000  -0.006  -0.010  -0.020  0.104  0.000  0.000  0.000  0.000  7  2.236  2.045  0.904  1.435  1.457  
84600.000  -0.006  -0.010  -0.019  0.107  0.000  0.000  0.000  0.000  7  3.325  2.898  1.631  1.785  2.283  
85500.000  -0.006  -0.010  -0.019  0.108  0.000  0.000  0.000  0.000  8  2.332  2.039  1.132  1.116  1.707  
Module of error vector: Average = 0.038922 m    Std. dev. = 0.00789807 m

```
PPP定位精度非常高, 平均误差0.038m，标准差0.0078m


## 3.9 example9.cpp
PPP高精度解算， 配置文件使用‘pppconfig.txt’

计算结果输出到onsa2240-05.out等文件.
```
./example9
... 
2004  237  7200.0000  0.0001  -0.0077  -0.0313  2.4959  0.0000  0.0000  0.0000  0.0000  10  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  8100.0000  0.0002  -0.0076  -0.0312  2.4955  0.0000  0.0000  0.0000  0.0000  9  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  9000.0000  0.0001  -0.0076  -0.0312  2.4938  0.0000  0.0000  0.0000  0.0000  8  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  9900.0000  0.0002  -0.0076  -0.0312  2.4969  0.0000  0.0000  0.0000  0.0000  8  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  10800.0000  0.0002  -0.0076  -0.0312  2.4950  0.0000  0.0000  0.0000  0.0000  9  2.0896  1.8811  0.9098  0.8344  1.6859  
```


## 3.10 example10.cpp
这个是example8的改进版， 参考paper "High accuracy positioning using carrier-phases with the open source GPSTk software".
计算的精度比example8还要高.

```
./example10
86220.0180  -0.0021  0.0043  0.0009  0.0439  0.0000  0.0000  0.0000  0.0000  7  2.3852  2.0629  1.1972  1.1253  1.7290  
86250.0180  -0.0021  0.0044  0.0009  0.0439  0.0000  0.0000  0.0000  0.0000  7  2.3846  2.0626  1.1966  1.1258  1.7282  
86280.0180  -0.0021  0.0044  0.0009  0.0425  0.0000  0.0000  0.0000  0.0000  7  2.3838  2.0621  1.1960  1.1263  1.7273  
86310.0180  -0.0021  0.0044  0.0009  0.0422  0.0000  0.0000  0.0000  0.0000  7  2.3830  2.0615  1.1952  1.1269  1.7263  
86340.0180  -0.0021  0.0044  0.0009  0.0401  0.0000  0.0000  0.0000  0.0000  7  2.3820  2.0609  1.1944  1.1274  1.7252  
86370.0180  -0.0021  0.0044  0.0009  0.0411  0.0000  0.0000  0.0000  0.0000  7  2.3809  2.0601  1.1935  1.1279  1.7239  
Module of error vector: Average = 0.00971589 m    Std. dev. = 0.00580035 m
```
平均误差0.009m，标准差0.0058m

## 3.11 example11.cpp
使用GUI显示数据

## 3.12 example12.cpp
自定义类型，可以自己扩展Rinex数据格式

## 3.13 example13.cpp
gpstk surface plot

## 3.14 example14.cpp

精确轨道定位
```
85800.00000 -0.00143 -0.00499 -0.00221 2.22699
85830.00000 -0.00055 -0.00988 -0.03134 2.22596
85860.00000 0.00566 -0.01188 -0.04113 2.22651
85890.00000 0.00045 -0.01005 -0.02067 2.22661
85920.00000 -0.00167 -0.01067 -0.02236 2.22557
85950.00000 0.00147 -0.01118 -0.01652 2.22466
85980.00000 0.01036 -0.00987 -0.01690 2.22450
86010.00000 0.00245 -0.00378 -0.01040 2.22365
86040.00000 0.00101 -0.00929 -0.01248 2.22249
86070.00000 0.00391 -0.00855 -0.00417 2.22181
86100.00000 0.00526 -0.00941 -0.01892 2.22047
86130.00000 0.00896 -0.01308 0.00882 2.21954
86160.00000 0.01072 -0.01710 -0.00525 2.21851
86190.00000 0.00023 -0.01064 -0.00553 2.21612
86220.00000 0.00752 -0.01103 0.01698 2.21565
86250.00000 0.01035 -0.02318 0.01166 2.21482
86280.00000 0.00819 -0.01150 0.01581 2.21356
86310.00000 0.01652 -0.01485 0.00583 2.21372
86340.00000 0.01647 -0.00995 0.02512 2.21342
86370.00000 0.01247 -0.01339 0.01017 2.21314
```


## 3.15 example15.cpp
鲁棒统计以及随机数生成器

```
./example15

Before perturbation: sample mean is               10.0586, 
                     sample standard deviation is 1.99072

Altering measurement 220 to take the value of 10000

After perturbation:  sample mean is                20.0476, 
                     sample standard deviation is 315.916

Robust statistics:
                     number    = 1000
                     quartiles =   8.6755863   11.334482
                     median    =   10.125088
                     MAD       =   1.9174815

Using robust stats:  sample mean is                10.0577, 
                     sample standard deviation is 1.9915078

```


## 3.16 example16.cpp 
编译不过

## 3.17 example17.cpp
编译不过

## example18.cpp
这个例子不是真实GPS数据，使用卡尔曼滤波算法解决通用方程和通用约束的问题.
这个是PPP-RTK算法中的核心问题，未来会用于RTK和PPP-RTK之中.

## 3.19 navfilterex.cpp
这个需要手动输入数据，不清楚要怎么跑

