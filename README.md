
# 1.环境配置

## gpstk安装

```
mkdir build
cd build
cmake .. -DBUILD_EXT=ON
make
sudo make install
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

1. Observation Data File 卫星观测数据,
2. Navigation Data File 卫星导航数据(星历计算)
3. Meteorological Data File(气象信息, 延时补偿?)
4. GLONASS Navigation Message File(格罗纳斯导航信息)
5. BDS Navigation Message File 北斗导航信息

每个文件分成2部分
1. header section
2. data section


## GPS observables
1. L1 L2 phase measurement on L1/L2
2. C1 Pseudorange using C/A code on L1
3. P1 P2: Pseudorange using P-code on L1/L2
4. D1 D2: Doppler frequency on L1/L2
5. T1 T2: transit Integrated Doppler on 150(T1) and 400MHz(T2)

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



```
删除onsa2240.05o中的注释??
 ./examples/example3 ../examples/onsa2240.05o

....
Name your PRN of interest (by number: 1 through 32): 3
Reading ../examples/onsa2240.05o.
---------------------------------- REQUIRED ----------------------------------
Rinex Version  2.10,  File type Observation,  System MIXED.
Prgm: teqc  2002Mar14,  Run: 03/29/2008 23:03:54,  By: 
Marker type: .
Observer : SWEPOS,  Agency: LMV
Rec#: AGOMET9N0N4,  Type: JPS E_GGD,  Vers: 2.3 Feb,17,2004 p4b2
Antenna # : 020,  Type : AOAD/M_B        OSOD
Position      (XYZ,m) : (3370658.8318, 711876.9387, 5349786.7450).
Antenna Delta (HEN,m) : (0.9950, 0.0000, 0.0000).
Galileo Observation types (3):
 Type #01 (C1B) L1 GALB pseudorange
 Type #02 (L1B) L1 GALB phase
 Type #03 (S1B) L1 GALB snr
GPS Observation types (7):
 Type #01 (C1C) L1 GPSC/A pseudorange
 Type #02 (L1C) L1 GPSC/A phase
 Type #03 (L2W) L2 GPScodelessZ phase
 Type #04 (C1W) L1 GPScodelessZ pseudorange
 Type #05 (C2W) L2 GPScodelessZ pseudorange
 Type #06 (S1C) L1 GPSC/A snr
 Type #07 (S2W) L2 GPScodelessZ snr
GLONASS Observation types (7):
 Type #01 (C1C) G1 GLOC/A pseudorange
 Type #02 (L1C) G1 GLOC/A phase
 Type #03 (L2C) G2 GLOC/A phase
 Type #04 (C1P) G1 GLOP pseudorange
 Type #05 (C2P) G2 GLOP pseudorange
 Type #06 (S1C) G1 GLOC/A snr
 Type #07 (S2C) G2 GLOC/A snr
Geosync Observation types (3):
 Type #01 (C1C) L1 SBASC/A pseudorange
 Type #02 (L1C) L1 SBASC/A phase
 Type #03 (S1C) L1 SBASC/A snr
R2ObsTypes: C1 L1 L2 P1 P2 S1 S2 
mapSysR2toR3ObsID[E] C1:C1B L1:L1B S1:S1B 
mapSysR2toR3ObsID[G] C1:C1C L1:L1C L2:L2W P1:C1W P2:C2W S1:S1C S2:S2W 
mapSysR2toR3ObsID[R] C1:C1C L1:L1C L2:L2C P1:C1P P2:C2P S1:S1C S2:S2C 
mapSysR2toR3ObsID[S] C1:C1C L1:L1C S1:S1C 
Time of first obs 2005/08/12 00:00:00.000 GPS
(This header is VALID)
---------------------------------- OPTIONAL ----------------------------------
Marker number : 10402M004
Signal Strenth Unit = 
Interval =  30.000
Wavelength factor L1: 1 L2: 1
Leap seconds: 13
-------------------------------- END OF HEADER --------------------------------
text 0:C1P is not stored in system G.
location 0:/home/libing/source/gnss/GPSTk/core/lib/FileHandling/RINEX3/Rinex3ObsHeader.cpp:2476
```

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
example5跟example4比较接近？ 根据星历计算卫星位置？ 然后根据伪距计算接收机位置

## example6.cpp
示范一种最简单的处理GPS数据的方法


```
./example6
WARNING: Navigation file bahr1620.04n doesn't have valid ionospheric correction parameters.
Exception at epoch: 2453167 00000000 0.000000000000000 GPS
0.00000000 3633909.10160000   4425275.50330000   2799861.27360000   50.60814318   26.20913892   -17.03005082   
Exception at epoch: 2453167 00030000 0.000000000000000 GPS
30.00000000 3633909.10160000   4425275.50330000   2799861.27360000   50.60814318   26.20913892   -17.03005082   
Exception at epoch: 2453167 00060000 0.000000000000000 GPS
60.00000000 3633909.10160000   4425275.50330000   2799861.27360000   50.60814318   26.20913892   -17.03005082   
Exception at epoch: 2453167 00090000 0.000000000000000 GPS
90.00000000 3633909.10160000   4425275.50330000   2799861.27360000   50.60814318   26.20913892   -17.03005082   
Exception at epoch: 2453167 00120000 0.000000000000000 GPS
120.00000000 3633909.10160000   4425275.50330000   2799861.27360000   50.60814318   26.20913892   -17.03005082   
Exception at epoch: 2453167 00150000 0.000000000000000 GPS
150.00000000 3633909.10160000   4425275.50330000   2799861.27360000   50.60814318   26.20913892   -17.03005082   
Exception at epoch: 2453167 00180000 0.000000000000000 GPS
180.00000000 3633909.10160000   4425275.50330000   2799861.27360000   50.60814318   26.20913892   -17.03005082   
Exception at epoch: 2453167 00210000 0.000000000000000 GPS
210.00000000 3633909.10160000   4425275.50330000   2799861.27360000   50.60814318   26.20913892   -17.03005082   
Exception at epoch: 2453167 00240000 0.000000000000000 GPS

```

## example7.cpp
示范了十几种处理GPS数据的方法 
多种方法之间的区别在哪里？精度有差别吗？
```
95  -0.036  0.001  3.050  0.221  3.771  2.650  0.290  3.447  2.614  0.082  0.006  1.384  0.626  0.845  1.230  -0.046  0.058  0.999  0.048  0.075  0.476  0.121  0.216  
50130.011  3.803  -0.214  0.874  3.803  -0.214  0.874  3.819  -0.181  0.829  3.454  0.011  0.500  3.489  -0.034  0.001  2.226  -0.089  3.624  2.540  0.313  3.308  2.770  0.011  0.006  1.386  0.623  0.856  0.704  0.131  0.256  0.596  0.210  0.296  0.476  0.122  0.216  
50160.011  3.903  -0.417  0.665  3.903  -0.417  0.665  3.909  -0.401  0.597  3.455  -0.025  0.493  3.491  -0.070  0.001  2.409  1.079  3.322  2.524  0.315  3.284  2.763  0.015  0.006  1.388  0.621  0.866  0.905  -0.067  0.176  0.720  -0.002  0.147  0.477  0.121  0.215  
50190.011  3.617  -0.476  0.398  3.617  -0.476  0.398  3.787  -0.448  0.490  3.441  -0.062  0.509  3.479  -0.108  0.001  3.535  0.573  3.592  2.632  0.271  3.530  2.902  -0.053  0.007  1.391  0.619  0.877  0.530  -0.063  -0.104  0.598  -0.024  0.104  0.477  0.121  0.215  
50220.011  4.186  -0.453  0.650  4.186  -0.453  0.650  4.147  -0.388  0.567  3.477  -0.089  0.526  3.520  -0.138  0.001  2.545  0.471  1.205  2.494  0.296  3.289  2.758  -0.008  0.006  1.393  0.617  0.888  0.979  0.000  0.249  0.848  0.069  0.205  0.478  0.121  0.215  
50250.011  3.748  0.044  0.934  3.748  0.044  0.934  3.783  0.018  0.802  3.467  -0.055  0.575  3.515  -0.108  0.001  1.598  1.268  3.742  2.334  0.349  3.065  2.591  0.064  0.006  1.395  0.615  0.897  0.614  0.498  0.766  0.475  0.473  0.589  0.478  0.122  0.216  
50280.011  3.713  -0.271  0.447  3.713  -0.271  0.447  3.829  -0.260  0.486  3.474  -0.045  0.610  3.527  -0.101  0.001  2.538  0.585  2.766  2.289  0.380  3.022  2.554  0.098  0.006  1.396  0.614  0.906  0.472  0.270  0.232  0.472  0.266  0.297  0.478  0.122  0.216  
50310.011  3.888  -0.469  0.524  3.888  -0.469  0.524  3.953  -0.401  0.581  3.498  -0.052  0.677  3.560  -0.115  0.001  2.736  0.048  1.879  2.216  0.418  2.960  2.487  0.141  0.006  1.398  0.612  0.915  0.837  0.041  0.478  0.608  0.129  0.458  0.478  0.122  0.217  
50340.011  3.550  -0.154  0.940  3.550  -0.154  0.940  3.844  -0.157  0.914  3.503  -0.038  0.752  3.575  -0.108  0.001  3.236  0.303  3.171  2.209  0.430  2.992  2.494  0.148  0.006  1.399  0.611  0.923  0.690  0.299  0.964  0.611  0.335  0.886  0.479  0.123  0.219 
```

## example8.cpp
高精度定位PPP: Precise Point Positioning
这个例子很诡异，在GPSTK下面编译就是可以运行的，但是拷贝到gpstk_example目录下编译就不行？
是哪里安装的方式不对吗? include路径或则lib路径会不一样吗？

```
This program shows how to use GNSS Data Structures (GDS) to obtain
"Precise Point Positioning" (PPP).

For details on the PPP algorithm please consult:

    Kouba, J. and P. Heroux. "Precise Point Positioning using IGS Orbit
       and Clock Products". GPS Solutions, vol 5, pp 2-28. October, 2001.
```
没看懂在干嘛？ 关键位置为何没运行？
这个地方代码有bug, 读取Rinex文件总是失败，可以用example3读取试下 
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

没找到问题， 但是把obs文件的那几行注释文件删掉就可以运行了


正确运行的结果：
```
1.779  
78300.000  -0.006  -0.010  -0.022  0.107  0.000  0.000  0.000  0.000  4  7.984  7.442  2.891  5.710  4.773  
79200.000  -0.006  -0.010  -0.022  0.102  0.000  0.000  0.000  0.000  5  3.845  3.404  1.786  1.886  2.835  
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


## example9
PPP高精度解算， 配置文件使用‘pppconfig.txt’
```
This program reads GPS receiver data from a configuration file and\n"
"process such data applying a 'Precise Point Positioning' strategy.\n\n"
"Please consult the default configuration file, 'pppconf.txt', for\n"
"further details
```

计算结果
```
2004  237  0.0000  0.0002  -0.0077  -0.0313  2.4877  0.0000  0.0000  0.0000  0.0000  9  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  900.0000  0.0002  -0.0077  -0.0313  2.4877  0.0000  0.0000  0.0000  0.0001  8  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  1800.0000  0.0002  -0.0077  -0.0313  2.4868  0.0000  0.0000  0.0000  0.0001  8  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  2700.0000  0.0002  -0.0077  -0.0313  2.4924  0.0000  0.0000  0.0000  0.0000  8  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  3600.0000  0.0002  -0.0077  -0.0313  2.4914  0.0000  0.0000  0.0000  0.0001  7  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  4500.0000  0.0002  -0.0077  -0.0313  2.4896  0.0000  0.0000  0.0000  0.0001  7  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  5400.0000  0.0001  -0.0077  -0.0313  2.4914  0.0000  0.0000  0.0000  0.0001  8  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  6300.0000  0.0001  -0.0076  -0.0312  2.4912  0.0000  0.0000  0.0000  0.0000  9  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  7200.0000  0.0001  -0.0077  -0.0313  2.4959  0.0000  0.0000  0.0000  0.0000  10  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  8100.0000  0.0002  -0.0076  -0.0312  2.4955  0.0000  0.0000  0.0000  0.0000  9  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  9000.0000  0.0001  -0.0076  -0.0312  2.4938  0.0000  0.0000  0.0000  0.0000  8  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  9900.0000  0.0002  -0.0076  -0.0312  2.4969  0.0000  0.0000  0.0000  0.0000  8  2.0896  1.8811  0.9098  0.8344  1.6859  
2004  237  10800.0000  0.0002  -0.0076  -0.0312  2.4950  0.0000  0.0000  0.0000  0.0000  9  2.0896  1.8811  0.9098  0.8344  1.6859  
```


## example10.cpp

使用卡尔曼滤波的方式求解方程？
```
86220.0180  -0.0021  0.0043  0.0009  0.0439  0.0000  0.0000  0.0000  0.0000  7  2.3852  2.0629  1.1972  1.1253  1.7290  
86250.0180  -0.0021  0.0044  0.0009  0.0439  0.0000  0.0000  0.0000  0.0000  7  2.3846  2.0626  1.1966  1.1258  1.7282  
86280.0180  -0.0021  0.0044  0.0009  0.0425  0.0000  0.0000  0.0000  0.0000  7  2.3838  2.0621  1.1960  1.1263  1.7273  
86310.0180  -0.0021  0.0044  0.0009  0.0422  0.0000  0.0000  0.0000  0.0000  7  2.3830  2.0615  1.1952  1.1269  1.7263  
86340.0180  -0.0021  0.0044  0.0009  0.0401  0.0000  0.0000  0.0000  0.0000  7  2.3820  2.0609  1.1944  1.1274  1.7252  
86370.0180  -0.0021  0.0044  0.0009  0.0411  0.0000  0.0000  0.0000  0.0000  7  2.3809  2.0601  1.1935  1.1279  1.7239  
Module of error vector: Average = 0.00971589 m    Std. dev. = 0.00580035 m

```

## example11.cpp
添加GUI显示数据

## example12.cpp
自定义类型，可以自己扩展Rinex数据格式
```
  This is a example program to demonstrate some of the functionality of the
  ObsID class. The intent is to use ObsID as a key in a STL map of gps data. ObsID
  supports identifying the data in a manner that is similiar but can extend
  the Rinex 3 specification.
```

## example13.cpp
gpstk surface plot

## example14.cpp
精密轨道定位？
```
"\nThis program reads GPS receiver data from a configuration file and\n"
"process such data applying a 'Precise Orbits Positioning' strategy.\n\n"
```

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
## example15.cpp

```
// An example of robust statistics found in lib/geomatics
// compute Robust statistics. Also demonstrate the use
// of random number generators.
```

生成随机数
```
libing@libing:~/source/gnss/gpstk_example/build$ ./example15

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


## example16.cpp  example17.cpp
编译不过 先不管

## example18.cpp
这个例子不是真实GPS数据，
模糊整周数问题？

```
// Example program Nro 18 for GPSTk
//
// This program shows how to use 'GeneralEquations' and 'GeneralConstraint' 
// together with 'SolverGeneral' solve complex problems with Kalman filter. 
//
// To show the outline of the processing framework clearly, this example won't 
// process some real GNSS data, but try to solver the following mathematical 
// problem(the true value of x1 x2 and x3 is 1.0 2.0 and 4.0):
//
//    y1 = x1 + x2 +x3                               equ.(1)
//    y2 = x2+x3                                     equ.(2)
//
// It's clear that the above equations is rand defect, and an additional 
// constraint is added to solver the problem:
//
//    x3 = 4.0                                       equ.(3)
//
// Now, x1 x2 and x3 can be solved. For gnss data processing, we usually feed back
// the fixed integer ambiguity to the solver to improve the solution, the following
// show how to feed back some other constraint to the solver.
//
//    x1 = 1.0                                       equ.(4)
//
// Rank defect is the key problem for some PPP-RTK algorithms, and this framework
// is designed to implement these algorithms gracefully. And more examples will 
// added to show how to use these classes to do RTK and PPP-RTK in the near future.
```

## navfilterex.cpp
这个需要手动输入数据，不清楚要怎么跑

