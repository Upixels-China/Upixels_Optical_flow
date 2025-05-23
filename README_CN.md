# Upixels_Optical_flow
Upixels Optical flow sensors docs,demos,e.g..

# 优像光流模块资料仓库
本仓库包含中英文说明文档、数据手册、demo等文件。

特别说明：有部分客户在飞控上使用LC302光流驱动无法正常使用的话，请检查模块PCB板上的丝印是否为LC-302-3C，如果是LC-302-8B请使用LC306的光流驱动，LC302-8B与LC306同样需要初始化才能正常工作。

点击这里切换至英文版本说明文档[Click here to switch to the English version of the instruction document：README.md](./README.md)
 
## 目录结构


```plaintext
仓库根目录
│
├── CH34x_driver/    #CH34x驱动
|                
├── demo/           #例程
|   └── stm32_demo_code_LC302-3C/   
|   |             #stm32解析LC302-3C光流模块例程
|   └── stm32_demo_code_Tx&LC302-GS/    
|   |           #stm32解析Tx系列与LC302-GS光流模块例程
|   └── uav_demo/   #飞控解析光流数据例程
|
├── docs/           #pdf文档目录
|   └── Upixels_Help/   #通用帮助文档
|   └── Upixels_LC3xx/  #LC3xx系列文档
|   └── Upixels_Tx/     #Tx系列文档
│
└── ......  # 其他的敬请期待
```

 
## 联系与支持
 
如有任何问题或需要技术支持，请通过以下方式联系我们：
 
- 官方网址：http://www.upixels.com
- 邮箱：hnyx@upixels.com
- 电话：17773155015
- 地址：长沙高新开发区麓湖路39号金荣央谷金苑A座11楼 

感谢您使用优像光流上位机调参工具！
