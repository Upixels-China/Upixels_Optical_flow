# Upixels_Optical_flow
Upixels Optical flow sensors docs,demos,e.g..

# Upixels Optical Flow Module Data Warehouse
This warehouse contains documentation in both Chinese and English, data manuals, demos, and other files.

Special Note: If some customers cannot use the LC302 optical flow driver on the flight control normally, please check whether the silk screen on the module PC board is LC-302-3C. If it is LC-302-8B, please use the optical flow driver of LC306. LC32-8B and LC306 also need to be initialized to work normally.

点击这里切换至中文版本说明文档[Click here to switch to the Chinese version of the instruction document：README_CN.md](./README_CN.md)
 
## Directory Structure


```plaintext
Repository Root Directory
│
├── CH34x_driver/    #CH34x driver
|                
├── demo/           #Sample code
|   └── stm32_demo_code_LC302-3C/   
|   |             #STM32 parsing LC302-3C optical flow 
|   |               module routine.
|   └── stm32_demo_code_Tx&LC302-GS/    
|   |           #STM32 analysis of Tx series and LC302-GS  
|   |               optical flow module routine.
|   └── uav_demo/   #FCU Analysis Optical Flow Data Routine.
|
├── docs/           #PDF document directory.
|   └── Upixels_Help/   #General Help Document.
|   └── Upixels_LC3xx/  #LC3xx series documents.
|   └── Upixels_Tx/     #Tx series documents.
│
└── ......  # Stay tuned for the rest.
```

## Contact and Support
 
If you have any questions or need technical support, please contact us through the following methods:
 
- Official Website: http://www.upixels.com
- Email: @upixels.com
- Phone: 17773155015
- Address: 11th Floor, Building A, Jinrong Yanggu Golden Garden, Luhu Road No. 39, Changsha High-Tech Development Zone (China)

Thank you for using the Upixels Optical Flow Upper Computer Parameter Adjustment Tool!
