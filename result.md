Task 1:

光度标定的结果:

![1585706235702](./result.assets/1585706235702.png)





参考: <https://arxiv.org/pdf/1807.08957.pdf>

## 1相机模型: 针孔模型

### 1.1 

![1585722460769](./result.assets/1585722460769.png)



### 1.2 反投影 

![1585722499466](./result.assets/1585722499466.png)

## 2 Unified Camera Model  (UCM)

### 2.1

![1585722682609](./result.assets/1585722682609.png)
![1585722697279](./result.assets/1585722697279.png)
![1585723280313](./result.assets/1585723280313.png)

### 2.2

![1585723359864](./result.assets/1585723359864.png)
## 3.Extended Unified Camera Model  (EUCM)

### 3.1

![1585723508961](./result.assets/1585723508961.png)
### 3.2 

![1585723544027](./result.assets/1585723544027.png)
## 4.  *** Kannala-Brandt Camera Model (KB)

![1585724024652](./result.assets/1585724024652.png)
### 4.1

![1585723597389](./result.assets/1585723597389.png)
### 4.2 

![1585723625757](./result.assets/1585723625757.png)
## 5. Field-of-View Camera Model  (FOV) 

### 5.1 

![1585723693733](./result.assets/1585723693733.png)
### 5.2 

![1585723719970](./result.assets/1585723719970.png)


## 5 . Double Sphere Camera Model  (DS)

### 5.1 

![1585723776372](./result.assets/1585723776372.png)
![1585723786492](./result.assets/1585723786492.png)
### 5.2 

![1585723808257](./result.assets/1585723808257.png)
![1585723820788](./result.assets/1585723820788.png)



## 6 去畸变内参

![1585724538949](/home/yhzhao/kint/hw/robkin_DSO_HW1/result.assets/1585724538949.png)




Task2 

DSO的初始化是直接基于像素值处理，需要相机曝光时间相对稳定，运动过程不宜过大。 Orb是基于特征的方式处理， 运算消耗相对大一些，速率慢， 初始化过程最好有一定的平移运动。
深层比较待续～～～～









代码参照DSO的过程已经完成，效果却不正常，暂没解决:   ====>  opencv 元素操作的问题

![image-20200329233933965](/home/yhzhao/kint/hw/robkin_DSO_HW.3965.png)