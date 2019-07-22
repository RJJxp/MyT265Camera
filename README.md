# 说明文档

- 订阅轮式里程计话题 `/topic_odom_in` , 向 t265 相机输入其数据辅助相机内置里程计
- 发布话题 `/topic_odom_out` , 融合了轮式里程计数据后的 t265 相机位姿信息



## `t265_odom_first` 与 `t265_odom_second` 

参考 https://github.com/IntelRealSense/librealsense librealsense中的 `/example` 

与 https://github.com/IntelRealSense/realsense-ros 中的框架

采取原始的相机启动方式

> context --> device --> sensor --> stream_profile --> pose
>
>    	                     --> wheel_odometry

最终实现是通过 `rs2::sensor` 的 `open()` 函数加载 `stream_profile` 

和 `start()` 函数调用 callback 函数

两个写的几乎一样, 区别只是在 `stream_profile` 不同



结果就是发布不了话题, 也接受不到话题

啃了很久 realsense 源码, 找不出问题出在哪里

猜想, 启动还是有问题或是线程冲突



## `t265_odom_third` 与 `t265_odom_4th` 

 赶时间, 所以没去纠结之前的问题

改变了获取数据的方式

> pipeline --> pose
>
> ​       	--> pipeline_profile --> device --> wheel_odometry

 `t265_odom_third` 面向对象的实验了一下

 `t265_odom_4th` 用对象封装

实现了上述功能



## 感想

对 git 使用的意识不够熟练, 所以分了四个文件夹去记录每个版本, 这点很失败



一开始对 realsense 不熟悉

最最最原始的版本中, 仿照 `example/` 中的写法

用 pipeline 获取 pose 数据

再用传统方式输入轮式里程计信息

两者冲突, 但谁能反应过来是启动方式的问题呢?



看源码, 写出了 first, 就是运行不起来

检查启动方式, 在 second 中改变了启动参数, 依旧启动不起来

苦思冥想, 找不到原因



在看 `example/` 中的启动方式

发现 stream_profile 可以得到传感器的各种信息

那么 pipeline 会不会也有 pipeline_profile 这种神奇的属性或是方法



于是一切迎刃而解



相当自己踩了一个坑

之间由于 `example/` 信息不全被误导

感觉不是很好

也没有任何人可以指导 .....

难受