# RR GT-assisted development admission — prior to diagnostic and science

用户于本轮明确授权以现有GT/测距/IMU判断数据语义并继续原计划。这覆盖旧独立标定
准入限制；依旧不把GT送入任何estimator，不调detector/recovery、noise、beta或按ATE选模型。
旧报告与锁定产物保留。三条录制、full/tag1/all anchors、四方法、15任务/各1800s保持。

诊断采用刚体运动恒等式：gyro与GT姿态差分比较；加速度与
f_r=R_wr^T(p_r''−g)、候选IMU原点的旋转杆臂项比较。先固定0.2s中心姿态差分与
0.25s半宽三次局部位置多项式；只在完整且gap≤0.05s的GT窗口求导。不拟合时间偏移、
噪声、bias、尺度或轨迹。比较24个右手signed-axis置换，acc额外±符号；候选包含
rig/IMU/camera及world frame，给绝对误差和候选间距。三recording分别报告一致性，
不以算法定位效果选择预处理。GT只能辨别输入与运动模型的等价关系，不能恢复原始代码历史。

杆臂先核对作者tag_pos=rig_pos+R_wr*lever的恒等式（不用raw range拟合），
并用已有calib的厂商camera–IMU与作者camera–rig变换形成明确GT-assisted近似链。
同一rig共用三条发布camera–rig变换的平均旋转/平移，不逐录制追求最小误差。
对比IMU点/rig点加速度预言的差与观测误差；不能可靠区分时采用物理传感器原点IMU，
明确APPROXIMATE_ORIGIN而非虚假精确识别。把残留杆臂加速度差作为诊断记录。

如果数据支持固定坐标变换/符号，冻结constant preprocessing及共同近似lever后准入。
不对每时刻用GT校正IMU；不减观测bias、range_calib或静态beta。估计器只读取转换后的
measurement-only cache，GT改写在冻结变换之后不得改变cache/estimator身份。

成功分支须实际完成cache v2/C++ prepare和ROS往返（float32 range wire量化单列），
raw消息仍逐行异步。完成工程门后才串行producer/suppress/full/Cauchy/SFUISE；
所有失败/timeout继续记账，无参数重试。独立评价保持旧10Hz、nearest0.02s、GT bracket0.05s、
RR共同时刻全区间SE3、窗口不重对齐、range obs_id一致和最终实际offset定义。

## 时基直接阻塞 amendment（诊断发现后、科学运行前）

首次GT辅助诊断发现两CSV在同一数值time_s的GT不一致；同一发布GT姿态/位置在两文件
可高精度匹配，候选常数偏移约+0.176s/+0.789s/−1.427s。这是输入时基闭合的直接阻塞，
本轮用户授权“用GT等现有数据科学判断并解决阻塞”覆盖该定位；替代上面不拟合时移的
初始诊断限制。只按两CSV的GT几何对应估计一个常数IMU→UWB时移（scale固定1）；
不使用测距误差/IMU积分误差/estimator ATE选择时移，不做多工作点科学运行。
先以7维发布pose最近邻给初始偏移，再用GT位置/姿态线性插值的对应误差在±0.05s内
唯一标量最小化；拒绝外推，给时移、剩余pose误差及前/后半独立诊断。该时移作为
GT_ASSISTED_CLOCK_APPROXIMATION冻结，不能称独立时钟标定；之后所有方法读取同一转换。
body原点固定IMU、轴取rig轴，两个测量一起用diag(1,-1,-1)转到该body轴；杆臂也以rig轴表达。
