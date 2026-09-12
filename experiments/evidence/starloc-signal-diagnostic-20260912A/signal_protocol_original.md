# STAR-loc 六项输入诊断（2026-09-12 用户授权）

仅原zigzag_s4、loop-3d_s3、zigzag_s3，保护旧RR锁及全部结果。只做GT辅助development信号/短时闭合与裁剪准备；不自动把诊断拟合值写回估计器，不调detector/LCB或开展新RR矩阵。

同步独立使用UWB文件rig GT姿态增量与原始gyro，不读取imu.csv附带GT。0.1s姿态增量、0.1s gyro均值，GT bracket/gap≤0.05s；0.1s网格。诊断offset范围[-2,2]s、步长0.01s，前半按角速率模长Huber(0.2rad/s)损失选择，后半独立检验；选点后±0.01s有界细化。它是诊断估计，非已标定输入。24个proper signed-permutation及作者几何旋转/逆、旧diag分别检查，gyro轴选择只用前半，单位分别rad/s和deg/s→rad/s。

低运动条件：GT中心±0.25s的位置距离中心最大≤0.02m、姿态差≤0.02rad；GT间断不跨越。acc取中心±0.1s均值，以Rᵀ[0,0,9.81]比较，检查24轴旋转、旧轴与几何旋转，报告方向角与模长；不把低运动当精确静止，不拟合bias。

短时闭合使用GT起始R,p,v（明确oracle局部诊断，不是定位ATE），原始IMU线性插值、每步中点specific-force和SO3旋转积分。时长0.1/0.5/1s、每1s起点，GT/IMU间断>0.05s跳过并计数；GT速度由±0.25s局部三次位置拟合。零bias，分别用GT姿态积分acc与gyro传播姿态积分acc定位旋转/平移来源。只比较无offset/旧offset/信号offset+旧轴/信号offset+诊断轴/deg单位假设；全部报告不选ATE最好者。原点采用旧物理IMU近似，限制保留。

裁剪共同区间为max(首UWB,校正首IMU)到min(末UWB,校正末IMU)，不按GT/残差选择，保留原source ID。边界若落在IMU两样本之间，保留前驱供插值，不伪造样本。先报告去掉的消息及bootstrap时间支持；裁剪只解决端点覆盖，是否改善ATE未经新受控估计运行不能宣称。核对各数据adapter、统一cache/backend及SF独立入口。新增诊断脚本、工程fixture、隔离证据与报告、STATUS/claim记录，无commit/push。
