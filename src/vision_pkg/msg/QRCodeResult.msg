# QR码检测结果消息
Header header
bool detected                 # 是否检测到二维码
string data                   # 二维码数据内容
geometry_msgs/Point position  # 二维码在图像中的中心位置
geometry_msgs/Point[] corners # 二维码四个角点在图像中的位置
float32 confidence           # 检测置信度 (0.0-1.0)
