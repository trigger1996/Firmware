# 收入的UDP套接字信息，全部更新在这个消息中

# 时间同步
uint8 hour
uint8 minute
uint8 sec
uint8 msec

# 航点
int32 lng	# 经纬度精确到10^7
int32 lat	
int32 alt	# 高度精确到毫米

# 任务事件
uint32 task	# 任务事件用4位表示，留出足够的空间
