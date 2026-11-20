# VLA lifecycle state (authoritative)

uint8 state
string error_code        # 稳定的类别标签，定义在 Python 端 VLAErrorCode；空串表示无错误
string error_message     # 人读的详细描述；非 ERROR 状态下为空串

uint8 IDLE     = 0
uint8 RUNNING  = 1
uint8 PAUSED   = 2
uint8 STOPPED  = 3
uint8 RESET    = 4
uint8 ERROR    = 255
