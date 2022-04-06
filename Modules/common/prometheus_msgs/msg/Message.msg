std_msgs/Header header

## 节点回传地面站的消息
uint8 message_type
# enum message_type
uint8 NORMAL = 0  
uint8 WARN   = 1  
uint8 ERROR  = 2  
 
string source_node

string content