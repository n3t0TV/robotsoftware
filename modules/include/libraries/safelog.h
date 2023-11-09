#define SAFE_LOG 0

void LOG_DEBUG_STREAM(string message)
{
  IF(!SAFE_LOG)
    ROS_DEBUG_STREAM(message);
}
void LOG_ERROR_STREAM(string message)
{
  IF(!SAFE_LOG)
    ROS_ERROR_STREAM(message);
}

void LOG_INFO_STREAM(string message)
{
  IF(!SAFE_LOG)
    ROS_INFO_STREAM(message);
}
void LOG_WARN_STREAM(string message)
{
  IF(!SAFE_LOG)
    ROS_WARN_STREAM(message);
}
