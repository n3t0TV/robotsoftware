
#include <string>
#include <ctime>

using namespace std;

/*
Feedtainer (feedback of container) is a data structure that holds timestamp information 
of each container feedback message published in the /mqtt_publishers topic.
*/
struct FeedtainerStatus {
    int pid;
    bool status;
    bool connection;
    string id;
    string param_path;
    time_t previous;
};