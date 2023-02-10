#include "commu_proto.h"
#include <unistd.h>
int main()
{
    while(1)
    {
        send_control(1.3,3.1,1.0,1.0,1.0,1,1);
        usleep(10000);  // 10ms
    }
    return 0;
}