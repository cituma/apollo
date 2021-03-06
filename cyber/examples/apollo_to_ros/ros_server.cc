#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <thread>
#include "cyber/cyber.h"
#include "cyber/examples/apollo_to_ros/apollo_to_ros.h"

using apollo::cyber::Time;

int main(int argc, char* argv[]) {
  apollo::cyber::Init(argv[0]);
  google::LogToStderr();

  uint16_t server_port = 11435;

  SendMessage snd_msg;
  snd_msg.Init(server_port);

  apollo::cyber::WaitForShutdown();
  snd_msg.Destroy();
  return 0;
}
