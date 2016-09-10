#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Range.h>
#include <iostream>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const RangeConstPtr& image1, const RangeConstPtr& image2)
{
  std::cout << "hello" << std::endl;
  // Solve all of perception here...
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<Range> image1_sub(nh, "/robot0/sonar_0", 1);
  message_filters::Subscriber<Range> image2_sub(nh, "/robot0/sonar_1", 1);

  typedef sync_policies::ApproximateTime<Range, Range> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
