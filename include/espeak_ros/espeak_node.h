#include <ros/ros.h>
#include <string>
#include <list>
#include <pthread.h>
#include <std_msgs/String.h>

class EspeakNode
{
  public:
    EspeakNode();
    virtual ~EspeakNode();

    void run();

  private:
    void loadParams();
    void initializeEspeak();

    ros::NodeHandle nh_;

    // A ROS topic we subscribe to and speak whatever gets published to it
    // Note that if the rate data is published is too fast we may drop messages or just be super slow
    ros::Subscriber speakInputTopic_;

    void speakCallback(const std_msgs::String::ConstPtr &msg);
    void speak(const std::string &msg);

    // pthread stuff for managing the message queue
    int maxQueue_;
    pthread_t speakThread_;
    pthread_mutex_t messageQueueLock_;
    std::list<std::string> messageQueue_;
    static void *speakThreadFn(void *arg);

    // espeak SDK elements
    std::string espeakVoice_;
    espeak_PARAMETER espeakParams_;
    espeak_POSITION_TYPE espeakPositionType_;
    espeak_AUDIO_OUTPUT espeakOutput_;
    unsigned int espeakFlags_;
    int espeakPlaybackBuffLength_;
    int espeakPlaybackOptions_;
    static int espeakSynthCallback(short *wav, int numSamples, espeak_EVENT *events);
};
