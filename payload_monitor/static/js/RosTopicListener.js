class RosTopicListener extends DataListener
{
    constructor(topicName, topicType)
    {
        let encodedTopicName = topicName.replace(/\//g, '-')
        let encodedTopicType = topicType.replace(/\//g, '-')
        let subscriptionUrl = '/ws/ros_subscribe/'
                            + encodedTopicName + '/'
                            + encodedTopicType + '/';
        console.log(subscriptionUrl)
        super(subscriptionUrl)
    }
};


