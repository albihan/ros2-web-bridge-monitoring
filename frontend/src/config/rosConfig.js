export const ROS_CONFIG = {
    WEBSOCKET_URL: `ws://${window.location.hostname}:9090`,
    TOPICS: {
        // topic dummy dari script python
        DATA_ENGINEER:{
            name: '/dashboard/telemetry',
            type: 'std_msgs/String'
        },
        IOT_SENSOR:{
            name: '/iot_data',
            type: 'std_msgs/String'
        },
        AI_DETECTION:{
            name: '/ai_data',
            type: 'std_msgs/String'
        },
        CONFIG_CMD:{
            name: '/dashboard/telemetry',
            type: 'std_msgs/String'
        }
    }
};