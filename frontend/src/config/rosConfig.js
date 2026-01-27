export const ROS_CONFIG = {
    WEBSOCKET_URL: `ws://${window.location.hostname}:9090`,
    TOPICS: {
        SUHU: { name: '/suhu', type: 'std_msgs/String' },
        KELEMBAPAN: { name: '/kelembapan', type: 'std_msgs/String' },
        LDR: { name: '/ldr', type: 'std_msgs/String' },
        // asumsi topik yg dikirim dari tim AI
        AI: { name: '/plant_health_status', type: 'std_msgs/String' }, // Asumsi output Sistem Cerdas
        CONFIG: { name: 'dashboard_config', type: 'std_msgs/String' }
    },
      
    DEFAULT_VALUES: {
        temperature: '--',
        humidity: '--',
        ldr: '--',
        ai_detection: 'Menunggu objek...',
        timestamp: null
    }
};