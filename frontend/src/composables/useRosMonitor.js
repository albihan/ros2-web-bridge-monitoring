//frontend/src/composables/useRosMonitor.js
import { ref, onUnmounted } from 'vue';
import ROSLIB from 'roslib';
import { ROS_CONFIG } from '../config/rosConfig';

export function useRosMonitor() {
  // --- STATE ---
  const isConnected = ref(false);
  const isStreamActive = ref(false);
  const rosInstance = ref(null);
  
  const sensorData = ref({
    temperature: ROS_CONFIG.DEFAULT_VALUES.temperature,
    humidity: ROS_CONFIG.DEFAULT_VALUES.humidity,
    ldr: ROS_CONFIG.DEFAULT_VALUES.ldr,
    ai_detection: ROS_CONFIG.DEFAULT_VALUES.ai_detection,
    timestamp: null
  });

  const events = ref([]);
  const connectionStatus = ref('Disconnected');

  // --- HELPERS ---
  const safeParse = (jsonString) => {
    try { 
      return JSON.parse(jsonString); 
    } catch (e) { 
      console.warn('Failed to parse JSON:', jsonString);
      return null; 
    }
  };

  const addEvent = (source, message, timeStr = null) => {
    const eventTime = timeStr || new Date().toLocaleTimeString();
    events.value.unshift({
      id: Date.now(),
      time: eventTime,
      source: source,
      msg: message,
      type: message.includes('⚠️') || message.includes('Gelap') ? 'warning' : 
            message.includes('❌') ? 'error' : 'info'
    });
    // Batasi log agar memori browser tidak penuh (Max 100)
    if (events.value.length > 100) events.value.pop();
  };

  // --- CORE CONNECTION ---
  const connect = () => {
    rosInstance.value = new ROSLIB.Ros({
      url: ROS_CONFIG.WEBSOCKET_URL
    });

    rosInstance.value.on('connection', () => {
      console.log('✅ Connected to ROS Bridge');
      isConnected.value = true;
      connectionStatus.value = 'Connected';
      addEvent('System', 'Connected to ROS Bridge');
      _setupSubscribers();
    });

    rosInstance.value.on('error', (error) => {
      console.error('❌ ROS Error:', error);
      isConnected.value = false;
      isStreamActive.value = false;
      connectionStatus.value = `Error: ${error.message}`;
      addEvent('System', `❌ Connection Error: ${error.message}`);
    });

    rosInstance.value.on('close', () => {
      console.log('⚠️ Connection Closed');
      isConnected.value = false;
      isStreamActive.value = false;
      connectionStatus.value = 'Disconnected';
      addEvent('System', '⚠️ Connection to ROS Bridge closed');
    });

    // Try to connect
    addEvent('System', 'Attempting to connect to ROS Bridge...');
  };

  const _setupSubscribers = () => {
    // ---------------------------------------------------------
    // 1. SUBSCRIBER SUHU
    // ---------------------------------------------------------
    const suhuTopic = new ROSLIB.Topic({
      ros: rosInstance.value,
      name: ROS_CONFIG.TOPICS.SUHU.name,
      messageType: ROS_CONFIG.TOPICS.SUHU.type
    });

    suhuTopic.subscribe((msg) => {
      isStreamActive.value = true;
      const data = msg.data;
      
      // Handle both string and object data
      if (typeof data === 'string') {
        const parsed = safeParse(data);
        if (parsed && parsed.temp !== undefined) {
          sensorData.value.temperature = parseFloat(parsed.temp).toFixed(1);
          sensorData.value.timestamp = parsed.timestamp || new Date().toISOString();
          
          // Check temperature thresholds
          const tempValue = parseFloat(parsed.temp);
          if (tempValue > 30) {
            addEvent('Suhu', `⚠️ Suhu Tinggi: ${tempValue.toFixed(1)}°C`);
          } else if (tempValue < 20) {
            addEvent('Suhu', `⚠️ Suhu Rendah: ${tempValue.toFixed(1)}°C`);
          }
        } else if (!isNaN(data)) {
          sensorData.value.temperature = parseFloat(data).toFixed(1);
          sensorData.value.timestamp = new Date().toISOString();
        }
      } else if (typeof data === 'number') {
        sensorData.value.temperature = data.toFixed(1);
        sensorData.value.timestamp = new Date().toISOString();
      }
    });

    // ---------------------------------------------------------
    // 2. SUBSCRIBER KELEMBAPAN
    // ---------------------------------------------------------
    const kelembapanTopic = new ROSLIB.Topic({
      ros: rosInstance.value,
      name: ROS_CONFIG.TOPICS.KELEMBAPAN.name,
      messageType: ROS_CONFIG.TOPICS.KELEMBAPAN.type
    });

    kelembapanTopic.subscribe((msg) => {
      isStreamActive.value = true;
      const data = msg.data;
      
      if (typeof data === 'string') {
        const parsed = safeParse(data);
        if (parsed && parsed.humid !== undefined) {
          sensorData.value.humidity = parseFloat(parsed.humid).toFixed(1);
          sensorData.value.timestamp = parsed.timestamp || new Date().toISOString();
          
          // Check humidity thresholds
          const humidValue = parseFloat(parsed.humid);
          if (humidValue > 80) {
            addEvent('Kelembapan', `⚠️ Kelembapan Tinggi: ${humidValue.toFixed(1)}%`);
          } else if (humidValue < 40) {
            addEvent('Kelembapan', `⚠️ Kelembapan Rendah: ${humidValue.toFixed(1)}%`);
          }
        } else if (!isNaN(data)) {
          sensorData.value.humidity = parseFloat(data).toFixed(1);
          sensorData.value.timestamp = new Date().toISOString();
        }
      } else if (typeof data === 'number') {
        sensorData.value.humidity = data.toFixed(1);
        sensorData.value.timestamp = new Date().toISOString();
      }
    });

    // ---------------------------------------------------------
    // 3. SUBSCRIBER LDR (Intensitas Cahaya)
    // ---------------------------------------------------------
    const ldrTopic = new ROSLIB.Topic({
      ros: rosInstance.value,
      name: ROS_CONFIG.TOPICS.LDR.name,
      messageType: ROS_CONFIG.TOPICS.LDR.type
    });

    ldrTopic.subscribe((msg) => {
      isStreamActive.value = true;
      const data = msg.data;
      
      if (typeof data === 'string') {
        const parsed = safeParse(data);
        if (parsed && parsed.ldr !== undefined) {
          const ldrValue = parseFloat(parsed.ldr);
          sensorData.value.ldr = ldrValue;
          sensorData.value.timestamp = parsed.timestamp || new Date().toISOString();
          
          // Logic Event: Jika terlalu gelap (< 100)
          if (ldrValue < 100) {
            const lastLog = events.value[0];
            if (!lastLog || !lastLog.msg.includes('Gelap')) {
              addEvent('LDR', `⚠️ Ruangan Gelap (LDR: ${ldrValue} lux)`);
            }
          } else if (ldrValue > 1500) {
            const lastLog = events.value[0];
            if (!lastLog || !lastLog.msg.includes('Terang')) {
              addEvent('LDR', `⚠️ Cahaya Sangat Terang (LDR: ${ldrValue} lux)`);
            }
          }
        } else if (!isNaN(data)) {
          sensorData.value.ldr = parseFloat(data);
          sensorData.value.timestamp = new Date().toISOString();
        }
      } else if (typeof data === 'number') {
        sensorData.value.ldr = data;
        sensorData.value.timestamp = new Date().toISOString();
      }
    });

    // ---------------------------------------------------------
    // 4. SUBSCRIBER SISTEM CERDAS (AI)
    // ---------------------------------------------------------
    const aiTopic = new ROSLIB.Topic({
      ros: rosInstance.value,
      name: ROS_CONFIG.TOPICS.AI.name,
      messageType: ROS_CONFIG.TOPICS.AI.type
    });

    aiTopic.subscribe((msg) => {
      const data = msg.data;
      
      if (typeof data === 'string') {
        const parsed = safeParse(data);
        
        if (parsed) {
          if (parsed.class === 'No Object' || parsed.class === 'Standby') {
            sensorData.value.ai_detection = 'Menunggu objek...';
          } else {
            // Tampilkan Deteksi + Confidence
            const conf = parsed.confidence ? (parsed.confidence * 100).toFixed(0) : 0;
            sensorData.value.ai_detection = `${parsed.class} (${conf}%)`;
            
            // Masukkan ke Event Log kalau confidence tinggi (>90%)
            if (parsed.confidence > 0.90) {
              const lastLog = events.value[0];
              if (!lastLog || !lastLog.msg.includes(parsed.class)) {
                addEvent('AI', `✅ Deteksi: ${parsed.class} (${conf}% confidence)`);
              }
            }
          }
        } else {
          // Fallback jika format bukan JSON
          sensorData.value.ai_detection = data;
        }
      }
    });
  };

  // --- ACTIONS ---
  const disconnect = () => {
    if (rosInstance.value) {
      rosInstance.value.close();
      isConnected.value = false;
      connectionStatus.value = 'Disconnected';
      addEvent('System', 'Manual disconnection');
    }
  };

  const reconnect = () => {
    disconnect();
    setTimeout(() => {
      connect();
    }, 1000);
  };

  const publishConfig = (payload) => {
    if (!isConnected.value) {
      addEvent('System', '❌ Cannot publish: Not connected to ROS');
      return;
    }
    
    try {
      const topic = new ROSLIB.Topic({
        ros: rosInstance.value,
        name: ROS_CONFIG.TOPICS.CONFIG.name,
        messageType: ROS_CONFIG.TOPICS.CONFIG.type
      });
      
      const message = new ROSLIB.Message({ 
        data: JSON.stringify(payload) 
      });
      
      topic.publish(message);
      addEvent('System', `📤 Published config: ${JSON.stringify(payload)}`);
    } catch (error) {
      console.error('Failed to publish config:', error);
      addEvent('System', `❌ Failed to publish config: ${error.message}`);
    }
  };

  // --- CLEANUP ---
  onUnmounted(() => {
    disconnect();
  });

  return { 
    isConnected, 
    isStreamActive, 
    connectionStatus,
    sensorData, 
    events, 
    connect, 
    disconnect,
    reconnect,
    publishConfig 
  };
}