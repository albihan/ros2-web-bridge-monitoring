// frontend/src/composables/useRosMonitor.js
import { ref, onUnmounted } from 'vue';
import ROSLIB from 'roslib';
import { ROS_CONFIG } from '../config/rosConfig';

export function useRosMonitor() {
  // --- STATE ---
  const isConnected = ref(false);
  const isStreamActive = ref(false);
  const rosInstance = ref(null);
  
  const sensorData = ref({
    temperature: '--',
    humidity: '--',
    ai_detection: 'Menunggu objek...',
    de_data: 'Menunggu Stream...'
  });

  const events = ref([]);

  // --- HELPERS ---
  const safeParse = (jsonString) => {
    try { return JSON.parse(jsonString); } 
    catch (e) { return null; }
  };

  const addEvent = (source, message, timeStr = null) => {
    events.value.unshift({
      time: timeStr || new Date().toLocaleTimeString(),
      source: source,
      msg: message
    });
    // Batasi log agar memori browser tidak penuh (Max 50)
    if (events.value.length > 50) events.value.pop();
  };

  // --- CORE CONNECTION ---
  const connect = () => {
    rosInstance.value = new ROSLIB.Ros({
      url: ROS_CONFIG.WEBSOCKET_URL
    });

    rosInstance.value.on('connection', () => {
      console.log('✅ Connected to ROS Bridge');
      isConnected.value = true;
      _setupSubscribers();
    });

    rosInstance.value.on('error', (error) => {
      console.error('❌ ROS Error:', error);
      isConnected.value = false;
      isStreamActive.value = false;
    });

    rosInstance.value.on('close', () => {
      console.log('⚠️ Connection Closed');
      isConnected.value = false;
      isStreamActive.value = false;
    });
  };

  const _setupSubscribers = () => {
    // ---------------------------------------------------------
    // 1. SUBSCRIBER DATA ENGINEER (Temp, Humid, LDR)
    // ---------------------------------------------------------
    const deTopic = new ROSLIB.Topic({
      ros: rosInstance.value,
      name: ROS_CONFIG.TOPICS.DATA_ENGINEER.name,
      messageType: ROS_CONFIG.TOPICS.DATA_ENGINEER.type
    });

    deTopic.subscribe((msg) => {
      isStreamActive.value = true;
      const parsed = safeParse(msg.data);

      if (parsed) {
        // A. Handling Suhu (Null Safety)
        if (parsed.temp !== null && parsed.temp !== undefined) {
          sensorData.value.temperature = parsed.temp.toFixed(1);
        } else {
          sensorData.value.temperature = '--';
        }

        // B. Handling Kelembapan
        if (parsed.humid !== null && parsed.humid !== undefined) {
          sensorData.value.humidity = parsed.humid.toFixed(1);
        } else {
          sensorData.value.humidity = '--';
        }

        // C. Handling LDR (Tampilkan Nilai & Cek Warning)
        if (parsed.ldr !== null && parsed.ldr !== undefined) {
          // Tampilkan nilai di kartu "Data Engineer"
          sensorData.value.de_data = `${parsed.ldr} Lux`;

          // Logic Event: Jika terlalu gelap (< 100)
          if (parsed.ldr < 100) {
            // Cek log terakhir agar tidak spam (opsional)
            const lastLog = events.value[0];
            if (!lastLog || !lastLog.msg.includes('Gelap')) {
                addEvent("Data Engineer", `⚠️ Ruangan Gelap (LDR: ${parsed.ldr})`, parsed.timestamp);
            }
          }
        }
      }
    });

    // ---------------------------------------------------------
    // 2. SUBSCRIBER SISTEM CERDAS (AI)
    // ---------------------------------------------------------
    const aiTopic = new ROSLIB.Topic({
      ros: rosInstance.value,
      name: ROS_CONFIG.TOPICS.AI_DETECTION.name,
      messageType: ROS_CONFIG.TOPICS.AI_DETECTION.type
    });

    aiTopic.subscribe((msg) => {
      const parsed = safeParse(msg.data);
      
      if (parsed && parsed.class) {
        if (parsed.class === 'No Object') {
            sensorData.value.ai_detection = 'Standby...';
        } else {
            // Tampilkan Deteksi + Confidence
            const conf = parsed.confidence ? (parsed.confidence * 100).toFixed(0) : 0;
            sensorData.value.ai_detection = `📷 ${parsed.class} (${conf}%)`;
            
            // Masukkan ke Event Log kalau confidence tinggi (>90%)
            if (parsed.confidence > 0.90) {
                 // Cek duplikasi event sederhana
                 const lastLog = events.value[0];
                 if (!lastLog || lastLog.msg !== `Objek Valid: ${parsed.class}`) {
                    addEvent('Sistem Cerdas', `Objek Valid: ${parsed.class}`, parsed.timestamp);
                 }
            }
        }
      } else {
        // Fallback jika format salah
        sensorData.value.ai_detection = msg.data;
      }
    });
  };

  // --- ACTIONS ---
  const publishConfig = (payload) => {
    if (!isConnected.value) return;
    const topic = new ROSLIB.Topic({
      ros: rosInstance.value,
      name: ROS_CONFIG.TOPICS.CONFIG_CMD.name,
      messageType: ROS_CONFIG.TOPICS.CONFIG_CMD.type
    });
    topic.publish(new ROSLIB.Message({ data: JSON.stringify(payload) }));
  };

  // --- CLEANUP ---
  onUnmounted(() => {
    if (rosInstance.value) {
        rosInstance.value.close();
    }
  });

  return { isConnected, isStreamActive, sensorData, events, connect, publishConfig };
}