<template>
  <div class="smart-dashboard-container">
    <header class="header">
      <div class="logo">
        <h1 class="text-xl font-bold flex items-center gap-2">
            🌡️ Smart Monitoring <span class="text-xs bg-blue-100 text-blue-800 px-2 py-1 rounded">System</span>
        </h1>
      </div>
      <div class="header-actions flex gap-4 items-center">
        <span :class="['status-badge', isConnected ? 'status-online' : 'status-warning']">
          ● {{ isConnected ? 'Sistem Online' : 'Terputus' }}
        </span>
        <span class="font-mono font-bold text-gray-700">{{ currentTime }}</span>
      </div>
    </header>

    <div class="main-container">
      <nav class="sidebar">
        <ul class="nav-menu">
          <li class="nav-item">
            <a href="#" @click.prevent="activeTab = 'dashboard'" :class="['nav-link', activeTab === 'dashboard' ? 'active' : '']">
              📊 Dashboard
            </a>
          </li>
          <li class="nav-item">
            <a href="#" @click.prevent="activeTab = 'events'" :class="['nav-link', activeTab === 'events' ? 'active' : '']">
              📋 Event Log
            </a>
          </li>
          <li class="nav-item">
            <a href="#" @click.prevent="activeTab = 'settings'" :class="['nav-link', activeTab === 'settings' ? 'active' : '']">
              ⚙️ Node Config
            </a>
          </li>
        </ul>
      </nav>

      <main class="content">
        
        <div v-if="activeTab === 'dashboard'" class="page active">
          <h2 class="text-2xl font-bold mb-6 text-gray-800">Dashboard Real-time</h2>
          
          <div class="status-panel">
            <div class="status-item">
              <span>Status ROS Bridge:</span>
              <span :class="['status-badge', isConnected ? 'status-online' : 'status-warning']">
                {{ isConnected ? 'Connected' : 'Disconnected' }}
              </span>
            </div>
            <div class="status-item">
              <span>Data Stream:</span>
              <span :class="['status-badge', isStreamActive ? 'status-online' : 'status-warning']">
                {{ isStreamActive ? 'Active (Receiving)' : 'Idle (No Data)' }}
              </span>
            </div>
          </div>

          <div class="sensor-grid">
            <div class="sensor-card temperature">
              <h3>🌡️ Temperature</h3>
              <div class="sensor-value">{{ sensorData.temperature }}</div>
              <div class="sensor-trend" style="color: var(--danger);">Live Data</div>
              <div class="sensor-meta">Source: /iot_data</div>
            </div>

            <div class="sensor-card humidity">
              <h3>💧 Humidity</h3>
              <div class="sensor-value">{{ sensorData.humidity }}</div>
              <div class="sensor-trend" style="color: var(--success);">Live Data</div>
              <div class="sensor-meta">Source: /iot_data</div>
            </div>

            <div class="sensor-card camera">
              <h3>📷 AI Detection</h3>
              <div class="ai-box">
                {{ sensorData.ai_detection || 'Menunggu deteksi...' }}
              </div>
              <div class="sensor-meta">Source: /ai_data</div>
            </div>
            
             <div class="sensor-card" style="border-left-color: purple;">
              <h3>💾{{ sensorData.de_data }}</h3>
              <div class="ai-box">
                {{ sensorData.de_data || 'Waiting...' }}
              </div>
              <div class="sensor-meta">Source: /dashboard/telemetry</div>
            </div>
          </div>

          <div v-if="latestEvent" class="notification">
            ⚠️ <strong>Alert Terakhir:</strong> {{ latestEvent.msg }} <br>
            <small>Waktu: {{ latestEvent.time }}</small>
          </div>
        </div>

        <div v-if="activeTab === 'events'" class="page">
          <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 2rem;">
            <h2 class="text-2xl font-bold text-gray-800">Event Log History</h2>
            <button class="btn-clear" disabled title="Logs managed by system">Auto-Managed</button>
          </div>

          <div class="event-table">
            <table>
              <thead>
                <tr>
                  <th>Timestamp</th>
                  <th>Topic Source</th>
                  <th>Pesan Event</th>
                </tr>
              </thead>
              <tbody>
                <tr v-for="(ev, index) in events" :key="index">
                  <td>{{ ev.time }}</td>
                  <td>{{ ev.source }}</td>
                  <td>{{ ev.msg }}</td>
                </tr>
                <tr v-if="events.length === 0">
                  <td colspan="3" style="text-align:center; padding: 2rem;">Belum ada event masuk.</td>
                </tr>
              </tbody>
            </table>
          </div>
        </div>

        <div v-if="activeTab === 'settings'" class="page">
          <div class="node-configuration">
            <h3 class="text-xl font-bold mb-4">⚙️ Konfigurasi Mode Operasi Node</h3>
            <p style="margin-bottom: 20px; color: #666;">
              Fitur ini memungkinkan Tim RPL mengirim perintah balik ke ROS2 untuk mengubah mode sensor Tim IoT.
            </p>
            
            <div class="node-settings">
              <h4 class="font-bold mb-4 text-green-700">🟢 Node-001 (IoT Sensor)</h4>
              
              <div class="settings-item">
                <span>Mode Operasi:</span>
                <select v-model="nodeConfig.mode" class="node-mode p-2 border rounded">
                  <option value="continuous">Continuous (Real-time)</option>
                  <option value="event_driven">Event-driven (Hemat Daya)</option>
                </select>
              </div>

              <div class="settings-item" v-if="nodeConfig.mode === 'continuous'">
                <span>Sampling Rate (ms):</span>
                <select v-model="nodeConfig.rate" class="p-2 border rounded">
                  <option value="1000">1000 ms (Cepat)</option>
                  <option value="5000">5000 ms (Normal)</option>
                </select>
              </div>

              <div class="settings-item" v-if="nodeConfig.mode === 'event_driven'">
                <span>Threshold Suhu (°C):</span>
                <input type="number" v-model="nodeConfig.threshold" class="p-2 border rounded w-24">
              </div>

              <div style="margin-top: 1rem; text-align: right;">
                <button @click="sendConfig" class="btn-save">
                  Kirim Konfigurasi ke ROS2
                </button>
              </div>
            </div>
          </div>
        </div>

      </main>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted } from 'vue';
import { useRosMonitor } from '../composables/useRosMonitor'; // Import Logic Bersih kita

// 1. Panggil Logic ROS dari Composable
const { 
  isConnected, 
  isStreamActive, 
  sensorData, 
  events, 
  connect, 
  publishConfig 
} = useRosMonitor();

// 2. State Lokal untuk UI (Tampilan)
const activeTab = ref('dashboard');
const currentTime = ref('');
const nodeConfig = ref({
  mode: 'continuous',
  rate: '5000',
  threshold: 30
});

// 3. Computed Property (Untuk mengambil event paling atas/terbaru)
const latestEvent = computed(() => {
  return events.value.length > 0 ? events.value[0] : null;
});

// 4. Logic UI Tambahan (Jam Digital)
let clockInterval;
const updateClock = () => {
  currentTime.value = new Date().toLocaleTimeString('id-ID');
};

// 5. Wrapper Function untuk Tombol Kirim Config
const sendConfig = () => {
  // Panggil fungsi publishConfig dari composable dengan data dari form lokal
  publishConfig(nodeConfig.value);
  alert(`Perintah dikirim ke ROS2:\nMode: ${nodeConfig.value.mode}`);
};

// Lifecycle
onMounted(() => {
  connect(); // Mulai ROS
  updateClock();
  clockInterval = setInterval(updateClock, 1000);
});

onUnmounted(() => {
  clearInterval(clockInterval);
});
</script>

<style scoped>
/* --- CSS AGAR TAMPILAN SESUAI PERMINTAAN --- */
:root {
  --primary: #2563eb; --success: #10b981; --warning: #f59e0b; --danger: #ef4444;
  --dark: #1e293b; --light: #f8fafc; --border: #e2e8f0;
}

.smart-dashboard-container {
  font-family: 'Segoe UI', system-ui, sans-serif;
  background: #f1f5f9;
  color: #1e293b;
  min-height: 100vh;
  display: flex;
  flex-direction: column;
}

/* HEADER */
.header {
  background: white; padding: 1rem 2rem; display: flex;
  justify-content: space-between; align-items: center;
  box-shadow: 0 2px 4px rgba(0,0,0,0.05);
  z-index: 10;
}

/* LAYOUT UTAMA */
.main-container { display: flex; flex: 1; }

/* SIDEBAR */
.sidebar {
  width: 260px; background: white; padding: 2rem 0;
  box-shadow: 2px 0 4px rgba(0,0,0,0.05);
  display: flex; flex-direction: column;
}
.nav-menu { list-style: none; padding: 0; margin: 0; }
.nav-link {
  display: flex; align-items: center; gap: 1rem;
  padding: 1rem 1.5rem; color: #64748b; text-decoration: none;
  transition: all 0.2s ease; border-left: 4px solid transparent;
  font-weight: 500;
}
.nav-link:hover { background: #f8fafc; color: #2563eb; }
.nav-link.active {
  background: #eff6ff; color: #2563eb; border-left-color: #2563eb;
}

/* CONTENT AREA */
.content { flex: 1; padding: 2rem; overflow-y: auto; }

/* STATUS PANEL */
.status-panel { display: flex; gap: 1rem; margin-bottom: 2rem; flex-wrap: wrap; }
.status-item {
  background: white; padding: 0.75rem 1rem; border-radius: 8px;
  box-shadow: 0 1px 2px rgba(0,0,0,0.05); display: flex;
  align-items: center; gap: 0.5rem; font-size: 0.9rem;
}
.status-badge {
  padding: 2px 10px; border-radius: 20px; font-size: 0.75rem;
  font-weight: 700; text-transform: uppercase;
}
.status-online { background: #dcfce7; color: #166534; }
.status-warning { background: #fee2e2; color: #991b1b; }

/* SENSOR CARDS */
.sensor-grid {
  display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 1.5rem; margin-bottom: 2rem;
}
.sensor-card {
  background: white; padding: 1.5rem; border-radius: 12px;
  box-shadow: 0 4px 6px -1px rgba(0,0,0,0.1); border-left: 4px solid #2563eb;
  transition: transform 0.2s;
}
.sensor-card:hover { transform: translateY(-2px); }
.sensor-card.temperature { border-left-color: #ef4444; }
.sensor-card.humidity { border-left-color: #10b981; }

.sensor-value { font-size: 2.5rem; font-weight: 800; margin: 0.5rem 0; color: #1e293b; }
.sensor-meta { font-size: 0.75rem; color: #94a3b8; margin-top: 1rem; }
.ai-box {
  background: #f1f5f9; height: 100px; border-radius: 8px; 
  display: flex; align-items: center; justify-content: center; 
  margin: 1rem 0; color: #475569; font-weight: 600; text-align: center; padding: 10px;
}

/* NOTIFICATION & TABLE */
.notification {
  background: #fffbeb; border: 1px solid #fcd34d; color: #92400e;
  padding: 1rem; border-radius: 8px; margin-bottom: 2rem;
}
.event-table {
  background: white; border-radius: 12px; overflow: hidden;
  box-shadow: 0 4px 6px -1px rgba(0,0,0,0.1);
}
table { width: 100%; border-collapse: collapse; }
th { background: #f8fafc; font-weight: 600; color: #64748b; text-transform: uppercase; font-size: 0.75rem; letter-spacing: 0.05em; padding: 1rem; text-align: left; }
td { padding: 1rem; border-top: 1px solid #e2e8f0; color: #334155; font-size: 0.9rem; }

/* SETTINGS FORM */
.node-configuration { background: white; padding: 2rem; border-radius: 12px; box-shadow: 0 4px 6px -1px rgba(0,0,0,0.1); }
.settings-item {
  display: flex; justify-content: space-between; align-items: center;
  margin-bottom: 1rem; padding: 1rem; background: #f8fafc;
  border-radius: 8px; border: 1px solid #e2e8f0;
}
.btn-save {
  background: #2563eb; color: white; border: none; padding: 10px 24px;
  border-radius: 6px; cursor: pointer; font-weight: 600;
  transition: background 0.2s;
}
.btn-save:hover { background: #1d4ed8; }
.btn-clear {
  background: #ef4444; color: white; border: none; padding: 6px 12px;
  border-radius: 4px; font-size: 0.8rem; cursor: not-allowed; opacity: 0.7;
}
</style>