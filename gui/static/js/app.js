// Vue App - Drone Control System
const { createApp } = Vue;

createApp({
    data() {
        return {
            activeTab: 'basic',  // Active tab
            statusTimer: null,   // Polling timer
            
            // Drone status
            drone: {
                connected: false,
                flying: false,
                battery: 0,
                gps_fix: false,
                satellites: 0,
                latitude: 0.0,
                longitude: 0.0,
                altitude: 0.0
            },
            
            // Navigation status
            navigation: {
                state: 'idle',
                target_lat: 0.0,
                target_lon: 0.0,
                target_alt: 2.0,
                current_distance: 0.0,
                progress: 0,
                message: 'Ready'
            },
            
            // Navigation config
            navConfig: {
                target_lat: 47.6217966,
                target_lon: -122.1784220,
                target_alt: 2.0,
                arrival_threshold: 0.5
            },
            
            // Perception status
            perception: {
                state: 'idle',
                model: 'yolov8n.pt',
                tracking_mode: 'yolo',
                target_classes: ['keyboard'],
                target_color: 'red',
                detected: false,
                tracking: false,
                stable: false,
                confidence: 0.0,
                message: 'Ready'
            },
            
            // Perception config
            perceptionConfig: {
                mode: 'yolo',
                classes_str: 'keyboard',
                color: 'red'
            },
            
            // Winch status
            winch: {
                state: 'idle',
                current_action: 'IDLE',
                message: 'Ready',
                progress: 0
            },
            
            // Logs
            logs: [],
            
            // Manual control
            manualControlInterval: null,
            currentControl: { axis: null, value: 0 },
            
            // Button busy locks (prevent double-click)
            flightBusy: false,
            connectBusy: false,
            
            // Saved GPS locations
            savedLocations: [],
            newLocationName: '',
            selectedLocationIdx: -1
        };
    },
    
    mounted() {
        // Start polling status every 500ms (no SocketIO needed)
        this.startPolling();
        this.loadSavedLocations();
        this.addLog('GUI loaded, polling started', 'info');
    },
    
    beforeUnmount() {
        this.stopPolling();
    },
    
    methods: {
        // ========== Status Polling (replaces SocketIO) ==========
        
        startPolling() {
            this.fetchStatus();  // Fetch immediately
            this.statusTimer = setInterval(() => {
                this.fetchStatus();
            }, 500);
        },
        
        stopPolling() {
            if (this.statusTimer) {
                clearInterval(this.statusTimer);
                this.statusTimer = null;
            }
        },
        
        async fetchStatus() {
            try {
                const response = await fetch('/api/status');
                if (response.ok) {
                    const data = await response.json();
                    this.updateStatus(data);
                }
            } catch (error) {
                // Silently ignore polling errors
            }
        },
        
        // Update status from polled data
        updateStatus(data) {
            if (data.drone) {
                // 诊断: 检测 flying 状态变更
                const oldFlying = this.drone.flying;
                const newFlying = data.drone.flying;
                if (oldFlying !== newFlying) {
                    console.warn(`[DIAG] flying changed: ${oldFlying} → ${newFlying}`, 
                                 'connected:', data.drone.connected,
                                 'battery:', data.drone.battery);
                    this.addLog(`DIAG: flying ${oldFlying}→${newFlying} (from backend poll)`, 'warning');
                }
                
                // 诊断: 检测 connected 状态变更
                const oldConn = this.drone.connected;
                const newConn = data.drone.connected;
                if (oldConn !== newConn) {
                    console.warn(`[DIAG] connected changed: ${oldConn} → ${newConn}`);
                    this.addLog(`DIAG: connected ${oldConn}→${newConn} (from backend poll)`, 'warning');
                }
                
                this.drone = { ...this.drone, ...data.drone };
            }
            if (data.navigation) {
                this.navigation = { ...this.navigation, ...data.navigation };
            }
            if (data.perception) {
                this.perception = { ...this.perception, ...data.perception };
            }
            if (data.winch) {
                this.winch = { ...this.winch, ...data.winch };
            }
            if (data.logs && data.logs.length > 0) {
                // Only add new logs
                data.logs.forEach(log => {
                    if (!this.logs.find(l => 
                        l.time === log.time && 
                        l.message === log.message
                    )) {
                        this.logs.push(log);
                    }
                });
                // Limit log count
                if (this.logs.length > 50) {
                    this.logs = this.logs.slice(-50);
                }
                // Auto scroll to bottom
                this.$nextTick(() => {
                    const logsContainer = document.querySelector('.logs-container');
                    if (logsContainer) {
                        logsContainer.scrollTop = logsContainer.scrollHeight;
                    }
                });
            }
        },
        
        // Add local log
        addLog(message, level = 'info') {
            const timestamp = new Date().toLocaleTimeString('en-US', { 
                hour12: false 
            });
            this.logs.push({
                time: timestamp,
                level: level,
                message: message
            });
        },
        
        // Clear logs
        clearLogs() {
            this.logs = [];
        },
        
        // ========== API Calls ==========
        
        async apiCall(endpoint, method = 'POST', data = null) {
            try {
                const options = {
                    method: method,
                    headers: {
                        'Content-Type': 'application/json'
                    }
                };
                
                if (data) {
                    options.body = JSON.stringify(data);
                }
                
                const response = await fetch(endpoint, options);
                const result = await response.json();
                return result;
            } catch (error) {
                console.error('API call error:', error);
                this.addLog(`API Error: ${error.message}`, 'error');
                return { success: false };
            }
        },
        
        // ========== Drone Control ==========
        
        async connectDrone() {
            if (this.connectBusy) return;
            this.connectBusy = true;
            try {
                this.addLog('Connecting to drone...', 'info');
                const result = await this.apiCall('/api/connect');
                if (result.success) {
                    this.addLog('Drone connected', 'success');
                } else {
                    this.addLog('Connection failed', 'error');
                }
            } finally {
                this.connectBusy = false;
            }
        },
        
        async disconnectDrone() {
            if (this.connectBusy) return;
            this.connectBusy = true;
            try {
                const result = await this.apiCall('/api/disconnect');
                if (result.success) {
                    this.addLog('Drone disconnected', 'info');
                }
            } finally {
                this.connectBusy = false;
            }
        },
        
        async toggleFlight() {
            if (this.flightBusy) return;  // 防止双击
            this.flightBusy = true;
            try {
                if (this.drone.flying) {
                    // Land
                    this.addLog('Landing...', 'info');
                    const result = await this.apiCall('/api/land');
                    if (result.success) {
                        this.drone.flying = false;
                        this.addLog('Landed', 'success');
                    } else {
                        this.addLog('Landing failed', 'error');
                    }
                } else {
                    // Take Off
                    this.addLog('Taking off...', 'info');
                    const result = await this.apiCall('/api/takeoff');
                    if (result.success) {
                        this.drone.flying = true;
                        this.addLog('Takeoff successful', 'success');
                    } else {
                        this.addLog('Takeoff failed', 'error');
                    }
                }
            } finally {
                this.flightBusy = false;
            }
        },
        
        async emergencyStop() {
            // No confirmation dialog - this is EMERGENCY, every ms counts
            this.addLog('🚨 EMERGENCY STOP', 'warning');
            await this.apiCall('/api/emergency_stop');
        },
        
        // ========== Saved Locations ==========
        
        loadSavedLocations() {
            try {
                const data = localStorage.getItem('drone_saved_locations');
                if (data) {
                    this.savedLocations = JSON.parse(data);
                } else {
                    // Default preset locations
                    this.savedLocations = [
                        { name: 'GIX Building', lat: 47.6218425, lon: -122.1769126, alt: 2.0 }
                    ];
                    this.persistLocations();
                }
            } catch (e) {
                this.savedLocations = [];
            }
        },
        
        persistLocations() {
            localStorage.setItem('drone_saved_locations', JSON.stringify(this.savedLocations));
        },
        
        saveCurrentLocation() {
            const name = this.newLocationName.trim();
            if (!name) {
                this.addLog('Please enter a name for this location', 'warning');
                return;
            }
            const lat = parseFloat(this.navConfig.target_lat);
            const lon = parseFloat(this.navConfig.target_lon);
            const alt = parseFloat(this.navConfig.target_alt) || 2.0;
            if (isNaN(lat) || isNaN(lon)) {
                this.addLog('Please enter valid coordinates first', 'warning');
                return;
            }
            const entry = { name: name, lat: lat, lon: lon, alt: alt };
            // Check duplicate name
            const exists = this.savedLocations.findIndex(loc => loc.name === name);
            if (exists >= 0) {
                this.savedLocations[exists] = entry;
                this.addLog(`Updated: ${name} → (${lat}, ${lon}), alt=${alt}m`, 'info');
            } else {
                this.savedLocations.push(entry);
                this.addLog(`Saved: ${name} → (${lat}, ${lon}), alt=${alt}m`, 'success');
            }
            this.persistLocations();
            this.newLocationName = '';
        },
        
        selectLocation() {
            const idx = this.selectedLocationIdx;
            if (idx < 0 || idx >= this.savedLocations.length) return;
            const loc = this.savedLocations[idx];
            this.navConfig.target_lat = loc.lat;
            this.navConfig.target_lon = loc.lon;
            this.navConfig.target_alt = loc.alt;
            this.addLog(`Loaded location: ${loc.name}`, 'info');
        },
        
        deleteSelectedLocation() {
            const idx = this.selectedLocationIdx;
            if (idx < 0 || idx >= this.savedLocations.length) return;
            const name = this.savedLocations[idx].name;
            this.savedLocations.splice(idx, 1);
            this.persistLocations();
            this.selectedLocationIdx = -1;
            this.addLog(`Deleted location: ${name}`, 'info');
        },
        
        saveCurrentPositionAsLocation() {
            // Save drone's current GPS as a location
            if (!this.drone.latitude || !this.drone.longitude) {
                this.addLog('No GPS data available', 'warning');
                return;
            }
            const name = prompt('Enter a name for current drone position:');
            if (!name || !name.trim()) return;
            this.savedLocations.push({
                name: name.trim(),
                lat: this.drone.latitude,
                lon: this.drone.longitude,
                alt: this.navConfig.target_alt
            });
            this.persistLocations();
            this.addLog(`Saved current position as: ${name.trim()}`, 'success');
        },
        
        // ========== Navigation ==========
        
        async startNavigation() {
            const lat = parseFloat(this.navConfig.target_lat);
            const lon = parseFloat(this.navConfig.target_lon);
            if (isNaN(lat) || isNaN(lon)) {
                this.addLog('Please enter valid GPS coordinates', 'error');
                return;
            }
            
            this.addLog(`GPS nav to (${this.navConfig.target_lat.toFixed(6)}, ${this.navConfig.target_lon.toFixed(6)})`, 'info');
            const result = await this.apiCall('/api/navigation/start', 'POST', this.navConfig);
            
            if (result.success) {
                this.addLog('Navigation started', 'success');
            } else {
                this.addLog('Navigation failed to start', 'error');
            }
        },
        
        async stopNavigation() {
            this.addLog('Stopping navigation...', 'info');
            const result = await this.apiCall('/api/navigation/stop');
            if (result.success) {
                this.addLog('Navigation stop requested', 'info');
            }
        },
        
        // ========== Perception ==========
        
        async updatePerceptionConfig() {
            const config = {
                mode: this.perceptionConfig.mode,
                color: this.perceptionConfig.color,
                classes: this.perceptionConfig.classes_str.split(',').map(s => s.trim())
            };
            
            await this.apiCall('/api/perception/config', 'POST', config);
            this.addLog(`Perception config updated: ${config.mode}`, 'info');
        },
        
        async startPerception() {
            await this.updatePerceptionConfig();
            this.addLog('Starting object tracking', 'info');
            const result = await this.apiCall('/api/perception/start');
            
            if (result.success) {
                this.addLog('Object tracking started', 'success');
            } else {
                this.addLog('Object tracking failed', 'error');
            }
        },
        
        async stopPerception() {
            this.addLog('Stopping tracking', 'info');
            await this.apiCall('/api/perception/stop');
        },
        
        // ========== Module Tests ==========
        
        async testNavigation() {
            if (confirm('Test Navigation module?')) {
                this.addLog('Testing Navigation module...', 'info');
                const result = await this.apiCall('/api/test/navigation');
                if (result.success) {
                    this.addLog('Navigation test completed', 'success');
                } else {
                    this.addLog('Navigation test failed', 'error');
                }
            }
        },
        
        async testPerception() {
            if (confirm('Test Perception module?')) {
                this.addLog('Testing Perception module...', 'info');
                const result = await this.apiCall('/api/test/perception');
                if (result.success) {
                    this.addLog('Perception test completed', 'success');
                } else {
                    this.addLog('Perception test failed', 'error');
                }
            }
        },
        
        async testWinch() {
            if (confirm('Test Winch System? (LOWER → PULL → STOP)')) {
                this.addLog('Testing Winch System...', 'info');
                const result = await this.apiCall('/api/test/winch');
                if (result.success) {
                    this.addLog('Winch System test completed', 'success');
                } else {
                    this.addLog('Winch System test failed', 'error');
                }
            }
        },
        
        // ========== Helpers ==========
        
        getStatusClass(state) {
            return `status-${state}`;
        },
        
        getStatusText(state) {
            const statusMap = {
                'idle': 'Ready',
                'connecting': 'Connecting',
                'connected': 'Connected',
                'running': 'Running',
                'paused': 'Paused',
                'error': 'Error',
                'completed': 'Completed'
            };
            return statusMap[state] || state;
        },
        
        getWinchActionClass(action) {
            const actionMap = {
                'IDLE': 'winch-idle',
                'LOWERING': 'winch-lowering',
                'PULLING': 'winch-pulling',
                'STOP': 'winch-completed',
                'COMPLETED': 'winch-completed'
            };
            return actionMap[action] || 'winch-idle';
        },
        
        getWinchIcon(action) {
            const iconMap = {
                'IDLE': '⏸️',
                'LOWERING': '⬇️',
                'PULLING': '⬆️',
                'STOP': '⏹️',
                'COMPLETED': '✅'
            };
            return iconMap[action] || '⏸️';
        },
        
        // ========== Manual Control ==========
        
        async startManualControl(axis, value) {
            this.currentControl = { axis, value };
            
            // Send immediately
            await this.sendManualControl(axis, value);
            
            // Keep sending every 100ms
            this.manualControlInterval = setInterval(() => {
                this.sendManualControl(axis, value);
            }, 100);
        },
        
        stopManualControl() {
            if (this.manualControlInterval) {
                clearInterval(this.manualControlInterval);
                this.manualControlInterval = null;
            }
            
            // Send stop
            if (this.currentControl.axis) {
                this.sendManualControl(this.currentControl.axis, 0);
                this.currentControl = { axis: null, value: 0 };
            }
        },
        
        async sendManualControl(axis, value) {
            try {
                await this.apiCall('/api/manual_control', 'POST', {
                    axis: axis,
                    value: value
                });
            } catch (error) {
                console.error('Manual control error:', error);
            }
        }
    }
}).mount('#app');
