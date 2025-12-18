import React, { useState, useEffect, useRef } from 'react';
import { Camera, Send, PlayCircle, Terminal, Wifi, WifiOff, Circle, Navigation, Radar, Map } from 'lucide-react';

const API_URL = 'http://localhost:8000';

export default function RobotControlUI() {
  const [mission, setMission] = useState('explore');
  const [rtspUrl, setRtspUrl] = useState('rtsp://172.17.0.1:8554/robotcam');
  const [customRtsp, setCustomRtsp] = useState('');
  const [duration, setDuration] = useState(60);
  const [prompt, setPrompt] = useState('');
  const [messages, setMessages] = useState([]);
  const [isConnected, setIsConnected] = useState(false);
  const [dockerStatus, setDockerStatus] = useState('checking');
  const [missionStatus, setMissionStatus] = useState('idle');
  
  // CHANGED: Add telemetry state
  const [telemetryActive, setTelemetryActive] = useState(false);
  const [odomData, setOdomData] = useState({ x: 0, y: 0, theta: 0, linear_vel: 0, angular_vel: 0 });
  const [scanData, setScanData] = useState({ ranges: [], angle_min: 0, angle_max: 0 });
  const [robotPath, setRobotPath] = useState([]); // Store trajectory
  const [showTelemetry, setShowTelemetry] = useState(false); // Toggle panel
  
  const wsRef = useRef(null);
  const telemetryWsRef = useRef(null); // CHANGED: Separate WebSocket for telemetry
  const canvasRef = useRef(null); // For LIDAR radar

  const rtspPresets = [
    'rtsp://172.17.0.1:8554/robotcam',
    'rtsp://172.17.0.1:8554/frontcam',
    'rtsp://172.17.0.1:8554/backcam',
    'rtsp://192.168.1.100:8554/camera1',
    'custom'
  ];

  // WebSocket connection (command channel)
  useEffect(() => {
    const ws = new WebSocket(`ws://localhost:8000/ws`);
    
    ws.onopen = () => {
      setIsConnected(true);
      console.log('WebSocket connected');
    };
    
    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.type === 'command_executed') {
        addMessage('assistant', `✅ Command executed: ${data.data.command}`);
      } else if (data.type === 'mission_started') {
        setMissionStatus('running');
        addMessage('system', `🚀 Mission ${data.mission} started`);
      }
    };
    
    ws.onclose = () => {
      setIsConnected(false);
      console.log('WebSocket disconnected');
    };
    
    wsRef.current = ws;
    
    return () => ws.close();
  }, []);

  // CHANGED: Telemetry WebSocket connection
  useEffect(() => {
    if (!telemetryActive) return;
    
    const telemetryWs = new WebSocket(`ws://localhost:8000/ws/telemetry`);
    
    telemetryWs.onopen = () => {
      console.log('Telemetry WebSocket connected');
    };
    
    telemetryWs.onmessage = (event) => {
      const data = JSON.parse(event.data);
      
      if (data.type === 'odom') {
        setOdomData(data.data);
        // CHANGED: Add to trajectory (keep last 100 points)
        setRobotPath(prev => [...prev.slice(-99), { x: data.data.x, y: data.data.y }]);
      } else if (data.type === 'scan') {
        setScanData(data.data);
      }
    };
    
    telemetryWs.onclose = () => {
      console.log('Telemetry WebSocket disconnected');
    };
    
    telemetryWsRef.current = telemetryWs;
    
    return () => telemetryWs.close();
  }, [telemetryActive]);

  // CHANGED: Draw LIDAR radar on canvas
  useEffect(() => {
    if (!canvasRef.current || !scanData.ranges.length) return;
    
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;
    const maxRadius = Math.min(centerX, centerY) - 20;
    
    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Draw background circles (grid)
    ctx.strokeStyle = 'rgba(100, 100, 255, 0.2)';
    ctx.lineWidth = 1;
    for (let r = maxRadius / 4; r <= maxRadius; r += maxRadius / 4) {
      ctx.beginPath();
      ctx.arc(centerX, centerY, r, 0, 2 * Math.PI);
      ctx.stroke();
    }
    
    // Draw LIDAR points
    const angleStep = (scanData.angle_max - scanData.angle_min) / scanData.ranges.length;
    
    scanData.ranges.forEach((range, i) => {
      if (range <= 0 || range === Infinity) return; // Skip invalid
      
      const angle = scanData.angle_min + i * angleStep;
      const normalizedRange = Math.min(range / scanData.range_max, 1);
      const radius = normalizedRange * maxRadius;
      
      const x = centerX + radius * Math.cos(angle - Math.PI / 2);
      const y = centerY + radius * Math.sin(angle - Math.PI / 2);
      
      // Color based on distance (red=close, green=far)
      const color = range < 0.5 ? 'rgb(255, 50, 50)' : 
                    range < 1.0 ? 'rgb(255, 200, 50)' : 
                    'rgb(50, 255, 100)';
      
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(x, y, 3, 0, 2 * Math.PI);
      ctx.fill();
    });
    
    // Draw robot center
    ctx.fillStyle = 'rgba(100, 150, 255, 0.8)';
    ctx.beginPath();
    ctx.arc(centerX, centerY, 8, 0, 2 * Math.PI);
    ctx.fill();
    
    // Draw direction indicator
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.9)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(centerX, centerY - 15);
    ctx.stroke();
    
  }, [scanData]);

  // Check Docker status periodically
  useEffect(() => {
    const checkStatus = async () => {
      try {
        const res = await fetch(`${API_URL}/api/docker/status`);
        const data = await res.json();
        setDockerStatus(data.status);
      } catch (err) {
        setDockerStatus('error');
      }
    };
    
    checkStatus();
    const interval = setInterval(checkStatus, 5000);
    return () => clearInterval(interval);
  }, []);

  const addMessage = (role, content) => {
    setMessages(prev => [...prev, { role, content, timestamp: new Date() }]);
  };

  const startMission = async () => {
    try {
      setMissionStatus('starting');
      const res = await fetch(`${API_URL}/api/mission/start`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mission_type: mission })
      });
      
      if (res.ok) {
        addMessage('system', `✅ Mission ${mission} started successfully`);
        setMissionStatus('running');
      }
    } catch (err) {
      addMessage('system', `❌ Failed to start mission: ${err.message}`);
      setMissionStatus('error');
    }
  };

  const sendCommand = async () => {
    if (!prompt.trim()) return;
    
    const originalPrompt = prompt;
    addMessage('user', originalPrompt);
    setPrompt('');
    
    const finalRtsp = rtspUrl === 'custom' ? customRtsp : rtspUrl;
    
    try {
      // Send initial command
      const res = await fetch(`${API_URL}/api/command/execute`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          prompt: originalPrompt,
          mission_type: mission,
          rtsp_url: finalRtsp,
          duration
        })
      });
      
      const data = await res.json();
      
      // Check if clarification needed
      if (data.status === 'needs_clarification' && data.question) {
        addMessage('system', '🤔 Mission needs clarification...');
        
        try {
          // Wait for user response via modal
          const response = await askClarification(data.question, originalPrompt);
          
          addMessage('user', `Clarification: ${response.user_response}`);
          
          // Send clarified command
          const clarifiedRes = await fetch(`${API_URL}/api/command/execute`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
              prompt: originalPrompt,
              clarification: response.user_response,
              mission_type: mission,
              rtsp_url: finalRtsp,
              duration
            })
          });
          
          const clarifiedData = await clarifiedRes.json();
          addMessage('assistant', clarifiedData.message);
          
        } catch (clarificationError) {
          addMessage('system', '❌ Mission cancelled');
        }
      } else {
        // No clarification needed
        addMessage('assistant', data.message);
      }
      
    } catch (err) {
      addMessage('assistant', `❌ Error: ${err.message}`);
    }
  };

  // CHANGED: Toggle telemetry streaming
  const toggleTelemetry = async () => {
    try {
      const res = await fetch(`${API_URL}/api/telemetry/control`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enabled: !telemetryActive })
      });
      
      if (res.ok) {
        setTelemetryActive(!telemetryActive);
        setShowTelemetry(!telemetryActive);
        if (!telemetryActive) {
          addMessage('system', '📡 Telemetry streaming started');
        } else {
          addMessage('system', '📡 Telemetry streaming stopped');
        }
      }
    } catch (err) {
      addMessage('system', `❌ Telemetry error: ${err.message}`);
    }
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-purple-900 to-slate-900 text-white p-4">
      {/* Animated background */}
      <div className="fixed inset-0 overflow-hidden pointer-events-none">
        <div className="absolute top-20 left-20 w-72 h-72 bg-purple-500 rounded-full mix-blend-multiply filter blur-xl opacity-20 animate-pulse"></div>
        <div className="absolute top-40 right-20 w-72 h-72 bg-blue-500 rounded-full mix-blend-multiply filter blur-xl opacity-20 animate-pulse delay-700"></div>
        <div className="absolute bottom-20 left-1/2 w-72 h-72 bg-pink-500 rounded-full mix-blend-multiply filter blur-xl opacity-20 animate-pulse delay-1000"></div>
      </div>

      <div className="max-w-7xl mx-auto relative z-10">
        {/* Header */}
        <div className="text-center mb-8 animate-fade-in">
          <h1 className="text-5xl font-bold mb-2 bg-clip-text text-transparent bg-gradient-to-r from-purple-400 to-pink-600">
            🤖 Robot Control Interface
          </h1>
          <p className="text-gray-400">Advanced AI-Powered Robot Control System</p>
        </div>

        {/* Status Bar */}
        <div className="bg-slate-800/50 backdrop-blur-lg rounded-2xl p-4 mb-6 border border-slate-700/50 shadow-2xl">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-4">
              <div className="flex items-center gap-2">
                {isConnected ? (
                  <Wifi className="w-5 h-5 text-green-400 animate-pulse" />
                ) : (
                  <WifiOff className="w-5 h-5 text-red-400" />
                )}
                <span className="text-sm">
                  {isConnected ? 'Connected' : 'Disconnected'}
                </span>
              </div>
              
              <div className="flex items-center gap-2">
                <Circle className={`w-3 h-3 ${
                  dockerStatus === 'running' ? 'text-green-400 animate-pulse' :
                  dockerStatus === 'stopped' ? 'text-yellow-400' : 'text-red-400'
                }`} fill="currentColor" />
                <span className="text-sm">Docker: {dockerStatus}</span>
              </div>
              
              <div className="flex items-center gap-2">
                <Circle className={`w-3 h-3 ${
                  missionStatus === 'running' ? 'text-green-400 animate-pulse' :
                  missionStatus === 'starting' ? 'text-yellow-400 animate-pulse' : 'text-gray-400'
                }`} fill="currentColor" />
                <span className="text-sm">Mission: {missionStatus}</span>
              </div>

              {/* CHANGED: Telemetry status */}
              <div className="flex items-center gap-2">
                <Radar className={`w-5 h-5 ${telemetryActive ? 'text-blue-400 animate-pulse' : 'text-gray-500'}`} />
                <span className="text-sm">Telemetry: {telemetryActive ? 'Active' : 'Off'}</span>
              </div>
            </div>

            {/* CHANGED: Telemetry toggle button */}
            <button
              onClick={toggleTelemetry}
              className={`px-4 py-2 rounded-lg font-semibold transition-all ${
                telemetryActive 
                  ? 'bg-red-600 hover:bg-red-500' 
                  : 'bg-blue-600 hover:bg-blue-500'
              }`}
            >
              {telemetryActive ? '📡 Stop Telemetry' : '📡 Start Telemetry'}
            </button>
          </div>
        </div>

        {/* Telemetry Dashboard (conditionally shown) */}
        {showTelemetry && (
          <div className="bg-slate-800/50 backdrop-blur-lg rounded-2xl p-6 mb-6 border border-slate-700/50 shadow-2xl">
            <h2 className="text-2xl font-bold mb-4 flex items-center gap-2">
              <Map className="w-6 h-6 text-green-400" />
              Robot Telemetry Dashboard
            </h2>
            
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
              {/* Odometry Panel */}
              <div className="bg-slate-900/50 rounded-xl p-4 border border-slate-600/50">
                <h3 className="text-lg font-bold mb-3 flex items-center gap-2">
                  <Navigation className="w-5 h-5 text-purple-400" />
                  Position & Velocity
                </h3>
                
                <div className="grid grid-cols-2 gap-4">
                  {/* Position Display */}
                  <div className="bg-slate-800/50 rounded-lg p-3">
                    <div className="text-xs text-gray-400 mb-1">Position (m)</div>
                    <div className="text-2xl font-mono font-bold text-green-400">
                      X: {odomData.x.toFixed(2)}
                    </div>
                    <div className="text-2xl font-mono font-bold text-blue-400">
                      Y: {odomData.y.toFixed(2)}
                    </div>
                  </div>

                  {/* Compass */}
                  <div className="bg-slate-800/50 rounded-lg p-3 flex flex-col items-center justify-center">
                    <div className="text-xs text-gray-400 mb-2">Heading</div>
                    <div className="relative w-20 h-20">
                      {/* Compass circle */}
                      <div className="absolute inset-0 border-4 border-purple-500/30 rounded-full"></div>
                      {/* Direction arrow */}
                      <div 
                        className="absolute inset-0 flex items-center justify-center"
                        style={{ transform: `rotate(${odomData.theta * 180 / Math.PI}deg)` }}
                      >
                        <div className="w-1 h-8 bg-gradient-to-t from-purple-600 to-pink-500 rounded-full"></div>
                      </div>
                    </div>
                    <div className="text-sm font-mono mt-1">{(odomData.theta * 180 / Math.PI).toFixed(0)}°</div>
                  </div>

                  {/* Linear Velocity Gauge */}
                  <div className="bg-slate-800/50 rounded-lg p-3">
                    <div className="text-xs text-gray-400 mb-1">Linear Vel (m/s)</div>
                    <div className="relative pt-1">
                      <div className="flex mb-2 items-center justify-between">
                        <div className="text-xl font-mono font-bold text-cyan-400">
                          {odomData.linear_vel.toFixed(2)}
                        </div>
                      </div>
                      <div className="overflow-hidden h-2 text-xs flex rounded bg-slate-700">
                        <div 
                          style={{ width: `${Math.min(Math.abs(odomData.linear_vel) * 50, 100)}%` }}
                          className="shadow-none flex flex-col text-center whitespace-nowrap text-white justify-center bg-gradient-to-r from-cyan-500 to-blue-500"
                        ></div>
                      </div>
                    </div>
                  </div>

                  {/* Angular Velocity Gauge */}
                  <div className="bg-slate-800/50 rounded-lg p-3">
                    <div className="text-xs text-gray-400 mb-1">Angular Vel (rad/s)</div>
                    <div className="relative pt-1">
                      <div className="flex mb-2 items-center justify-between">
                        <div className="text-xl font-mono font-bold text-orange-400">
                          {odomData.angular_vel.toFixed(2)}
                        </div>
                      </div>
                      <div className="overflow-hidden h-2 text-xs flex rounded bg-slate-700">
                        <div 
                          style={{ width: `${Math.min(Math.abs(odomData.angular_vel) * 50, 100)}%` }}
                          className="shadow-none flex flex-col text-center whitespace-nowrap text-white justify-center bg-gradient-to-r from-orange-500 to-red-500"
                        ></div>
                      </div>
                    </div>
                  </div>
                </div>

                {/* 2D Trajectory Path */}
                <div className="mt-4 bg-slate-800/50 rounded-lg p-3">
                  <div className="text-xs text-gray-400 mb-2">Trajectory (Last 100 points)</div>
                  <svg width="100%" height="150" className="bg-slate-900/50 rounded">
                    {/* Draw grid */}
                    <defs>
                      <pattern id="grid" width="20" height="20" patternUnits="userSpaceOnUse">
                        <path d="M 20 0 L 0 0 0 20" fill="none" stroke="rgba(100,100,255,0.1)" strokeWidth="0.5"/>
                      </pattern>
                    </defs>
                    <rect width="100%" height="100%" fill="url(#grid)" />
                    
                    {/* Draw path */}
                    {robotPath.length > 1 && (
                      <polyline
                        points={robotPath.map((p, i) => {
                          const scale = 30;
                          const offsetX = 200;
                          const offsetY = 75;
                          return `${offsetX + p.x * scale},${offsetY - p.y * scale}`;
                        }).join(' ')}
                        fill="none"
                        stroke="rgba(100,200,255,0.8)"
                        strokeWidth="2"
                      />
                    )}
                    
                    {/* Draw current position */}
                    {robotPath.length > 0 && (
                      <circle
                        cx={200 + robotPath[robotPath.length - 1].x * 30}
                        cy={75 - robotPath[robotPath.length - 1].y * 30}
                        r="4"
                        fill="#ff6b6b"
                      />
                    )}
                  </svg>
                </div>
              </div>

              {/* LIDAR Panel */}
              <div className="bg-slate-900/50 rounded-xl p-4 border border-slate-600/50">
                <h3 className="text-lg font-bold mb-3 flex items-center gap-2">
                  <Radar className="w-5 h-5 text-cyan-400" />
                  LIDAR Radar (360°)
                </h3>
                
                <canvas 
                  ref={canvasRef}
                  width={400}
                  height={400}
                  className="w-full rounded-lg bg-slate-900/50"
                />

                {/* Obstacle warnings */}
                <div className="mt-3 grid grid-cols-3 gap-2 text-xs">
                  {scanData.ranges.filter(r => r > 0 && r < 0.5).length > 0 && (
                    <div className="bg-red-900/50 border border-red-500 rounded px-2 py-1 text-center">
                      ⚠️ Close: {scanData.ranges.filter(r => r > 0 && r < 0.5).length}
                    </div>
                  )}
                  {scanData.ranges.filter(r => r >= 0.5 && r < 1.0).length > 0 && (
                    <div className="bg-yellow-900/50 border border-yellow-500 rounded px-2 py-1 text-center">
                      ⚡ Near: {scanData.ranges.filter(r => r >= 0.5 && r < 1.0).length}
                    </div>
                  )}
                  <div className="bg-green-900/50 border border-green-500 rounded px-2 py-1 text-center">
                    ✅ Clear: {scanData.ranges.filter(r => r >= 1.0).length}
                  </div>
                </div>
              </div>
            </div>
          </div>
        )}

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          {/* Left Panel - Mission Control */}
          <div className="lg:col-span-1 space-y-6">
            <div className="bg-slate-800/50 backdrop-blur-lg rounded-2xl p-6 border border-slate-700/50 shadow-2xl transform transition-all hover:scale-105 hover:shadow-purple-500/20">
              <h2 className="text-2xl font-bold mb-4 flex items-center gap-2">
                <PlayCircle className="w-6 h-6 text-purple-400" />
                Mission Control
              </h2>
              
              <div className="space-y-4">
                <div>
                  <label className="block text-sm font-medium mb-2">Mission Type</label>
                  <div className="grid grid-cols-3 gap-2">
                    {['explore', 'patrol', 'follow'].map((m) => (
                      <button
                        key={m}
                        onClick={() => setMission(m)}
                        className={`p-3 rounded-xl font-semibold transition-all transform hover:scale-105 ${
                          mission === m
                            ? 'bg-gradient-to-r from-purple-600 to-pink-600 shadow-lg shadow-purple-500/50'
                            : 'bg-slate-700/50 hover:bg-slate-600/50'
                        }`}
                      >
                        {m}
                      </button>
                    ))}
                  </div>
                </div>
                
                <button
                  onClick={startMission}
                  disabled={missionStatus === 'starting'}
                  className="w-full bg-gradient-to-r from-green-600 to-emerald-600 hover:from-green-500 hover:to-emerald-500 text-white font-bold py-4 rounded-xl transition-all transform hover:scale-105 shadow-lg shadow-green-500/50 disabled:opacity-50"
                >
                  {missionStatus === 'starting' ? '🔄 Starting...' : '🚀 Start Mission Stack'}
                </button>
              </div>
            </div>

            {/* Camera Config */}
            <div className="bg-slate-800/50 backdrop-blur-lg rounded-2xl p-6 border border-slate-700/50 shadow-2xl">
              <h3 className="text-xl font-bold mb-4 flex items-center gap-2">
                <Camera className="w-5 h-5 text-blue-400" />
                Camera Config
              </h3>
              
              <div className="space-y-4">
                <div>
                  <label className="block text-sm font-medium mb-2">RTSP URL</label>
                  <select
                    value={rtspUrl}
                    onChange={(e) => setRtspUrl(e.target.value)}
                    className="w-full bg-slate-700/50 border border-slate-600 rounded-lg px-3 py-2 focus:ring-2 focus:ring-purple-500 focus:outline-none"
                  >
                    {rtspPresets.map((url) => (
                      <option key={url} value={url}>
                        {url === 'custom' ? '🔧 Custom URL...' : url}
                      </option>
                    ))}
                  </select>
                </div>
                
                {rtspUrl === 'custom' && (
                  <input
                    type="text"
                    value={customRtsp}
                    onChange={(e) => setCustomRtsp(e.target.value)}
                    placeholder="rtsp://your-custom-url:port/stream"
                    className="w-full bg-slate-700/50 border border-slate-600 rounded-lg px-3 py-2 focus:ring-2 focus:ring-purple-500 focus:outline-none"
                  />
                )}
                
                <div>
                  <label className="block text-sm font-medium mb-2">
                    Duration: {duration}s
                  </label>
                  <input
                    type="range"
                    min="5"
                    max="300"
                    step="5"
                    value={duration}
                    onChange={(e) => setDuration(Number(e.target.value))}
                    className="w-full accent-purple-500"
                  />
                </div>
              </div>
            </div>
          </div>

          {/* Right Panel - Chat Interface */}
          <div className="lg:col-span-2">
            <div className="bg-slate-800/50 backdrop-blur-lg rounded-2xl border border-slate-700/50 shadow-2xl h-[600px] flex flex-col">
              <div className="p-6 border-b border-slate-700/50">
                <h2 className="text-2xl font-bold flex items-center gap-2">
                  <Terminal className="w-6 h-6 text-green-400" />
                  Command Interface
                </h2>
              </div>
              
              {/* Messages */}
              <div className="flex-1 overflow-y-auto p-6 space-y-4">
                {messages.length === 0 && (
                  <div className="text-center text-gray-500 mt-20">
                    <Terminal className="w-16 h-16 mx-auto mb-4 opacity-50" />
                    <p>Start sending commands to your robot...</p>
                  </div>
                )}
                
                {messages.map((msg, idx) => (
                  <div
                    key={idx}
                    className={`flex ${msg.role === 'user' ? 'justify-end' : 'justify-start'} animate-fade-in`}
                  >
                    <div className={`max-w-[80%] rounded-2xl p-4 ${
                      msg.role === 'user'
                        ? 'bg-gradient-to-r from-purple-600 to-pink-600 shadow-lg shadow-purple-500/30'
                        : msg.role === 'system'
                        ? 'bg-blue-600/20 border border-blue-500/50'
                        : 'bg-slate-700/50'
                    }`}>
                      <p className="text-sm">{msg.content}</p>
                      <p className="text-xs text-gray-400 mt-1">
                        {msg.timestamp.toLocaleTimeString()}
                      </p>
                    </div>
                  </div>
                ))}
              </div>
              
              {/* Input */}
              <div className="p-6 border-t border-slate-700/50">
                <div className="flex gap-3">
                  <input
                    type="text"
                    value={prompt}
                    onChange={(e) => setPrompt(e.target.value)}
                    onKeyDown={(e) => e.key === 'Enter' && sendCommand()}
                    placeholder="Enter command (e.g., explore, move forward 3 meters...)"
                    className="flex-1 bg-slate-700/50 border border-slate-600 rounded-xl px-4 py-3 focus:ring-2 focus:ring-purple-500 focus:outline-none"
                  />
                  <button
                    onClick={sendCommand}
                    disabled={!prompt.trim()}
                    className="bg-gradient-to-r from-purple-600 to-pink-600 hover:from-purple-500 hover:to-pink-500 text-white px-6 py-3 rounded-xl font-semibold transition-all transform hover:scale-105 shadow-lg shadow-purple-500/50 disabled:opacity-50 disabled:cursor-not-allowed flex items-center gap-2"
                  >
                    <Send className="w-5 h-5" />
                    Send
                  </button>
                </div>

                {/* Quick Commands */}
                <div className="flex flex-wrap gap-2 mt-3">
                  {['explore', 'patrol area', 'move forward', 'turn left', 'stop'].map((cmd) => (
                    <button
                      key={cmd}
                      onClick={() => setPrompt(cmd)}
                      className="px-3 py-1 bg-slate-700/50 hover:bg-slate-600/50 rounded-lg text-sm transition-all transform hover:scale-105"
                    >
                      {cmd}
                    </button>
                  ))}
                </div>
              </div>
            </div>
          </div>

          {/* Closing tags và CSS */}
          </div>
        </div>

        <style jsx>{`
          @keyframes fade-in {
            from {
              opacity: 0;
              transform: translateY(10px);
            }
            to {
              opacity: 1;
              transform: translateY(0);
            }
          }
          .animate-fade-in {
            animation: fade-in 0.5s ease-out;
          }
        `}</style>
      </div>
    );
}