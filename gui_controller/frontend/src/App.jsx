import React, { useState, useEffect, useRef } from 'react';
import { Camera, Send, PlayCircle, Terminal, Wifi, WifiOff, Circle } from 'lucide-react';

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
  const wsRef = useRef(null);

  const rtspPresets = [
    'rtsp://172.17.0.1:8554/robotcam',
    'rtsp://172.17.0.1:8554/frontcam',
    'rtsp://172.17.0.1:8554/backcam',
    'rtsp://192.168.1.100:8554/camera1',
    'custom'
  ];

  // WebSocket connection
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
    
    addMessage('user', prompt);
    
    const finalRtsp = rtspUrl === 'custom' ? customRtsp : rtspUrl;
    
    try {
      const res = await fetch(`${API_URL}/api/command/execute`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          prompt,
          mission_type: mission,
          rtsp_url: finalRtsp,
          duration
        })
      });
      
      const data = await res.json();
      addMessage('assistant', data.message);
      setPrompt('');
    } catch (err) {
      addMessage('assistant', `❌ Error: ${err.message}`);
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
            </div>
          </div>
        </div>

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
                    onKeyPress={(e) => e.key === 'Enter' && sendCommand()}
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