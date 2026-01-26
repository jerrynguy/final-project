import React, { useEffect, useRef, useState } from 'react';
import { Download, ZoomIn, ZoomOut, RotateCcw, Maximize2 } from 'lucide-react';

export default function TrajectoryViewer({ odomData, plannedPath = null, isFullscreen = false, onToggleFullscreen }) {
  const canvasRef = useRef(null);
  const [robotHistory, setRobotHistory] = useState([]);
  const [scale, setScale] = useState(50); // pixels per meter
  const [offset, setOffset] = useState({ x: 400, y: 400 });
  const [isPanning, setIsPanning] = useState(false);
  const [panStart, setPanStart] = useState({ x: 0, y: 0 });
  const [stats, setStats] = useState({
    distance: 0,
    maxSpeed: 0,
    avgSpeed: 0
  });

  // Add current position to history
  useEffect(() => {
    if (odomData && odomData.x !== undefined) {
      setRobotHistory(prev => {
        const newHistory = [...prev, { 
          x: odomData.x, 
          y: odomData.y,
          timestamp: Date.now()
        }];
        
        // Keep last 1000 points
        if (newHistory.length > 1000) {
          return newHistory.slice(-1000);
        }
        return newHistory;
      });
    }
  }, [odomData.x, odomData.y]);

  // Calculate statistics
  useEffect(() => {
    if (robotHistory.length < 2) return;
    
    let totalDistance = 0;
    let maxSpeed = 0;
    
    for (let i = 1; i < robotHistory.length; i++) {
      const prev = robotHistory[i - 1];
      const curr = robotHistory[i];
      
      const dx = curr.x - prev.x;
      const dy = curr.y - prev.y;
      const dist = Math.sqrt(dx * dx + dy * dy);
      totalDistance += dist;
      
      const dt = (curr.timestamp - prev.timestamp) / 1000; // seconds
      if (dt > 0) {
        const speed = dist / dt;
        maxSpeed = Math.max(maxSpeed, speed);
      }
    }
    
    const duration = (robotHistory[robotHistory.length - 1].timestamp - robotHistory[0].timestamp) / 1000;
    const avgSpeed = duration > 0 ? totalDistance / duration : 0;
    
    setStats({
      distance: totalDistance,
      maxSpeed,
      avgSpeed
    });
  }, [robotHistory]);

  // Draw canvas
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    
    // Clear
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(0, 0, width, height);
    
    // Draw grid
    ctx.strokeStyle = 'rgba(100, 100, 255, 0.1)';
    ctx.lineWidth = 1;
    
    const gridSize = scale; // 1 meter
    for (let x = offset.x % gridSize; x < width; x += gridSize) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, height);
      ctx.stroke();
    }
    for (let y = offset.y % gridSize; y < height; y += gridSize) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(width, y);
      ctx.stroke();
    }
    
    // Draw axes
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, offset.y);
    ctx.lineTo(width, offset.y);
    ctx.moveTo(offset.x, 0);
    ctx.lineTo(offset.x, height);
    ctx.stroke();
    
    // Helper: world to screen coordinates
    const worldToScreen = (x, y) => ({
      x: offset.x + x * scale,
      y: offset.y - y * scale // Y axis inverted
    });
    
    // Draw planned path if available
    if (plannedPath && plannedPath.length > 0) {
      ctx.strokeStyle = 'rgba(255, 200, 50, 0.5)';
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      
      plannedPath.forEach((wp, i) => {
        const screen = worldToScreen(wp.x, wp.y);
        if (i === 0) {
          ctx.moveTo(screen.x, screen.y);
        } else {
          ctx.lineTo(screen.x, screen.y);
        }
      });
      ctx.stroke();
      ctx.setLineDash([]);
      
      // Draw waypoint markers
      ctx.fillStyle = 'rgba(255, 200, 50, 0.6)';
      plannedPath.forEach((wp, i) => {
        const screen = worldToScreen(wp.x, wp.y);
        ctx.beginPath();
        ctx.arc(screen.x, screen.y, 3, 0, 2 * Math.PI);
        ctx.fill();
      });
    }
    
    // Draw robot history path
    if (robotHistory.length > 1) {
      // Gradient from old (faint) to new (bright)
      ctx.lineWidth = 3;
      
      for (let i = 1; i < robotHistory.length; i++) {
        const prev = robotHistory[i - 1];
        const curr = robotHistory[i];
        
        const alpha = 0.3 + (0.7 * i / robotHistory.length);
        ctx.strokeStyle = `rgba(100, 200, 255, ${alpha})`;
        
        const prevScreen = worldToScreen(prev.x, prev.y);
        const currScreen = worldToScreen(curr.x, curr.y);
        
        ctx.beginPath();
        ctx.moveTo(prevScreen.x, prevScreen.y);
        ctx.lineTo(currScreen.x, currScreen.y);
        ctx.stroke();
      }
    }
    
    // Draw current robot position
    if (odomData && odomData.x !== undefined) {
      const screen = worldToScreen(odomData.x, odomData.y);
      
      // Robot body
      ctx.fillStyle = '#ff6b6b';
      ctx.beginPath();
      ctx.arc(screen.x, screen.y, 8, 0, 2 * Math.PI);
      ctx.fill();
      
      // Direction indicator
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      const dirLength = 15;
      const dirX = screen.x + dirLength * Math.cos(odomData.theta);
      const dirY = screen.y - dirLength * Math.sin(odomData.theta); // Y inverted
      
      ctx.beginPath();
      ctx.moveTo(screen.x, screen.y);
      ctx.lineTo(dirX, dirY);
      ctx.stroke();
      
      // Position label
      ctx.fillStyle = '#ffffff';
      ctx.font = '12px monospace';
      ctx.fillText(
        `(${odomData.x.toFixed(2)}, ${odomData.y.toFixed(2)})`,
        screen.x + 12,
        screen.y - 12
      );
    }
    
    // Draw scale reference
    ctx.fillStyle = 'rgba(255, 255, 255, 0.8)';
    ctx.font = '14px monospace';
    ctx.fillText(`Scale: ${(1 / scale * 100).toFixed(1)}m per 100px`, 10, height - 10);
    
  }, [robotHistory, odomData, plannedPath, scale, offset]);

  // Pan handlers
  const handleMouseDown = (e) => {
    setIsPanning(true);
    setPanStart({ x: e.clientX - offset.x, y: e.clientY - offset.y });
  };

  const handleMouseMove = (e) => {
    if (!isPanning) return;
    setOffset({
      x: e.clientX - panStart.x,
      y: e.clientY - panStart.y
    });
  };

  const handleMouseUp = () => {
    setIsPanning(false);
  };

  // Zoom handlers
  const handleWheel = (e) => {
    e.preventDefault();
    const delta = e.deltaY > 0 ? 0.9 : 1.1;
    setScale(prev => Math.max(10, Math.min(200, prev * delta)));
  };

  const zoomIn = () => setScale(prev => Math.min(200, prev * 1.2));
  const zoomOut = () => setScale(prev => Math.max(10, prev / 1.2));
  const resetView = () => {
    setScale(50);
    setOffset({ x: 400, y: 400 });
  };

  // Export trajectory
  const exportTrajectory = () => {
    const csv = robotHistory.map(p => 
      `${p.x.toFixed(4)},${p.y.toFixed(4)},${p.timestamp}`
    ).join('\n');
    
    const blob = new Blob([`x,y,timestamp\n${csv}`], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `trajectory_${Date.now()}.csv`;
    a.click();
  };

  const canvasSize = isFullscreen ? { width: 1200, height: 800 } : { width: 800, height: 600 };

  return (
    <div className={`bg-slate-900/50 rounded-xl p-4 border border-slate-600/50 ${isFullscreen ? 'fixed inset-4 z-50' : ''}`}>
      {/* Header */}
      <div className="flex items-center justify-between mb-3">
        <h3 className="text-lg font-bold flex items-center gap-2">
          🗺️ Trajectory Viewer
        </h3>
        <div className="flex gap-2">
          <button
            onClick={zoomIn}
            className="p-2 bg-slate-700/50 hover:bg-slate-600/50 rounded-lg transition-all"
            title="Zoom In"
          >
            <ZoomIn className="w-4 h-4" />
          </button>
          <button
            onClick={zoomOut}
            className="p-2 bg-slate-700/50 hover:bg-slate-600/50 rounded-lg transition-all"
            title="Zoom Out"
          >
            <ZoomOut className="w-4 h-4" />
          </button>
          <button
            onClick={resetView}
            className="p-2 bg-slate-700/50 hover:bg-slate-600/50 rounded-lg transition-all"
            title="Reset View"
          >
            <RotateCcw className="w-4 h-4" />
          </button>
          <button
            onClick={exportTrajectory}
            className="p-2 bg-blue-600 hover:bg-blue-500 rounded-lg transition-all"
            title="Export CSV"
          >
            <Download className="w-4 h-4" />
          </button>
          {onToggleFullscreen && (
            <button
              onClick={onToggleFullscreen}
              className="p-2 bg-purple-600 hover:bg-purple-500 rounded-lg transition-all"
              title="Toggle Fullscreen"
            >
              <Maximize2 className="w-4 h-4" />
            </button>
          )}
        </div>
      </div>

      {/* Canvas */}
      <canvas
        ref={canvasRef}
        width={canvasSize.width}
        height={canvasSize.height}
        className="rounded-lg cursor-move bg-slate-950"
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
        onWheel={handleWheel}
      />

      {/* Stats */}
      <div className="grid grid-cols-3 gap-3 mt-3">
        <div className="bg-slate-800/50 rounded-lg p-2">
          <div className="text-xs text-gray-400">Distance Traveled</div>
          <div className="text-lg font-bold text-cyan-400">{stats.distance.toFixed(2)}m</div>
        </div>
        <div className="bg-slate-800/50 rounded-lg p-2">
          <div className="text-xs text-gray-400">Avg Speed</div>
          <div className="text-lg font-bold text-green-400">{stats.avgSpeed.toFixed(2)}m/s</div>
        </div>
        <div className="bg-slate-800/50 rounded-lg p-2">
          <div className="text-xs text-gray-400">Max Speed</div>
          <div className="text-lg font-bold text-orange-400">{stats.maxSpeed.toFixed(2)}m/s</div>
        </div>
      </div>

      {/* Legend */}
      <div className="flex gap-4 mt-3 text-xs">
        <div className="flex items-center gap-2">
          <div className="w-3 h-3 rounded-full bg-cyan-400"></div>
          <span>Actual Path</span>
        </div>
        {plannedPath && (
          <div className="flex items-center gap-2">
            <div className="w-3 h-1 bg-yellow-400"></div>
            <span>Planned Path</span>
          </div>
        )}
        <div className="flex items-center gap-2">
          <div className="w-3 h-3 rounded-full bg-red-400"></div>
          <span>Current Position</span>
        </div>
      </div>
    </div>
  );
}