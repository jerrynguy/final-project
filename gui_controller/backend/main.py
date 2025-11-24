"""
Robot Control Backend - FastAPI + WebSocket
Install: pip install fastapi uvicorn websockets python-multipart
Run: uvicorn main:app --reload --host 0.0.0.0 --port 8000
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import subprocess
import asyncio
import os
from pathlib import Path
from typing import List, Optional
import json
from datetime import datetime

app = FastAPI(title="Robot Control API")

# CORS để React có thể connect
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Trong production nên chỉ định cụ thể
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Config
DOCKER_DIR = Path.home() / "nemo-agent-toolkit" / "docker"
CONFIG_FILE = "/workspace/mounted_code/src/multi_function_agent/configs/config.yml"

MISSION_SCRIPTS = {
    "explore": "~/start_robot_stack_explore.sh",
    "patrol": "~/start_robot_stack_patrol.sh",
    "follow": "~/start_robot_stack_follow.sh"
}

# WebSocket connections management
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except:
                pass

manager = ConnectionManager()

# Models
class MissionStart(BaseModel):
    mission_type: str

class RobotCommand(BaseModel):
    prompt: str
    mission_type: str
    rtsp_url: str
    duration: int

class CommandResponse(BaseModel):
    status: str
    message: str
    command: str
    timestamp: str

# Endpoints
@app.get("/")
async def root():
    return {"status": "Robot Control API Running", "version": "1.0"}

@app.get("/api/health")
async def health_check():
    """Check if backend is healthy"""
    return {
        "status": "healthy",
        "docker_dir_exists": DOCKER_DIR.exists(),
        "timestamp": datetime.now().isoformat()
    }

@app.get("/api/missions")
async def get_missions():
    """Get available missions"""
    return {
        "missions": list(MISSION_SCRIPTS.keys()),
        "scripts": MISSION_SCRIPTS
    }

@app.post("/api/mission/start")
async def start_mission(mission: MissionStart):
    """Start a mission stack"""
    script_path = os.path.expanduser(MISSION_SCRIPTS.get(mission.mission_type, ""))
    
    if not script_path or not os.path.exists(script_path):
        raise HTTPException(status_code=404, detail=f"Script not found: {script_path}")
    
    try:
        # Start mission stack in new terminal
        terminal_cmd = f"gnome-terminal -- bash -c '{script_path}; exec bash'"
        subprocess.Popen(terminal_cmd, shell=True)
        
        # Broadcast to all connected clients
        await manager.broadcast({
            "type": "mission_started",
            "mission": mission.mission_type,
            "timestamp": datetime.now().isoformat()
        })
        
        return {
            "status": "success",
            "message": f"Mission {mission.mission_type} started",
            "script": script_path
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/command/execute")
async def execute_command(cmd: RobotCommand):
    """Execute NAT command"""
    
    # Build full prompt
    full_prompt = f"Control robot using {cmd.rtsp_url} and {cmd.prompt} for {cmd.duration} seconds"
    
    # Create execution script
    container_script = f"""#!/bin/bash
cd {DOCKER_DIR}

# Check/start container
CONTAINER_ID=$(docker ps --filter "ancestor=nemo-agent-toolkit" --format "{{{{.ID}}}}" | head -n 1)

if [ -z "$CONTAINER_ID" ]; then
    echo "Starting new container..."
    ./run_hybrid_container.sh -d
    sleep 5
    CONTAINER_ID=$(docker ps --filter "ancestor=nemo-agent-toolkit" --format "{{{{.ID}}}}" | head -n 1)
fi

echo "Using container: $CONTAINER_ID"
echo "Executing NAT command..."

# Execute NAT (without -it for non-interactive automation)
docker exec "$CONTAINER_ID" bash -c "
    source /workspace/.venv/bin/activate && \\
    cd /workspace/mounted_code && \\
    uv pip install -e . > /dev/null 2>&1 && \\
    nat run --config_file {CONFIG_FILE} --input '{full_prompt}'
"

echo ""
echo "Command execution completed!"
"""
    
    script_path = "/tmp/run_nat_temp.sh"
    with open(script_path, "w") as f:
        f.write(container_script)
    os.chmod(script_path, 0o755)
    
    try:
        # Execute in new terminal
        terminal_cmd = f"gnome-terminal -- bash -c '{script_path}; echo \"\\n\\nPress Enter to close...\"; read'"
        subprocess.Popen(terminal_cmd, shell=True)
        
        response = CommandResponse(
            status="executing",
            message="Command sent to robot",
            command=full_prompt,
            timestamp=datetime.now().isoformat()
        )
        
        # Broadcast to WebSocket clients
        await manager.broadcast({
            "type": "command_executed",
            "data": response.dict()
        })
        
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/docker/status")
async def docker_status():
    """Check Docker container status"""
    try:
        result = subprocess.run(
            ["docker", "ps", "--filter", "ancestor=nemo-agent-toolkit", "--format", "{{.ID}}|{{.Status}}"],
            capture_output=True,
            text=True
        )
        
        if result.stdout.strip():
            container_info = result.stdout.strip().split("|")
            return {
                "status": "running",
                "container_id": container_info[0],
                "container_status": container_info[1] if len(container_info) > 1 else "unknown"
            }
        else:
            return {"status": "stopped"}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket for real-time updates"""
    await manager.connect(websocket)
    try:
        while True:
            # Keep connection alive and receive messages
            data = await websocket.receive_text()
            message = json.loads(data)
            
            # Echo back for now (can add more logic)
            await websocket.send_json({
                "type": "echo",
                "data": message,
                "timestamp": datetime.now().isoformat()
            })
    except WebSocketDisconnect:
        manager.disconnect(websocket)

# Terminal output streaming endpoint
@app.websocket("/ws/terminal/{session_id}")
async def terminal_stream(websocket: WebSocket, session_id: str):
    """Stream terminal output in real-time"""
    await websocket.accept()
    try:
        # This would stream actual terminal output
        # For now, send mock data
        while True:
            await asyncio.sleep(1)
            await websocket.send_json({
                "type": "terminal_output",
                "data": f"[{datetime.now().strftime('%H:%M:%S')}] Processing...",
                "session_id": session_id
            })
    except WebSocketDisconnect:
        pass

if __name__ == "__main__":
    import uvicorn
    print("🚀 Starting Robot Control Backend...")
    print(f"📁 Docker directory: {DOCKER_DIR}")
    print(f"🌐 API will be available at: http://localhost:8000")
    print(f"📚 Docs at: http://localhost:8000/docs")
    uvicorn.run(app, host="0.0.0.0", port=8000)