#!/bin/bash

# Robot Control System - Auto Launcher
# This script automatically starts all components of the robot control system

set -e

echo "=========================================="
echo "🤖 Robot Control System Launcher"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Paths
DOCKER_DIR="$HOME/nemo-agent-toolkit/docker"
BACKEND_DIR="$HOME/nemo-agent-toolkit/examples/multi_function_agent/gui_controller/backend"
FRONTEND_DIR="$HOME/nemo-agent-toolkit/examples/multi_function_agent/gui_controller/frontend"

# Check if directories exist
if [ ! -d "$DOCKER_DIR" ]; then
    echo -e "${RED}❌ Docker directory not found: $DOCKER_DIR${NC}"
    exit 1
fi

if [ ! -d "$BACKEND_DIR" ]; then
    echo -e "${RED}❌ Backend directory not found: $BACKEND_DIR${NC}"
    exit 1
fi

if [ ! -d "$FRONTEND_DIR" ]; then
    echo -e "${RED}❌ Frontend directory not found: $FRONTEND_DIR${NC}"
    exit 1
fi

echo -e "${BLUE}📂 All directories found!${NC}"
echo ""

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check requirements
echo -e "${BLUE}🔍 Checking requirements...${NC}"

if ! command_exists gnome-terminal; then
    echo -e "${RED}❌ gnome-terminal not found. Please install it.${NC}"
    exit 1
fi

if ! command_exists docker; then
    echo -e "${RED}❌ Docker not found. Please install Docker.${NC}"
    exit 1
fi

if ! command_exists python3; then
    echo -e "${RED}❌ Python3 not found. Please install Python.${NC}"
    exit 1
fi

if ! command_exists npm; then
    echo -e "${RED}❌ npm not found. Please install Node.js.${NC}"
    exit 1
fi

echo -e "${GREEN}✅ All requirements met!${NC}"
echo ""

# Step 1: Start Docker Container
echo -e "${YELLOW}[1/4]${NC} ${BLUE}Starting Docker container...${NC}"
gnome-terminal --title="🐳 Robot Container" --geometry=100x30+0+0 -- bash -c "
    cd $DOCKER_DIR
    echo '=========================================='
    echo '🐳 Docker Container Terminal'
    echo '=========================================='
    echo ''
    echo 'Starting container...'
    ./run_hybrid_container.sh
" &
CONTAINER_PID=$!
echo -e "${GREEN}✅ Container terminal opened (PID: $CONTAINER_PID)${NC}"
echo ""

# Wait for container to start
echo -e "${YELLOW}⏳ Waiting 8 seconds for container to initialize...${NC}"
sleep 8

# Check if container is running
CONTAINER_ID=$(docker ps -q --filter "ancestor=nemo-agent-toolkit" | head -n 1)
if [ -z "$CONTAINER_ID" ]; then
    echo -e "${RED}❌ Container failed to start. Please check the container terminal.${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Container running: $CONTAINER_ID${NC}"
echo ""

# Step 1.5: Setup container (run once)
echo -e "${YELLOW}[1.5/4]${NC} ${BLUE}Setting up container environment...${NC}"
docker exec "$CONTAINER_ID" bash -c "
    source /workspace/.venv/bin/activate &&
    cd /workspace/mounted_code &&
    uv pip install -e . > /dev/null 2>&1
" 2>/dev/null || true
echo -e "${GREEN}✅ Container setup complete${NC}"
echo ""

# Step 2: Start Backend API
echo -e "${YELLOW}[2/4]${NC} ${BLUE}Starting Backend API...${NC}"
gnome-terminal --title="🐍 Backend API" --geometry=100x30+0+400 -- bash -c "
    cd $BACKEND_DIR
    echo '=========================================='
    echo '🐍 Backend API Server'
    echo '=========================================='
    echo ''
    echo 'Starting FastAPI server...'
    echo ''
    python3 main.py
    echo ''
    echo 'Backend stopped.'
    read -p 'Press Enter to close...'
" &
BACKEND_PID=$!
echo -e "${GREEN}✅ Backend terminal opened (PID: $BACKEND_PID)${NC}"
echo ""

# Wait for backend to start
echo -e "${YELLOW}⏳ Waiting 3 seconds for backend to start...${NC}"
sleep 3

# Check if backend is running
if ! curl -s http://localhost:8000/api/health > /dev/null; then
    echo -e "${YELLOW}⚠️  Backend not responding yet (may need more time)${NC}"
else
    echo -e "${GREEN}✅ Backend is healthy${NC}"
fi
echo ""

# Step 3: Start Frontend
echo -e "${YELLOW}[3/4]${NC} ${BLUE}Starting Frontend...${NC}"
gnome-terminal --title="⚛️  Frontend Dev" --geometry=100x30+900+0 -- bash -c "
    cd $FRONTEND_DIR
    echo '=========================================='
    echo '⚛️  Frontend Development Server'
    echo '=========================================='
    echo ''
    echo 'Starting Vite dev server...'
    echo ''
    npm run dev
    echo ''
    echo 'Frontend stopped.'
    read -p 'Press Enter to close...'
" &
FRONTEND_PID=$!
echo -e "${GREEN}✅ Frontend terminal opened (PID: $FRONTEND_PID)${NC}"
echo ""

# Wait for frontend to start
echo -e "${YELLOW}⏳ Waiting 5 seconds for frontend to start...${NC}"
sleep 5

# Step 4: Open Browser
echo -e "${YELLOW}[4/4]${NC} ${BLUE}Opening browser...${NC}"
if command_exists xdg-open; then
    xdg-open http://localhost:5173 2>/dev/null &
    echo -e "${GREEN}✅ Browser opened${NC}"
elif command_exists google-chrome; then
    google-chrome http://localhost:5173 2>/dev/null &
    echo -e "${GREEN}✅ Browser opened${NC}"
elif command_exists firefox; then
    firefox http://localhost:5173 2>/dev/null &
    echo -e "${GREEN}✅ Browser opened${NC}"
else
    echo -e "${YELLOW}⚠️  Could not open browser automatically${NC}"
    echo -e "${YELLOW}   Please open: http://localhost:5173${NC}"
fi
echo ""

# Summary
echo "=========================================="
echo -e "${GREEN}✅ Robot Control System Started!${NC}"
echo "=========================================="
echo ""
echo "📊 System Status:"
echo "  🐳 Container: Running ($CONTAINER_ID)"
echo "  🐍 Backend:   http://localhost:8000"
echo "  ⚛️  Frontend:  http://localhost:5173"
echo ""
echo "📱 Open terminals:"
echo "  - Container Terminal (setup & running)"
echo "  - Backend API Server"
echo "  - Frontend Dev Server"
echo ""
echo "🌐 Access Points:"
echo "  - Web UI:     http://localhost:5173"
echo "  - API Docs:   http://localhost:8000/docs"
echo "  - Health:     http://localhost:8000/api/health"
echo ""
echo "🛑 To stop all services:"
echo "  - Close all terminal windows"
echo "  - Or run: docker stop $CONTAINER_ID"
echo ""
echo "=========================================="
echo -e "${BLUE}Enjoy controlling your robot! 🤖${NC}"
echo "=========================================="
