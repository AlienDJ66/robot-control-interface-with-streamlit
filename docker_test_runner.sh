#!/bin/bash

# Docker Test Runner for Robot Control Application
# This script runs comprehensive tests inside a Docker container

echo "🐳 Docker Robot Control Application Test Runner"
echo "=============================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    if [ $2 -eq 0 ]; then
        echo -e "${GREEN}✅ $1${NC}"
    else
        echo -e "${RED}❌ $1${NC}"
    fi
}

print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

# Check if we're in a Docker container
echo ""
print_info "Checking Docker Environment..."
if [ -f /.dockerenv ]; then
    print_status "Running in Docker container" 0
else
    print_warning "Not detected as Docker container"
fi

# Check Python availability
echo ""
print_info "Checking Python Environment..."
if command -v python &> /dev/null; then
    PYTHON_CMD="python"
    print_status "Python found" 0
elif command -v python3 &> /dev/null; then
    PYTHON_CMD="python3"
    print_status "Python3 found" 0
else
    print_status "Python not found" 1
    exit 1
fi

# Check Python version
PYTHON_VERSION=$($PYTHON_CMD --version 2>&1)
print_info "Python version: $PYTHON_VERSION"

# Check required packages
echo ""
print_info "Checking Required Packages..."
PACKAGES=("streamlit" "pybullet" "numpy" "websockets")
MISSING_PACKAGES=()

for package in "${PACKAGES[@]}"; do
    if $PYTHON_CMD -c "import $package" 2>/dev/null; then
        print_status "$package" 0
    else
        print_status "$package - MISSING" 1
        MISSING_PACKAGES+=("$package")
    fi
done

if [ ${#MISSING_PACKAGES[@]} -gt 0 ]; then
    echo ""
    print_warning "Missing packages: ${MISSING_PACKAGES[*]}"
    print_info "Install with: pip install ${MISSING_PACKAGES[*]}"
fi

# Check file structure
echo ""
print_info "Checking File Structure..."
REQUIRED_FILES=("src/app.py" "src/robot_arm.py" "src/wheel_car.py" "src/simulation.py")
MISSING_FILES=()

for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$file" ]; then
        print_status "$file" 0
    else
        print_status "$file - MISSING" 1
        MISSING_FILES+=("$file")
    fi
done

if [ ${#MISSING_FILES[@]} -gt 0 ]; then
    echo ""
    print_warning "Missing files: ${MISSING_FILES[*]}"
fi

# Test PyBullet headless mode
echo ""
print_info "Testing PyBullet Headless Mode..."
if $PYTHON_CMD -c "import pybullet as p; print('PyBullet headless test:', p.connect(p.DIRECT))" 2>/dev/null; then
    print_status "PyBullet headless mode works" 0
else
    print_status "PyBullet headless mode failed" 1
fi

# Test Streamlit import
echo ""
print_info "Testing Streamlit Import..."
if $PYTHON_CMD -c "import streamlit as st; print('Streamlit version:', st.__version__)" 2>/dev/null; then
    print_status "Streamlit import successful" 0
else
    print_status "Streamlit import failed" 1
fi

# Test robot classes
echo ""
print_info "Testing Robot Classes..."
if $PYTHON_CMD -c "
import sys
sys.path.insert(0, 'src')
from robot_arm import RobotArm6DOF
from wheel_car import CarRobot
print('Robot classes imported successfully')
" 2>/dev/null; then
    print_status "Robot classes import successful" 0
else
    print_status "Robot classes import failed" 1
fi

# Test port availability
echo ""
print_info "Testing Port Availability..."
PORTS=(8501 8765)
AVAILABLE_PORTS=()

for port in "${PORTS[@]}"; do
    if netstat -tuln 2>/dev/null | grep -q ":$port "; then
        print_warning "Port $port in use"
    else
        print_status "Port $port available" 0
        AVAILABLE_PORTS+=("$port")
    fi
done

# Run comprehensive Python tests
echo ""
print_info "Running Comprehensive Tests..."
if [ -f "test_docker.py" ]; then
    if $PYTHON_CMD test_docker.py; then
        print_status "Comprehensive tests passed" 0
    else
        print_status "Comprehensive tests failed" 1
    fi
else
    print_warning "test_docker.py not found, skipping comprehensive tests"
fi

# Test Streamlit startup (brief)
echo ""
print_info "Testing Streamlit Startup..."
print_info "Starting Streamlit app (will timeout after 15 seconds)..."

# Start Streamlit in background
$PYTHON_CMD -m streamlit run src/app.py --server.headless true --server.port 8501 --server.runOnSave false --browser.gatherUsageStats false &
STREAMLIT_PID=$!

# Wait a bit for startup
sleep 10

# Check if Streamlit is running
if kill -0 $STREAMLIT_PID 2>/dev/null; then
    print_status "Streamlit started successfully" 0
    
    # Test if it's responding
    if curl -s -o /dev/null -w "%{http_code}" http://localhost:8501 | grep -q "200"; then
        print_status "Streamlit responding on port 8501" 0
    else
        print_warning "Streamlit not responding on port 8501"
    fi
    
    # Kill the process
    kill $STREAMLIT_PID 2>/dev/null
    wait $STREAMLIT_PID 2>/dev/null
else
    print_status "Streamlit startup failed" 1
fi

# Summary
echo ""
echo "=============================================="
echo "📊 TEST SUMMARY"
echo "=============================================="

# Count results (simplified)
TOTAL_TESTS=8
PASSED_TESTS=0

# Check each test result (simplified counting)
[ ${#MISSING_PACKAGES[@]} -eq 0 ] && ((PASSED_TESTS++))
[ ${#MISSING_FILES[@]} -eq 0 ] && ((PASSED_TESTS++))
[ ${#AVAILABLE_PORTS[@]} -gt 0 ] && ((PASSED_TESTS++))

echo "Overall: $PASSED_TESTS/$TOTAL_TESTS tests passed"

if [ $PASSED_TESTS -eq $TOTAL_TESTS ]; then
    echo ""
    echo -e "${GREEN}🎉 APPLICATION IS READY IN DOCKER!${NC}"
    echo ""
    echo "To run the application:"
    echo "1. Start simulation server: $PYTHON_CMD src/simulation_server.py"
    echo "2. Start Streamlit: $PYTHON_CMD -m streamlit run src/app.py --server.headless true --server.port 8501"
    echo "3. Access at: http://localhost:8501"
elif [ $PASSED_TESTS -ge $((TOTAL_TESTS * 80 / 100)) ]; then
    echo ""
    echo -e "${YELLOW}⚠️  MOSTLY READY - Some minor issues${NC}"
    echo "Application should still work for basic functionality"
else
    echo ""
    echo -e "${RED}❌ NEEDS ATTENTION - Critical issues found${NC}"
    echo "Check the output above for details"
fi

echo ""
echo "For detailed testing, run: $PYTHON_CMD test_docker.py"
