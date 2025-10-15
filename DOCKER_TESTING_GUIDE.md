# Docker Testing Guide for Robot Control Application

## 🐳 Quick Testing

Run the Docker-specific test suite:
```bash
python test_docker.py
```

## Manual Testing

### Quick Commands
```bash
# Test PyBullet
python -c "import pybullet as p; print('PyBullet test:', p.connect(p.DIRECT))"

# Test robot classes
python -c "import sys; sys.path.insert(0, 'src'); from robot_arm import RobotArm6DOF; from wheel_car import CarRobot; print('Robot classes work!')"
```

## Running the Application

```bash
# Terminal 1: Start simulation server
python src/simulation_server.py

# Terminal 2: Start Streamlit app
streamlit run src/app.py --server.headless true --server.port 8501
```

## Manual Testing Checklist

### ✅ Basic Tests
- [ ] Run `python test_docker.py` - should show mostly green checkmarks
- [ ] PyBullet works in headless mode
- [ ] Streamlit starts without errors
- [ ] Robot classes import successfully

### ✅ UI Tests (after starting app)
- [ ] Page loads at http://localhost:8501
- [ ] Robot type selection works (6-DOF Arm ↔ Wheeled Car)
- [ ] "Start Simulation" opens PyBullet GUI
- [ ] Position input accepts X, Y, Z coordinates
- [ ] "Go to Position" moves robot to target
- [ ] "Reset Position" returns robot to start
- [ ] Position display shows current robot position

## Troubleshooting

### Common Issues
- **Port in use**: `pkill -f streamlit` then restart
- **PyBullet errors**: Always use headless mode `p.connect(p.DIRECT)`
- **Import errors**: Check file structure and Python path

### Quick Fixes
```bash
# Kill existing processes
pkill -f streamlit
pkill -f python

# Check ports
netstat -tulpn | grep :8501

# Test imports
python -c "import streamlit, pybullet, numpy, websockets"
```
