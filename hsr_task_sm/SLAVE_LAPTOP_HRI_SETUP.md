# Slave Laptop HRI Setup Guide

This guide explains how to set up the slave laptop for Whisper STT + Ollama LLM HRI.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                           ROS NETWORK                               │
│                      ROS_MASTER_URI on Robot                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌────────────────────────┐      ┌────────────────────────────────┐ │
│  │      ROBOT (HSR)       │      │       SLAVE LAPTOP             │ │
│  │                        │      │                                │ │
│  │  - State Machine       │      │  - Whisper STT Service         │ │
│  │  - Navigation          │◄────►│  - Ollama LLM Service          │ │
│  │  - TTS (sound_play)    │      │  - Mic recording (arecord)     │ │
│  │  - Person Detection    │      │                                │ │
│  │                        │      │                                │ │
│  └────────────────────────┘      └────────────────────────────────┘ │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

## Slave Laptop Requirements

### Hardware
- Laptop with microphone (USB or built-in)
- WiFi connection to robot's network
- Recommended: GPU for faster Whisper inference (optional)

### Software Dependencies

```bash
# Install system packages
sudo apt install alsa-utils pulseaudio python3-pip

# Install Python packages
pip3 install openai-whisper ollama pyaudio

# Install Ollama
curl -fsSL https://ollama.ai/install.sh | sh

# Start Ollama server
ollama serve &

# Pull the LLM model
ollama pull llama3.2
```

## Network Configuration

### On the Robot
```bash
# Find robot's IP
hostname -I

# Ensure roscore is running
roscore
```

### On the Slave Laptop
```bash
# Set ROS environment (add to ~/.bashrc)
export ROS_MASTER_URI=http://<ROBOT_IP>:11311
export ROS_IP=<SLAVE_LAPTOP_IP>

# Verify connectivity
rostopic list
```

## Running the Services

### Option 1: Using Launch File
```bash
# On slave laptop
roslaunch hsr_task_sm slave_laptop_hri.launch
```

### Option 2: Manual Start
```bash
# Terminal 1: Start Whisper STT service
rosrun hsr_task_sm whisper_stt_service.py

# Terminal 2: Start Voicebot Ollama service  
rosrun hsr_task_sm voicebot_ollama_service.py
```

### Option 3: With Custom Parameters
```bash
roslaunch hsr_task_sm slave_laptop_hri.launch \
    whisper_model:=small.en \
    ollama_model:=mistral \
    record_duration:=7
```

## Available ROS Services & Topics

### Services
| Service | Type | Description |
|---------|------|-------------|
| `/speech_recognize` | `std_srvs/Trigger` | Record & transcribe speech |
| `/voicebot/prompt` | `hsr_task_sm/VoicebotPrompt` | Send prompt to LLM |
| `/voicebot/reset` | `std_srvs/Trigger` | Reset conversation history |

### Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/condition_record` | `std_msgs/Bool` | Enable/disable mic recording |
| `/speech_text` | `std_msgs/String` | Published recognized text |
| `/voicebot/response` | `std_msgs/String` | Published LLM responses |
| `/voicebot/guest_info` | `std_msgs/String` | JSON guest info |

## Testing

### Test Microphone
```bash
# List audio devices
arecord -l

# Test recording (5 seconds)
arecord -d 5 -f cd test.wav
aplay test.wav
```

### Test Whisper Service
```bash
# Enable mic
rostopic pub /condition_record std_msgs/Bool "data: true" -1

# Call speech recognition
rosservice call /speech_recognize

# Check result
rostopic echo /speech_text -n 1
```

### Test Ollama Service
```bash
# Send prompt
rosservice call /voicebot/prompt "prompt: 'Hello, what is your name?'"

# Check response
rostopic echo /voicebot/response -n 1
```

## Whisper Model Options

| Model | Size | Speed | Accuracy |
|-------|------|-------|----------|
| `tiny.en` | 39MB | Fastest | Lower |
| `base.en` | 74MB | Fast | Good (default) |
| `small.en` | 244MB | Medium | Better |
| `medium.en` | 769MB | Slow | Great |
| `large` | 1.5GB | Slowest | Best |

## Ollama Model Options

| Model | Size | Description |
|-------|------|-------------|
| `llama3.2` | 3B | Fast, good conversations (default) |
| `mistral` | 7B | Higher quality |
| `phi3` | 3.8B | Microsoft's efficient model |

## Troubleshooting

### "No audio devices found"
```bash
# Check PulseAudio
pulseaudio --check
pulseaudio --start

# List devices again
arecord -l
```

### "Connection refused" for Ollama
```bash
# Make sure Ollama server is running
ollama serve

# Check if it's listening
curl http://localhost:11434/api/tags
```

### "Cannot connect to ROS Master"
```bash
# Verify IP settings
echo $ROS_MASTER_URI
echo $ROS_IP

# Test connectivity
ping <ROBOT_IP>

# Check firewall
sudo ufw allow 11311/tcp
```

### "Whisper model not found"
```bash
# Models are downloaded on first use
# Or pre-download:
python3 -c "import whisper; whisper.load_model('base.en')"
```

## Running the Full HRI Challenge

### On the Robot
```bash
# Make sure TTS service is running
rosrun sound_play soundplay_node.py &

# Run the HRI challenge state machine
rosrun hsr_task_sm hri_ollama_challenge_sm
```

### Monitor Conversation
```bash
# Watch speech recognition
rostopic echo /speech_text

# Watch LLM responses
rostopic echo /voicebot/response

# Watch guest info
rostopic echo /voicebot/guest_info
```

## Configuration Files

- **YAML Config**: `ros/config/challenges/hri_ollama_challenge.yaml`
- **Launch File**: `ros/launch/slave_laptop_hri.launch`
- **State Machine**: `ros/scripts/hri_ollama_challenge_sm`

## Contact

For issues, check the b-it-bots reference implementation:
https://github.com/b-it-bots/mas_domestic_robotics/tree/german_open_2026
