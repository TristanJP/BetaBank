# BetaBank
FYP

## Setup
Python3 and Pip should be installed. Install remaining dependecies with:
```
py -m pip install -r requirements.txt
Note: It is recommended to install dependencies in an activated venv
```

### Create and Activate Venv
```
py -m venv beeware-venv

beeware-venv\Scripts\activate.bat
```

## Run Application GUI
```
briefcase dev
```

## CLI Controls
From top `betabank/` directory:

### Capture Video
```
py -m src.betabank.capture video demo1.mp4
py -m src.betabank.capture picture demo1.png
```

### Display in WebGL
```
py -m src.betabank.main webgl "test_images_1920x1080/capture_2.png" "test_videos_1920x1080/demo2.mp4"
```