# BetaBank
FYP - Augmented Reality climbing videos display using OpenCV and ArUco.

## Setup
Python3 and Pip should be installed. Install remaining dependecies with:
```
python -m pip install -r requirements.txt
Note: It is recommended to install dependencies in an activated virtual environment (venv)
```

### Create and Activate Venv
```
python -m venv beeware-venv

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
python -m src.betabank.capture video demo1.mp4
python -m src.betabank.capture picture demo1.png
```

## Display in WebGL (MAIN)
```
python -m src.betabank.main webgl "test_images_1920x1080/capture_2.png" "test_videos_1920x1080/demo2.mp4"
```

### Display Origin Pose in Video
```
python -m src.betabank.main video "test_images_1920x1080/capture_2.png" "test_videos_1920x1080/demo2.mp4" "cube"
```

### Display Origin Pose in Image
```
python -m src.betabank.main image "test_images_1920x1080/capture_2.png" "test_images_1920x1080/capture_1.png" "axis"
```

### Display All Pose in Image
```
python -m src.betabank.main image_all "test_images_1920x1080/capture_2.png" "test_images_1920x1080/capture_1.png" "axis"
```

## Testing
Run Unit Tests with (BASH):
```
./unit_test.sh
```
All tests should pass.