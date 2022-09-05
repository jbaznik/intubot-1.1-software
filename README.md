# Intubot 1.1 Software

## Install
1. Make sure that Python 3.7 or 3.8 is installed
1. create with python a new virtual environment: python -m venv env
1. activate new virtual environment 
    - bash: ./env/Scripts/activate
    - cmd: env/Scripts/activate
    - powershell: cd .\env\Scripts\; .\activate
1. Install packages: pip install -r requirements.txt

# TODO: run as .exe?

## Run

1. activate virtual environment: .
    - bash: ./env/Scripts/activate
    - cmd: env/Scripts/activate
    - powershell: cd .\env\Scripts\; .\activate
2. without GUI, run: python main.py
3. or with GUI, run: python gui.py

## GUI

Implemented keys:

- c: change camera mode (camera 1, camera 2, both cameras)
- h: flip image horizontally
- m: change computer vision model (implemented models: manikin, coco)
- i: drive translational sled in
- o: drive translational sled out
- space: show/hide expert mode
- esc: close program
