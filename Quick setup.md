## Setup enviornment
```bash
conda env create -f environment.yml
conda activate opencda
python setup.py develop
```

## Fix environment variables

## Pip install carla
```bash
pip install -e "Y:\CEE 490 068 WN 2022\OpenCDA\cache\carla-0.9.12-py3.7-win-amd64"
```

## Extras

```bash
conda install pytorch==1.8.0 torchvision==0.9.0 torchaudio==0.8.0 cudatoolkit=11.1 -c pytorch -c conda-forge
pip install -qr https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt
pip install traci
SUMO_HOME="/e/CEE 490 068 WN 2022/sumo-1.14.1"
```