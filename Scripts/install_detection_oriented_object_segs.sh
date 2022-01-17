# download submodule: object_detection_oneshot
git clone https://gitee.com/jario-jin/prometheus-oriented-object-segs.git Modules/oriented_detection_segs

cd Modules/oriented_detection_segs

sudo apt-get install curl
sudo apt-get install wget
sudo apt-get install swig

pip3 install shapely -i https://pypi.tuna.tsinghua.edu.cn/simple

cd datasets/DOTA_devkit
swig -c++ -python polyiou.i
python3 setup.py build_ext --inplace

cd ..
cd ..
mkdir weights_dota
wget http://jario.ren/models/prometheus-oriented-object-segs/model_50.pth -O weights_dota/model_50.pth

