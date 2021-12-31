# download submodule: object_detection_oneshot
git clone https://gitee.com/jario-jin/One-Shot-Object-Detection.git Modules/object_detection_oneshot
cd Modules/object_detection_oneshot
pip3 install scikit-build -i https://pypi.tuna.tsinghua.edu.cn/simple/
pip3 install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple/
pip3 install easydict -i https://pypi.tuna.tsinghua.edu.cn/simple/
pip3 install matplotlib -i https://pypi.tuna.tsinghua.edu.cn/simple/
pip3 install imageio tqdm -i https://pypi.tuna.tsinghua.edu.cn/simple/
cd lib
sudo python3 setup.py build develop
cd ..
bash prepare_models.sh

