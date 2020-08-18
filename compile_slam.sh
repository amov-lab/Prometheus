catkin_make --source Modules/slam --build build/slam

echo "Uncompress vocabulary ..."

cd Modules/slam/config/Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ../../../../
