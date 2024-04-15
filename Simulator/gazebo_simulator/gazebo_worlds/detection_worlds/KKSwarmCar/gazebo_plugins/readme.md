https://blog.csdn.net/yldmkx/article/details/108894093\

XXX.cc为插件的源文件，使用cmake编译
进入buid目录右键打开终端，输入【cmake ..】命令，然后再输入【make】命令就会在build目录生成一个libXXXX.so库文件


将文件复制到gazebo默认的库目录

cd /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/

sudo rm libkkmove.so 


sudo cp libkkmove.so  /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libkkmove.so

sudo cp libkkmove1.so  /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libkkmove1.so

sudo cp libkkmove2.so  /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libkkmove2.so

sudo cp libkkplan.so  /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libkkplan.so

sudo cp libkkplan1.so  /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libkkplan1.so

sudo cp libkkplan2.so  /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libkkplan2.so








sudo cp build/libXXXX.so /usr/lib/x86_64-linux-gnu/gazebo-X/plugins/libXXXX.so

打开gazebo使用model_editor编辑模型添加对应运动的插件name和filename，然后保存为world


在world里修改filename，name任意,即可测试插件功能
      <plugin name='move' filename='libline.so'/>

