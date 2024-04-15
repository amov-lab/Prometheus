#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
 
namespace gazebo
{
  class landmove : public ModelPlugin  //修改此处的类名为  move  ，要和最后注册的名称一样
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
 
        this->model = _parent;
 
        //使用PoseAnimation类实例化一个对象，然后通过三个参数可设置运动模型名称，运动持续时间以及是否循环执行
        gazebo::common::PoseAnimationPtr anim(new gazebo::common::PoseAnimation("test", 120.0, true));  
        
        //声明一个控制模型位姿的对象
        gazebo::common::PoseKeyFrame *key;  
	double j[60]={0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2,2.1,2.2,2.3,2.4,2.5,2.6,2.7,2.8,2.9,3,2.9,2.8,2.7,2.6,2.5,2.4,2.3,2.2,2.1,2,1.9,1.8,1.7,1.6,1.5,1.4,1.3,1.2,1.1,1,0.9,0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.1};
        int i = 0;
        //设置模型到达某位姿的时刻   可以任意修改
	for(float tme=0;tme<=4000;tme+=0.5){
		key = anim->CreateKeyFrame(tme);
        	key->Translation(ignition::math::Vector3d(2.5,j[i % 60],1));  //x,y,z j[static_cast<int>(tme) % 61]
        	key->Rotation(ignition::math::Quaterniond(0, 0, 0));	
        	i++;
		}
	_parent->SetAnimation(anim);	
	

 
       
 

    }
 
    private: physics::ModelPtr model;
 
    //通过事件响应来更新触发程序
    private: event::ConnectionPtr updateConnection;
  };
 
  //在Gazebo仿真器中注册该插件     和类名一样
  GZ_REGISTER_MODEL_PLUGIN(landmove)
}
