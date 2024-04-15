#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
 
namespace gazebo
{
  class kkmove2 : public ModelPlugin  //修改此处的类名为  move  ，要和最后注册的名称一样
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
 
        this->model = _parent;
 
        //使用PoseAnimation类实例化一个对象，然后通过三个参数可设置运动模型名称，运动持续时间以及是否循环执行
        gazebo::common::PoseAnimationPtr anim(new gazebo::common::PoseAnimation("test", 40.0, true));  
        
        //声明一个控制模型位姿的对象
        gazebo::common::PoseKeyFrame *key;  
 	int tme;
	double j[40] = {-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,0.9,0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.1,0,-0.1,-0.2,-0.3,-0.4,-0.5,-0.6,-0.7,-0.8,-0.9};
	double k[40] = {-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,1.5,1.4,1.3,1.2,1.1,1,0.9,0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.1,0,-0.1,-0.2,-0.3,-0.4};
        //设置模型到达某位姿的时刻   可以任意修改
	for(tme=0;tme<=1200;tme++){
		key = anim->CreateKeyFrame(tme);
        	key->Translation(ignition::math::Vector3d(j[tme%40],k[tme%40], 0.01));   //xyz
        	key->Rotation(ignition::math::Quaterniond(0, 0, 0));	
		}
	_parent->SetAnimation(anim);	
	
       
 

    }
 
    private: physics::ModelPtr model;
 
    //通过事件响应来更新触发程序
    private: event::ConnectionPtr updateConnection;
  };
 
  //在Gazebo仿真器中注册该插件     和类名一样
  GZ_REGISTER_MODEL_PLUGIN(kkmove2)
}
