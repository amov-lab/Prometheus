#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
 
namespace gazebo
{
  class kkplan1 : public ModelPlugin  //修改此处的类名为  move  ，要和最后注册的名称一样
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
 
        this->model = _parent;
 
        //使用PoseAnimation类实例化一个对象，然后通过三个参数可设置运动模型名称，运动持续时间以及是否循环执行
        gazebo::common::PoseAnimationPtr anim(new gazebo::common::PoseAnimation("test", 4000, true));  
        
        //声明一个控制模型位姿的对象
        gazebo::common::PoseKeyFrame *key;  
 	int tme;
	double j[40] = {3,3.1,3.2,3.3,3.4,3.5,3.6,3.7,3.8,3.9,4,3.9,3.8,3.7,3.6,3.5,3.4,3.3,3.2,3.1,3,3.1,3.2,3.3,3.4,3.5,3.6,3.7,3.8,3.9,4,3.9,3.8,3.7,3.6,3.5,3.4,3.3,3.2,3.1};
  double k[40] = {5,5.1,5.2,5.3,5.4,5.5,5.6,5.7,5.8,5.9,6,5.9,5.8,5.7,5.6,5.5,5.4,5.3,5.2,5.1,5,5.1,5.2,5.3,5.4,5.5,5.6,5.7,5.8,5.9,6,5.9,5.8,5.7,5.6,5.5,5.4,5.3,5.2,5.1};
        //设置模型到达某位姿的时刻   可以任意修改
	for(tme=0;tme<=1200;tme++){
		key = anim->CreateKeyFrame(tme);
        	key->Translation(ignition::math::Vector3d(j[tme%40],3, k[tme%40]));   //xyz
        	key->Rotation(ignition::math::Quaterniond(0, 0, 0));	
		}
	_parent->SetAnimation(anim);	
	
       
 

    }
 
    private: physics::ModelPtr model;
 
    //通过事件响应来更新触发程序
    private: event::ConnectionPtr updateConnection;
  };
 
  //在Gazebo仿真器中注册该插件     和类名一样
  GZ_REGISTER_MODEL_PLUGIN(kkplan1)
}
