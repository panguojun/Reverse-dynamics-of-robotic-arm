#pragma once
#include "unity.h"

/**********************************************************************
			反向动力学
	先从json配置文件创建节点树，然后根据节点树安装生成机械臂
**********************************************************************/
#define ARM_COLLISION

using namespace PM;
using namespace Unity;
struct IKArmBase
{
	float RotationClamp = 2.5 / 180. * PI;		// 最大旋转步长
	float TranslationClamp = 0.01 * SCALE;		// 最大平移步长

	struct ArmStruct				// 机械臂单元
	{
		GameObject ArmParent;			// 机械臂单元根物
		GameObject ArmObject;			// 机械臂单元物体
		GameObject ArmBase;			// 机械臂单元基物体
		GameObject ArmEnd;			// 机械臂单元末端物体
		GameObject ArmSocket;			// 机械臂单元连接（上一个机械臂单元的ArmEnd)

		float RotationMin = 0;			// 旋转最小角度
		float RotationMax = 110. / 180. * PI;	// 旋转最大角度
		float MinLength = 0, MaxLength = 1;	// 平移最大最小长度
	};
	std::vector<ArmStruct> ArmsList;		// 机械臂单元列表
	std::vector<Unity::SceneNode*> nodelist;	// 机械臂单元节点列表

	GameObject Target;				// 目标点物体
	Unity::SceneNode targetnd;			// 目标节点
	GameObject BaseObject;				// 机械臂的基座物体对应根节点
	Unity::SceneNode root;				// 根节点

	// --------------------------------------------------------
	// 计算垂直于旋转轴的平面上投影点
	vec3 ProjectOntoPlane(GameObject& ArmObject, GameObject& ArmBase, const vec3& rotax, GameObject& ArmTarget)
	{
		Unity::Transform r = ArmBase.transform();
		vec3 basepos = r.position();
		PM::plane_t p = PM::plane_t(basepos, rotax);

		vec3 TargetToOrigin = ArmTarget.transform().position() - basepos;
		float NormalDistance = Mathf::Dot(TargetToOrigin, p.n);
		vec3 ProjectedPoint = ArmTarget.transform().position() - (p.n * NormalDistance);
		return ProjectedPoint;
	}
	// --------------------------------------------------------
	// 机械臂单元指向某个目标物体
	inline float rrnd(float min = 0.0, float max = 1.0) { return (max) == (min) ? (min) : (min)+((float)(rand()) / (RAND_MAX + 1.0)) * ((max)-(min)); }

	bool PointAt(int x, GameObject& TargetObject)
	{	
		Unity::Transform r = ArmsList[x].ArmBase.transform();
		vec3 rotax = ArmsList[x].ArmBase.transform().rotation() * ArmsList[x].ArmBase.scenenode->rotdir;
	
		vec3 ProjectedPoint = ProjectOntoPlane(ArmsList[x].ArmObject, ArmsList[x].ArmBase,
				rotax,
				TargetObject);
		
		float SignAngle = Mathf::SignedAngle(
			r.rotation() * ArmsList[x].ArmBase.scenenode->pointdir, 
			ProjectedPoint - r.position(), 
			rotax);

		SignAngle = Mathf::Clamp(SignAngle, -RotationClamp, RotationClamp);

		float CurrentAngle;
		float FutureAngle;
		if (x == 0)
		{
			FutureAngle = Mathf::SignedAngle(
				BaseObject.transform().rotation() * BaseObject.scenenode->pointdir,
				TargetObject.transform().position() - r.position(),
				rotax);

			CurrentAngle = Mathf::SignedAngle(
				BaseObject.transform().rotation() * BaseObject.scenenode->pointdir,
				ArmsList[x].ArmObject.transform().rotation() * ArmsList[x].ArmObject.scenenode->pointdir,
				rotax);
		}
		else
		{
			FutureAngle = Mathf::SignedAngle(
				ArmsList[x - 1].ArmObject.transform().rotation() * ArmsList[x - 1].ArmObject.scenenode->pointdir,
				TargetObject.transform().position() - r.position(),
				rotax);

			CurrentAngle = Mathf::SignedAngle(
				ArmsList[x - 1].ArmObject.transform().rotation() * ArmsList[x - 1].ArmObject.scenenode->pointdir,
				ArmsList[x].ArmObject.transform().rotation() * ArmsList[x].ArmObject.scenenode->pointdir,
				rotax);
		}

		if (fabs(SignAngle) > 0.001) // 防抖动
		{
			float ang = (CurrentAngle + SignAngle);
			if (ang < -PI) ang += PI * 2;
			if (ang > PI) ang -= PI * 2;

			//CTRLTEST PRINTV(x << " : CurrentAngle=" << CurrentAngle << " SignAngle=" << SignAngle)

			if (ang > ArmsList[x].RotationMin && ang < ArmsList[x].RotationMax)
			{
			#ifdef ARM_COLLISION	
				PM::quaternion q0 = ArmsList[x].ArmObject.transform().q;
				for (int j = 0; j < 10; j++)
				{
					ArmsList[x].ArmObject.transform().Rotate(SignAngle / 5, ArmsList[x].ArmBase.scenenode->rotdir);
					if (x > 0 && // test!
						(HitTest(ArmsList[x].ArmBase.scenenode) || HitTest(ArmsList[x].ArmEnd.scenenode))
						)
					{
						PRINT("HitTest");
						ArmsList[x].ArmObject.transform().q = q0;
						SignAngle += rrnd(-1, 1) * RotationClamp;
						continue;
					}
					return false;
				}
			#else
				ArmsList[x].ArmObject.transform().Rotate(SignAngle / 5, ArmsList[x].ArmBase.scenenode->rotdir);
			#end	
			}
		}
		return true;
	}
	// --------------------------------------------------------
	// 反向计算旋转，从最末单元指向目标点然后向后传递，指向前一个的基点，位置也要调整
	void InverseArmRotation()
	{
		for (int i = ArmsList.size() - 1; i >= 0; i--)  // Reverse loop
		{
			if (i == ArmsList.size() - 1)   // if the final arm
			{
				PointAt(i, Target);

				vec3 p = ArmsList[i].ArmObject.transform().position();
				vec3 np = ArmsList[i].ArmObject.transform().position() -
							((ArmsList[i].ArmEnd.transform().position() - Target.transform().position()));
				{
			#ifdef ARM_COLLISION	
					bool bsuc = false;
					float deta = (p - np).len() * 20;
					for (int j = 0; j < 10; j++)
					{
						ArmsList[i].ArmObject.transform().position(np);
						if (HitTest(ArmsList[i].ArmBase.scenenode) || HitTest(ArmsList[i].ArmEnd.scenenode))
						{
							PRINT("InverseArmRotation HitTest");
							ArmsList[i].ArmObject.transform().position(p);
							np += vec3(rrnd(-1, 1), rrnd(-1, 1), rrnd(-1, 1)).normcopy() * deta;
							continue;
						}
						bsuc = true;
						break;
					}
					if (!bsuc)
						ArmsList[i].ArmObject.transform().position(p);
			#else						
					ArmsList[i].ArmObject.transform().position(np);		
			#end					
				}
			}
			else                            // for other arms
			{
				PointAt(i, ArmsList[i + 1].ArmBase);

				vec3 p = ArmsList[i].ArmObject.transform().position();
				vec3 np = ArmsList[i].ArmObject.transform().position() -
					(ArmsList[i].ArmEnd.transform().position() - ArmsList[i + 1].ArmBase.transform().position());
				{
			#ifdef ARM_COLLISION		
					bool bsuc = false;
					float deta = (p - np).len() * 20;
					for (int j = 0; j < 10; j++)
					{
						ArmsList[i].ArmObject.transform().position(np);
						if (HitTest(ArmsList[i].ArmBase.scenenode) || HitTest(ArmsList[i].ArmEnd.scenenode))
						{
							PRINT("InverseArmRotation HitTest other arms");
							ArmsList[i].ArmObject.transform().position(p);
							np += vec3(rrnd(-1, 1), rrnd(-1, 1), rrnd(-1, 1)).normcopy() * deta;
							continue;
						}
					 
						bsuc = true;
						break;
					}
					if (!bsuc)
						ArmsList[i].ArmObject.transform().position(p);
			#else
				ArmsList[i].ArmObject.transform().position(np);
			#end
				}
			}
		}
	}
	// --------------------------------------------------------
	// 正向调整机械臂单元的位置，从基座开始回归原来的位置
	void ForwardArmTransform()
	{
		for (int i = 0; i < ArmsList.size(); i++)
		{
			ArmStruct ArmData = ArmsList[i];

			vec3 position = ArmData.ArmObject.transform().position() 
							- ArmData.ArmBase.transform().position() + ArmData.ArmSocket.transform().position();
			
			ArmData.ArmObject.transform().position(position);
		}
	}
	// --------------------------------------------------------
	// 反向调整机械臂的平移位置，从末端向后传递
	void InverseArmTranslation()
	{
		for (int i = ArmsList.size() - 1; i >= 0; i--)  //Reverse loop
		{
			if (i == ArmsList.size() - 1)   // if the final arm
			{
				vec3 targetpos = Target.transform().position();
				vec3 endpos = ArmsList[i].ArmEnd.transform().position();
				vec3 basepos = ArmsList[i].ArmBase.transform().position();
				
				vec3 movdir = ArmsList[i].ArmBase.transform().rotation() * ArmsList[i].ArmBase.scenenode->movedir;
				float dis = (targetpos - endpos).dot(movdir);
				float singlemov = Mathf::Clamp(dis, -TranslationClamp, TranslationClamp);
				float armdot = (endpos + movdir * singlemov - basepos).dot(movdir);
				//PRINTV(armdot);
				if (armdot >= ArmsList[i].MinLength && armdot <= ArmsList[i].MaxLength) // 伸展范围 ：MinLength， MaxLength
				{
					ArmsList[i].ArmEnd.transform().position(endpos + movdir * singlemov);
					
					ArmsList[i].ArmObject.transform().position(
						ArmsList[i].ArmObject.transform().position() -
						((ArmsList[i].ArmEnd.transform().position() - Target.transform().position()))
					);
				}
			}
			else                            // for other arms
			{
				vec3 targetpos = ArmsList[i + 1].ArmBase.transform().position();
				vec3 endpos = ArmsList[i].ArmEnd.transform().position();
				vec3 basepos = ArmsList[i].ArmBase.transform().position();

				vec3 movdir = ArmsList[i].ArmBase.transform().rotation() * ArmsList[i].ArmBase.scenenode->movedir;
				float dis = (targetpos - endpos).dot(movdir);
				float singlemov = Mathf::Clamp(dis, -TranslationClamp, TranslationClamp);
				float armdot = (endpos - basepos + movdir * singlemov).dot(movdir);
				
				if (armdot >= ArmsList[i].MinLength && armdot <= ArmsList[i].MaxLength) // 伸展范围 ：MinLength， MaxLength
				{
					ArmsList[i].ArmEnd.transform().position(endpos + movdir * singlemov);

					ArmsList[i].ArmObject.transform().position(
						ArmsList[i].ArmObject.transform().position() -
						(ArmsList[i].ArmEnd.transform().position() - ArmsList[i + 1].ArmBase.transform().position())
					);
				}
			}
		}
	}
	// --------------------------------------------------------
	// 旋转机械臂的根物体，变换每个单元的本地坐标系
	void InheritRotations()
	{
		for (int i = 0; i < ArmsList.size(); i++)
		{
			if (i == 0)
			{
				ArmsList[i].ArmParent.transform().q = BaseObject.transform().rotation();
			}
			else
			{
				ArmsList[i].ArmParent.transform().q = ArmsList[i - 1].ArmObject.transform().rotation();
			}
		}
	}
	// 碰撞接口
	virtual bool HitTest(Unity::SceneNode* sn) = 0;
	// --------------------------------------------------------
	// 更新
	float update()
	{
		ASSERT(!ArmsList.empty());
		InheritRotations();

		float EndEffectorDistance = Mathf::Distance(
			Target.transform().position(),
			ArmsList[ArmsList.size() - 1].ArmEnd.transform().position());
		
		if (EndEffectorDistance > 0.025 * SCALE) // 判断末端是否到达位置
		{
			CTRLTEST PRINT("===========EndEffectorDistance ：" << EndEffectorDistance);
			{
				InverseArmTranslation();
				ForwardArmTransform(); 
			}
			{
				InverseArmRotation();
				ForwardArmTransform();
			}
			//return false;
		}
		return EndEffectorDistance;
	}
	// --------------------------------------------------------
	// 创建机械臂单元节点
	Unity::SceneNode* createArmNode(Unity::SceneNode* parent, 
		float anglemin, float anglemax, 
		float lengthmin, float lengthmax,
		Xre::SceneNode* xrebase = 0, Xre::SceneNode* xreend = 0)
	{
		Unity::SceneNode* armparent = new Unity::SceneNode();
		
		armparent->parent = parent;
		parent->child = armparent;
		{
			Unity::SceneNode* arm = new Unity::SceneNode();
			
			arm->parent = armparent;
			armparent->armchild = arm;
			{
				Unity::SceneNode* armbase = new Unity::SceneNode();
				armbase->parent = arm;
				arm->basechild = armbase;
				if (xrebase)
				{
					armbase->xrescenenode = xrebase;
				}
			}
			{
				Unity::SceneNode* armend = new Unity::SceneNode();
				armend->parent = arm;
				arm->endchild = armend;
				if (xreend)
				{
					armend->xrescenenode = xreend;
				}
			}
		}
		armparent->fdat[0] = anglemin;
		armparent->fdat[1] = anglemax;
		armparent->fdat[2] = lengthmin;
		armparent->fdat[3] = lengthmax;
		nodelist.push_back(armparent);
		return armparent;
	}
	// 删除节点
	void delArmNode(Unity::SceneNode* sn)
	{
		if (sn)
		{
			for (int i = 0; i < MAX_CHILDRREN; i++)
			{
				Unity::SceneNode* c = sn->children[i];
				if (c)
					delArmNode(c);
				sn->children[i] = 0;
			}

			if(sn != &root)
				delete sn;
		}
	}
	// --------------------------------------------------------	
	// 机械臂安装生成
	void setup()
	{
		for (int i = 0; i < nodelist.size(); i++)
		{
			ArmStruct as;
			as.ArmParent.scenenode = nodelist[i];
			as.ArmObject.scenenode = nodelist[i]->armchild;
			as.ArmBase.scenenode = nodelist[i]->armchild->basechild;
			as.ArmEnd.scenenode = nodelist[i]->armchild->endchild;

			as.RotationMin = nodelist[i]->fdat[0];
			as.RotationMax = nodelist[i]->fdat[1];
			as.MinLength = nodelist[i]->fdat[2];
			as.MaxLength = nodelist[i]->fdat[3];

			ArmsList.push_back(as);
		}

		for (int x = 0; x < ArmsList.size(); x++)
		{
			if (x == 0)
			{
				ArmsList[x].ArmSocket = BaseObject;
			}
			else 
			{
				ArmsList[x].ArmSocket = ArmsList[x - 1].ArmEnd;
			}
		}
	}
	// --------------------------------------------------------
	// 清理
	void clear()
	{
		delArmNode(&root);
		ArmsList.clear();
		nodelist.clear();
	}
};
