/**
 * @file <argos3/plugins/robots/epuck/simulator/dynamics3d_epuck_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_EPUCK_MODEL_H
#define DYNAMICS3D_EPUCK_MODEL_H

namespace argos {
   class CDynamics3DEPuckModel;
   class CEPuckEntity;
   class CWheeledEntity;
}

#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_multi_body_object_model.h>

namespace argos {

   class CDynamics3DEPuckModel : public CDynamics3DMultiBodyObjectModel {

   public:     

      CDynamics3DEPuckModel(CDynamics3DEngine& c_engine,
                            CEPuckEntity& c_entity);

      virtual ~CDynamics3DEPuckModel() {}

      virtual void Reset();

      virtual void CalculateBoundingBox();

      virtual void UpdateEntityStatus();

      virtual void UpdateFromEntityStatus();

      virtual void AddToWorld(btMultiBodyDynamicsWorld& c_world);

      virtual void RemoveFromWorld(btMultiBodyDynamicsWorld& c_world);

      virtual void UpdateOriginAnchor(SAnchor& s_anchor);

   private:
      CWheeledEntity& m_cWheeledEntity;
      
      /* joint constraints */
      std::unique_ptr<btMultiBodyJointMotor> m_ptrLeftMotor;
      std::unique_ptr<btMultiBodyJointMotor> m_ptrRightMotor;

      /* links */
      std::shared_ptr<CBase> m_ptrBase;
      std::shared_ptr<CLink> m_ptrLeftWheel;
      std::shared_ptr<CLink> m_ptrRightWheel;

      /* lower base data */
      static const btVector3 m_cBaseHalfExtents;
      static const btScalar m_fBaseMass;
      static const btTransform m_cBaseOffset;
      static const btTransform m_cBaseGeometricOffset;
      /* wheel data */
      static const btVector3 m_cWheelHalfExtents;
      static const btScalar m_fWheelMass;
      static const btTransform m_cWheelGeometricOffset;
      static const btTransform m_cLeftWheelOffset;
      static const btTransform m_cRightWheelOffset;
      static const btVector3 m_cBaseToRightWheelJointOffset;
      static const btVector3 m_cRightWheelToBaseJointOffset;
      static const btQuaternion m_cBaseToRightWheelJointRotation;
      static const btVector3 m_cBaseToLeftWheelJointOffset;
      static const btVector3 m_cLeftWheelToBaseJointOffset;
      static const btQuaternion m_cBaseToLeftWheelJointRotation;
      static const btScalar m_fWheelMotorMaxImpulse;
   };
}

#endif
