/**
 * @file <argos3/plugins/robots/epuck/simulator/dynamics3d_epuck_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_epuck_model.h"

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_shape_manager.h>

#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DEPuckModel::CDynamics3DEPuckModel(CDynamics3DEngine& c_engine,
                                                CEPuckEntity& c_entity) :
      CDynamics3DMultiBodyObjectModel(c_engine, c_entity, 2, false),
      m_cWheeledEntity(c_entity.GetWheeledEntity()) {
      /* get the required collision shapes */
      std::shared_ptr<btCollisionShape> ptrBaseShape =
         CDynamics3DShapeManager::RequestCylinder(m_cBaseHalfExtents);
      std::shared_ptr<btCollisionShape> ptrWheelShape =
         CDynamics3DShapeManager::RequestCylinder(m_cWheelHalfExtents);
      /* calculate the inertia of the collision objects */
      btVector3 cBaseInertia;
      btVector3 cWheelInertia;
      ptrBaseShape->calculateLocalInertia(m_fBaseMass, cBaseInertia);
      ptrWheelShape->calculateLocalInertia(m_fWheelMass, cWheelInertia);
      /* calculate a btTransform that moves us from the global coordinate system to the
         local coordinate system */
      const SAnchor& sOriginAnchor = c_entity.GetEmbodiedEntity().GetOriginAnchor();
      const CQuaternion& cOrientation = sOriginAnchor.Orientation;
      const CVector3& cPosition = sOriginAnchor.Position;     
      const btTransform& cStartTransform = btTransform(
         btQuaternion(cOrientation.GetX(),
                      cOrientation.GetZ(),
                     -cOrientation.GetY(),
                      cOrientation.GetW()),
         btVector3(cPosition.GetX(),
                   cPosition.GetZ(),
                  -cPosition.GetY()));
      /* create a CAbstractBody::SData structure for each body */
      CAbstractBody::SData sBaseData(cStartTransform * m_cBaseOffset,
                                     m_cBaseGeometricOffset,
                                     cBaseInertia,
                                     m_fBaseMass,
                                     GetEngine().GetDefaultFriction());
      CAbstractBody::SData sLeftWheelData(cStartTransform * m_cLeftWheelOffset,
                                          m_cWheelGeometricOffset,
                                          cWheelInertia,
                                          m_fWheelMass,
                                          GetEngine().GetDefaultFriction());
      CAbstractBody::SData sRightWheelData(cStartTransform * m_cRightWheelOffset,
                                           m_cWheelGeometricOffset,
                                           cWheelInertia,
                                           m_fWheelMass,
                                           GetEngine().GetDefaultFriction());
      /* create the bodies */
      m_ptrBase = std::make_shared<CBase>(*this, nullptr, ptrBaseShape, sBaseData);
      m_ptrLeftWheel = std::make_shared<CLink>(*this, 0, nullptr, ptrWheelShape, sLeftWheelData);
      m_ptrRightWheel = std::make_shared<CLink>(*this, 1, nullptr, ptrWheelShape, sRightWheelData);
      /* copy the bodies to the base class */
      m_vecBodies = {
         m_ptrBase, m_ptrLeftWheel, m_ptrRightWheel,
      };
      /* synchronize with the entity with the space */
      Reset();
   }
   
   /****************************************/
   /****************************************/
   
   void CDynamics3DEPuckModel::Reset() {
      /* reset the base class */
      CDynamics3DMultiBodyObjectModel::Reset();
      /* set up wheels */
      m_cMultiBody.setupRevolute(m_ptrLeftWheel->GetIndex(),
                                 m_ptrLeftWheel->GetData().Mass,
                                 m_ptrLeftWheel->GetData().Inertia,
                                 m_ptrBase->GetIndex(),
                                 m_cBaseToLeftWheelJointRotation,
                                 btVector3(0.0f, 1.0f, 0.0f),
                                 m_cBaseToLeftWheelJointOffset,
                                 m_cLeftWheelToBaseJointOffset,
                                 true);
      m_cMultiBody.setupRevolute(m_ptrRightWheel->GetIndex(),
                                 m_ptrRightWheel->GetData().Mass,
                                 m_ptrRightWheel->GetData().Inertia,
                                 m_ptrBase->GetIndex(),
                                 m_cBaseToRightWheelJointRotation,
                                 btVector3(0.0f, 1.0f, 0.0f),
                                 m_cBaseToRightWheelJointOffset,
                                 m_cRightWheelToBaseJointOffset,
                                 true);
      /* set up motors for the wheels */
      m_ptrLeftMotor = 
         std::make_unique<btMultiBodyJointMotor>(&m_cMultiBody,
                                                 m_ptrLeftWheel->GetIndex(),
                                                 0.0f,
                                                 m_fWheelMotorMaxImpulse);
      m_ptrRightMotor = 
         std::make_unique<btMultiBodyJointMotor>(&m_cMultiBody,
                                                 m_ptrRightWheel->GetIndex(),
                                                 0.0f,
                                                 m_fWheelMotorMaxImpulse);
      /* Allocate memory and prepare the btMultiBody */
      m_cMultiBody.finalizeMultiDof();
      /* Synchronize with the entity in the space */
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEPuckModel::CalculateBoundingBox() {
      btVector3 cModelAabbMin, cModelAabbMax;
      /* Initialize the bounding box with the base's AABB */
      m_ptrBase->GetShape().getAabb(m_ptrBase->GetTransform(), cModelAabbMin, cModelAabbMax);
      /* Write back the bounding box swapping the coordinate systems and the Y component */
      GetBoundingBox().MinCorner.Set(cModelAabbMin.getX(), -cModelAabbMax.getZ(), cModelAabbMin.getY());
      GetBoundingBox().MaxCorner.Set(cModelAabbMax.getX(), -cModelAabbMin.getZ(), cModelAabbMax.getY());
   }

   /****************************************/
   /****************************************/
   
   void CDynamics3DEPuckModel::UpdateEntityStatus() {
      /* just run the base class's implementation of this method */
      CDynamics3DMultiBodyObjectModel::UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEPuckModel::UpdateFromEntityStatus() {
      /* run the base class's implementation of this method */
      CDynamics3DMultiBodyObjectModel::UpdateFromEntityStatus();
      /* update joint velocities */
      const Real* pfWheelVelocities = m_cWheeledEntity.GetWheelVelocities();    
      m_ptrLeftMotor->setVelocityTarget(pfWheelVelocities[0]);
      m_ptrRightMotor->setVelocityTarget(pfWheelVelocities[1]);
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEPuckModel::AddToWorld(btMultiBodyDynamicsWorld& c_world) {
      /* run the base class's implementation of this method */
      CDynamics3DMultiBodyObjectModel::AddToWorld(c_world);
      /* add the actuators (btMultiBodyJointMotors) constraints to the world */
      c_world.addMultiBodyConstraint(m_ptrLeftMotor.get());
      c_world.addMultiBodyConstraint(m_ptrRightMotor.get());
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEPuckModel::RemoveFromWorld(btMultiBodyDynamicsWorld& c_world) {
      /* remove the actuators (btMultiBodyJointMotors) constraints from the world */
      c_world.removeMultiBodyConstraint(m_ptrRightMotor.get());
      c_world.removeMultiBodyConstraint(m_ptrLeftMotor.get());
      /* run the base class's implementation of this method */
      CDynamics3DMultiBodyObjectModel::RemoveFromWorld(c_world);
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEPuckModel::UpdateOriginAnchor(SAnchor& s_anchor) {
      /* copy the base transform */
      btTransform cTransform(m_cBaseOffset.inverse());
      cTransform *= m_ptrBase->GetTransform();
      
      const btVector3& cPosition = cTransform.getOrigin();
      const btQuaternion& cOrientation = cTransform.getRotation();
      /* Swap coordinate system and set anchor position */
      s_anchor.Position.Set(cPosition.getX(), -cPosition.getZ(), cPosition.getY());
      /* Swap coordinate system and set anchor orientation */
      s_anchor.Orientation.Set(cOrientation.getW(),
                               cOrientation.getX(),
                              -cOrientation.getZ(),
                               cOrientation.getY());
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CEPuckEntity, CDynamics3DEPuckModel);

   /****************************************/
   /****************************************/

   const btVector3    CDynamics3DEPuckModel::m_cBaseHalfExtents(0.037f, 0.0375, 0.037f);
   const btScalar     CDynamics3DEPuckModel::m_fBaseMass(0.2388f);
   const btTransform  CDynamics3DEPuckModel::m_cBaseOffset(btQuaternion(0.0f, 0.0f, 0.0f, 1.0f), btVector3(-0.0f, 0.0005f, -0.0f));
   const btTransform  CDynamics3DEPuckModel::m_cBaseGeometricOffset(btQuaternion(0.0f, 0.0f, 0.0f, 1.0f), btVector3(0.0f, -0.0375f, 0.0f));

   const btVector3    CDynamics3DEPuckModel::m_cWheelHalfExtents(0.021f, 0.0015f, 0.021f);
   const btScalar     CDynamics3DEPuckModel::m_fWheelMass(0.01);
   const btTransform  CDynamics3DEPuckModel::m_cWheelGeometricOffset(btQuaternion(0.0f, 0.0f, 0.0f, 1.0f), btVector3(0.0f,-0.0015f,0.0f));
   const btTransform  CDynamics3DEPuckModel::m_cLeftWheelOffset(btQuaternion(-0.707107f, 0.0f, 0.0f, 0.707107f), btVector3(0.0f, 0.021f, -0.025f)); //
   const btTransform  CDynamics3DEPuckModel::m_cRightWheelOffset(btQuaternion(0.707107f, 0.0f, 0.0f, 0.707107f), btVector3(0.0f, 0.021f, 0.025f)); //
   
   const btVector3    CDynamics3DEPuckModel::m_cBaseToRightWheelJointOffset(0.0f, 0.0205f, 0.025f); //
   const btVector3    CDynamics3DEPuckModel::m_cBaseToLeftWheelJointOffset(0.0f, 0.0205f, -0.025f); //
   
   const btVector3    CDynamics3DEPuckModel::m_cRightWheelToBaseJointOffset(0.0f, 0.0015f, 0.0f);
   const btVector3    CDynamics3DEPuckModel::m_cLeftWheelToBaseJointOffset(0.0f, 0.0015f, 0.0f);
   
   const btQuaternion CDynamics3DEPuckModel::m_cBaseToLeftWheelJointRotation(0.707107f, 0.0f, 0.0f, 0.707107f);
   const btQuaternion CDynamics3DEPuckModel::m_cBaseToRightWheelJointRotation(-0.707107f, 0.0f, 0.0f, 0.707107f);

   const btScalar     CDynamics3DEPuckModel::m_fWheelMotorMaxImpulse(0.025f);

   /****************************************/
   /****************************************/

}
