/*************************************************************************
 *                                                                       *
 * 2d Hopper Demo for ODE, Copyright (C) 2013 Joseph L. Cooper           *
 * All rights reserved.                                                  *
 *                                                                       *
 * This demo is free software; you can redistribute it and/or            *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

/*
  This demo illustrates the 2d hopper described in 
  Marc Raibert's "Legged Robots That Balance" MIT press, 1986.
*/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#endif



// dynamics and collision objects
static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;
static dGeomID ground;

//  Higher friction makes it so the hopper binds between the 2d constraint and the ground
dReal groundFriction = 1;   
dReal stepSize = 1/60.0;  //  fps
dReal gravity = 9.8; 



/**
  Physical parts of the model
*/
enum PartsList
{
  CrossBar_Part=0,
  SpringTop_Part,
  SpringBottom_Part,
  Part_Count
};
dBodyID partBody[Part_Count];
dGeomID partGeom[Part_Count];

dReal partMass[Part_Count] = {8,.5,.5};
dReal partDim [Part_Count][3] = {
  {.05,1,.05},
  {.04,.16,0},
  {.02,.6,0}
};
dReal partPos [Part_Count][3] = {
  {2.5,0,.5},
  {2.5,0,.5},
  {2.5,0,.32}
};

/**
  Joints holding the model together
*/
enum JointList
{
  MainSpring_Joint=0,
  MainHinge_Joint,
  CircleLimit_Joint,
  Joint_Count
};
dJointID joints[Joint_Count];
// TODO: Bring the joint constant parameters here.


/*
  Controller constants
*/

// These values determine how high the hopper will jump.
dReal setPoint = -.10;    // Default spring setpoint
// This re-introduces energy lost by damping and friction
dReal pushVal =  -.75;    // How does the spring setpoint change while we're pushing off the ground.

dReal springKp = 400;     // Hopper spring stiffness
dReal springKd = 15;      // Hopper spring damping

// These values control how quickly the hopper goes around the circle
dReal desiredVel = .2;    // How fast are we trying to hop (counter clockwise as seen from above)

dReal tGain    = 100.0;   // Hinge gains for achieving desired angle between leg and crosspiece
dReal tDotGain = 5.0;     // while in the air

dReal  phiGain = -9.0;    // Hinge gains for keeping the crossbar level while on the ground
dReal  phiDotGain = -1.0; // This controls the angular velocity of the system.

dReal oldPhi = 0;         // Angle between crossbar and ground (for finite difference velocity estimation)
bool prevTouch=false;     // Is the foot on the ground?
int touchTime=0;          // How many frames has foot been on ground or airborne
bool jumping = false;     // Do we add extra force to make the spring jump
dReal jumpTime=0;         // When we trigger a jump, we set a fixed amount of time to be jumping.


// Collision callback
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);

  if (b1 && b2 ) return;  // Only collide with ground
  // The hopper behaves badly if the foot collides with the crosspiece.

  dContact contact[3];		// up to 3 contacts per body
  for (int ii=0; ii<3; ++ii) {
    contact[ii].surface.mode = dContactSoftCFM | dContactApprox1; 
	  contact[ii].surface.soft_cfm = 0.0001; //Ground surface is a little squishy
	  contact[ii].surface.mu = groundFriction; 
  }
  if (int numc = dCollide (o1,o2,3,&contact[0].geom,sizeof(dContact))) {
    for (int ii=0; ii<numc; ++ii) {
      dJointID c = dJointCreateContact (world,contactgroup,contact+ii);
      dJointAttach (c,b1,b2);
    }
  }
}

void resetModel();
static void command (int cmd)
{
  switch (cmd) {
  case ' ':
    jumping=!jumping;
    jumpTime = 0.5;
    break;
  case 'w':
  case 'W':
    pushVal-=.05;
    break;
  case 's':
  case 'S':
    pushVal+=.05;
    break;
  case 'a':
  case 'A':
    desiredVel-=.05;
    break;
  case 'd':
  case 'D':
    desiredVel+=.05;
    break;
  case 'r':
  case 'R':
    resetModel();
    break;
  }
}


// start simulation - set viewpoint
static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {0,-3.0,1.500};
  static float hpr[3] = {90.0000,-5.0000,0.0000};
  dsSetViewpoint (xyz,hpr);

  printf ("Press:\n"
    "\t'a' to increase clockwise velocity.\n"
	  "\t'z' to decrease clockwise.\n"
	  "\t'w' to increase hop height.\n"
	  "\t's' to decrease hop height.\n"
	  "\t'r' to reset model.\n"
    "\t' ' to queue a jump.\n"
	  );
}

/**
  Create a box body part
*/
void createBox(int id)
{
  dMass mass;

  partBody[id] = dBodyCreate(world);
	dMassSetBoxTotal(&mass,partMass[id],partDim[id][0],partDim[id][1],partDim[id][2]);
	dBodySetMass(partBody[id],&mass);
	partGeom[id] = dCreateBox(space,partDim[id][0],partDim[id][1],partDim[id][2]);
	dGeomSetBody(partGeom[id],partBody[id]);
  dBodySetPosition(partBody[id],partPos[id][0],partPos[id][1],partPos[id][2]);
}

/**
  Create a capsule body part
*/
void createCapsule(int id)
{
  dMass mass;

  partBody[id] = dBodyCreate(world);
	dMassSetCapsuleTotal(&mass,partMass[id],3,partDim[id][0],partDim[id][1]);
	dBodySetMass(partBody[id],&mass);
	partGeom[id] = dCreateCapsule(space,partDim[id][0],partDim[id][1]);
	dGeomSetBody(partGeom[id],partBody[id]);
  dBodySetPosition(partBody[id],partPos[id][0],partPos[id][1],partPos[id][2]);
}

/**
  Create all the parts
*/
void createModel()
{
	
  createBox(CrossBar_Part);
  createCapsule(SpringTop_Part);
  createCapsule(SpringBottom_Part);

  // Attach the parts with appropriate joints

  // Tether the hopper to the origin
  joints[CircleLimit_Joint] = dJointCreateBall(world,0);
	dJointAttach(joints[CircleLimit_Joint],partBody[CrossBar_Part],0);
	dJointSetBallAnchor(joints[CircleLimit_Joint],0,0,.5);

  // Connect the crosspiece to the top of the spring
  joints[MainHinge_Joint] = dJointCreateHinge(world,0);
  dJointAttach(joints[MainHinge_Joint],partBody[SpringTop_Part],partBody[CrossBar_Part]);
	dJointSetHingeAnchor(joints[MainHinge_Joint],2.5,0,.5);
	dJointSetHingeAxis(joints[MainHinge_Joint],1,0,0);
	dJointSetHingeParam(joints[MainHinge_Joint],dParamLoStop,-M_PI/3); // Limit the hinge range
	dJointSetHingeParam(joints[MainHinge_Joint],dParamHiStop, M_PI/3);

  // Connect the bottom part of the spring to the top
  joints[MainSpring_Joint] = dJointCreateSlider(world,0);
  dJointAttach(joints[MainSpring_Joint],partBody[SpringBottom_Part],partBody[SpringTop_Part]);
	dJointSetSliderAxis(joints[MainSpring_Joint],0,0,1);
  dJointSetSliderParam(joints[MainSpring_Joint],dParamLoStop,-.5); // Limit spring range
  dJointSetSliderParam(joints[MainSpring_Joint],dParamHiStop, .5);
}

/**
  Put things into their initial configurations
  This currently puts the hopper back the way it
  started, but doesn't change the control variables;
  so it will behave differently each time.
*/
void resetModel()
{
  dQuaternion ident={1,0,0,0};
  for (int ii=0;ii<Part_Count;++ii) {
    dBodySetQuaternion(partBody[ii],ident);
    dBodySetPosition(partBody[ii],partPos[ii][0],partPos[ii][1],partPos[ii][2]);
    dBodySetLinearVel(partBody[ii],0,0,0);
    dBodySetAngularVel(partBody[ii],0,0,0);
  }
}

/**
  Draw the model
*/
void renderModel()
{
  // Draw the different body parts
  for (int ii=0;ii<Part_Count;++ii) {
    int partType = dGeomGetClass(partGeom[ii]);
    const dReal* pos = dGeomGetPosition(partGeom[ii]);
    dQuaternion q;
    dGeomGetQuaternion(partGeom[ii],q);

    switch (partType) {
    case dBoxClass: 
    {
      dVector3 sides;
      dGeomBoxGetLengths(partGeom[ii],sides);
      dsDrawBox (dGeomGetPosition(partGeom[ii]),dGeomGetRotation(partGeom[ii]),sides);
    } break;
    case dCapsuleClass:
    {
      dReal rad,len;
      dGeomCapsuleGetParams(partGeom[ii],&rad,&len);
      dsDrawCapsule(dGeomGetPosition(partGeom[ii]),dGeomGetRotation(partGeom[ii]),len,rad);
    } break;
    }
  }

  // Draw the tether
  const dReal* pos = dGeomGetPosition(partGeom[CrossBar_Part]);
  dVector3 tether = {0,0,0.5,0};
  dsDrawLine(pos,tether);
}

/**
  Apply the appropiate control signals
*/
void controlModel()
{
  // Aggregate the required state data

  // Spring state
  dReal zz = dJointGetSliderPosition(joints[MainSpring_Joint]);
  dReal zzDot = dJointGetSliderPositionRate(joints[MainSpring_Joint]);
  // Hinge state
	dReal theta = dJointGetHingeAngle(joints[MainHinge_Joint]);
	dReal thetaDot = dJointGetHingeAngleRate(joints[MainHinge_Joint]);

  // Orientation, position, and velocity of crosspiece
	const dReal* bodyRot = dBodyGetRotation(partBody[CrossBar_Part]);
	const dReal* bodyVel = dBodyGetLinearVel(partBody[CrossBar_Part]);
	const dReal* bodyPos = dBodyGetPosition(partBody[CrossBar_Part]);
  
  // Project body position onto the ground
	dVector3 centerPos;
	centerPos[0]=bodyPos[0];
	centerPos[1]=bodyPos[1];
	centerPos[2]=0;
	dNormalize3(centerPos);
	centerPos[0]*=2.5;
	centerPos[1]*=2.5;
	centerPos[2]=0;


	// It seems that we're making life more difficult for ourselves by
	// duplicating Raibert's circular setup instead of moving linearly.

  // We need to find the current velocity around the circle.
	// Take the cross product of z and the position
	// Take the dot product of the result and the velocity
	dVector3 zAxis = {0,0,1,0};
	dVector3 posVec;
	posVec[0] = bodyPos[0]; posVec[1]=bodyPos[1]; posVec[2] = bodyPos[2]-0.5;
	dNormalize3(posVec);
	
  dVector3 tangent;
	dCROSS(tangent,=,zAxis,posVec);
	dNormalize3(tangent);
	dVector3 upVec;
	dCROSS(upVec,=,posVec,tangent);

  // Now that we've computed 'up' and 'forward' relative to the sphere
  // defined by the tether, we can compute the forward and upward velocities.
	dReal ccwVel = dDOT(tangent,bodyVel);
	dReal upVel = dDOT(upVec,bodyVel);
	
  // Compute the orientation of the crossbar
  // relative to the tangent direction
	dVector3 dipEdge;
	dBodyGetRelPointPos(partBody[CrossBar_Part],0,1,0,dipEdge);
	dipEdge[0]-=bodyPos[0];
	dipEdge[1]-=bodyPos[1];
	dipEdge[2]-=bodyPos[2];
	dReal hPro = dDOT(dipEdge,tangent);
	double phi = atan2(dipEdge[2],hPro);
	double phiDot = (phi-oldPhi)/stepSize;
	oldPhi=phi;

	// Find the global location of the foot-tip
  // Determine if hopper is on the ground
  dReal legLength = partDim[SpringBottom_Part][1]/2 + partDim[SpringBottom_Part][0];
	dVector3 touchPoint;
  dGeomGetRelPointPos(partGeom[SpringBottom_Part],
    0,0,-legLength,touchPoint);
	bool touchingFloor = touchPoint[2]<=0;

  // Compute how long we're on the ground or in the air
	if (touchingFloor==prevTouch) {
		touchTime++;
	} else {
		touchTime=1;
		prevTouch=touchingFloor;
	}
	
  dReal effectiveSetPoint = setPoint;
  // We stop pushing once the spring has extended a bit or leaves the ground
	if (touchingFloor&&zz>-.2) effectiveSetPoint+=pushVal;

  // Compute the explicit spring forces
  dReal springForce=0;
	springForce += -springKp*(zz-effectiveSetPoint);
	springForce += -springKd*zzDot;
  
	dReal targetPos;   // This will hold how far ahead we're aiming the foot.

  // We attempt to place the foot so that it spends half the time in front
  // of the center of mass and half the time behind, unless we're not at our 
  // target velocity.
	targetPos = ccwVel*.0025 + .05*(ccwVel-desiredVel);
  // Prevent the leg from trying to reach farther than it is long.
  if (targetPos>legLength) targetPos=legLength;
	if (targetPos<-legLength) targetPos=-legLength;

  dReal targetAngle;  // This is the leg angle that will get the foot where we're aiming.
	targetAngle = asin(targetPos/legLength)-phi;

  // Apply jumping force
	if (jumping&&touchingFloor) {
    springForce-=200;
    jumpTime-=stepSize;
    if (jumpTime<=0) jumping = false;
  }
  dJointAddSliderForce(joints[MainSpring_Joint],springForce);

  // Compute and apply hinge force
	dReal hingeTorque=0;
	if (!touchingFloor) {
    // Prepare the foot for placement on the ground
		hingeTorque += -tGain*(theta-targetAngle)-tDotGain*thetaDot;
	} else {
    // Torque the crosspiece to control angular velocity
		hingeTorque += -phiGain*(phi-0) - phiDotGain*phiDot;
	}

  dJointAddHingeTorque(joints[MainHinge_Joint],hingeTorque);

}

// simulation loop
static void simLoop (int pause)
{
  if (!pause) {
    controlModel();
    dSpaceCollide (space,0,&nearCallback);
    dWorldStep (world,stepSize);

    // remove all contact joints
    dJointGroupEmpty (contactgroup);
  }

  renderModel();
}


void initializeWorld()
{
  // create world
  dInitODE2(0);
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-gravity);
  ground = dCreatePlane (space,0,0,1,0);

  // Setup constraint satisfaction parameters.
	dWorldSetERP (world, 0.2);      // How hard the world pushes to fix unsatisfied constraints
	dWorldSetCFM(world, 0.00001);   // Constraint force mixing (how squishy constraints are)
	dWorldSetContactSurfaceLayer(world,.001); // How deeply an object can penetrate without violating contacts
}

int main (int argc, char **argv)
{
  int i,j;
  dMass m;

  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;


  initializeWorld();  
  createModel();
  
  

  // run simulation
  dsSimulationLoop (argc,argv,352,288,&fn);

  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
  return 0;
}
