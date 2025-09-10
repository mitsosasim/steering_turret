/* =======================================================================================================
  CUSim.h
  Created on: Feb 25, 2025

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
    R. Filippone
    A. Kamal Eddine

 ------------------------------------------------------------------------------
  v. 1.0 - Feb 25, 2025 - Short description
 ------------------------------------------------------------------------------
  Reference:
    R1 - Reference (document, web site, other source files, etc.)
    R2
 ------------------------------------------------------------------------------
  Functions and logics

======================================================================================================= */
#ifndef CUSIM_H_
#define CUSIM_H_

void* ThSIMSender(void* arg);
void* ThSIMReceiver(void* arg);

#define MAXLINE 1024


// System commands
void SimCheckUDP();							// Check UDP connection
void SimAskBIT();								// Ask the Turret to perform a complete BIT
void SimAskStatus();							// Request the Turret full current status
void SimSetStart();								// Start Turret activities
void SimSetStop();								// Set the Turret in safe mode and stop all activities
void SimSetToZero();							// Move all motors to their zero angle (initial position)
void StartLevelling();							// Start Self-level

// Turret commands
void SimMoveTurretTo(float AbsAngle);			// move the Turret to absolute position
void SimMoveTurretBy(float RelAngle);			// move the Turret by a relative angle +/- alfa

// Mirrors commands
void SimMoveMirrorsTo(float AbsAngle);			// move all mirrors to absolute position
void SimMoveMirrorsBy(float RelAngle);			// move all mirrors by a relative angle +/- alfa
void SimMoveMirrorTo(int id,float AbsAngle);	// move mirror id to absolute position
void SimMoveMirrorBy(int id,float RelAngle);	// move mirror by a relative angle +/- alfa

// SUTs commands
void SimMoveSUTsTo(float AbsAngle);				// move all SUTs to absolute position
void SimMoveSUTsBy(int id,float RelAngle);		// move all SUTs by a relative angle +/- alfa
void SimMoveSUTTo(int id,float AbsAngle);		// move SUT id to absolute position
void SimMoveSUTBy(int id,float RelAngle);		// move SUT id by a relative angle +/- alfa

// Laser commands
void SimLasers(int light);						// Switch all laser 1=on/0=off
void SimLaser(int id,int light);				// Switch laser id 1=on/0=off

// Camera commands
void MoveCameraBy(float RelAngle);				// Move camera motor by a relative angle +/- alfa

#endif /* CUSIM_H_ */
