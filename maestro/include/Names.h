/*
Copyright (c) 2013, Drexel University, iSchool, Applied Informatics Group
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef _NAMES_H
#define _NAMES_H

enum PROPERTY {
	POSITION, VELOCITY, TEMPERATURE, CURRENT, HOMED, ZEROED, ENABLED,
	ERRORED, JAM_ERROR, PWM_SATURATED_ERROR, BIG_ERROR, ENC_ERROR, DRIVE_FAULT_ERROR,
	POS_MIN_ERROR, POS_MAX_ERROR, VELOCITY_ERROR, ACCELERATION_ERROR, TEMP_ERROR,
	X_ACCEL, Y_ACCEL, Z_ACCEL, X_ROTAT, Y_ROTAT,
	M_X, M_Y, F_Z
};

enum COMMAND {
	ENABLE, ENABLEALL,
	DISABLE, DISABLEALL,
	RESET, RESETALL,
	HOME, HOMEALL,
	INITSENSORS,
	UPDATE
};

#endif
