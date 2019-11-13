; This program includes...
; - Robot initialization (checking the battery, stopping motors, etc.).
; - The movement API.
; - Several useful subroutines (ATAN2, Neg, Abs, mult, div).
; - Some useful constants (masks, numbers, robot stuff, etc.)

; This code uses the timer interrupt for the movement control code.
; The ISR jump table is located in mem 0-4.  See manual for details.
ORG 0
	JUMP   Init        ; Reset vector
	RETI               ; Sonar interrupt (unused)
	JUMP   CTimer_ISR  ; Timer interrupt
	RETI               ; UART interrupt (unused)
	RETI               ; Motor stall interrupt (unused)

;***************************************************************
;* Initialization
;***************************************************************
Init:
	; Always a good idea to make sure the robot
	; stops in the event of a reset.
	LOAD   Zero
	OUT    LVELCMD     ; Stop motors
	OUT    RVELCMD
	STORE  DVel        ; Reset API variables
	STORE  DTheta
	;OUT    SONAREN     ; Disable sonar (optional)
	OUT    BEEP        ; Stop any beeping (optional)
	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display battery voltage (hex, tenths of volts)
	; Enable all sonar
	LOAD   FullMask
	OUT	   SONAREN

WaitForSafety:
	; This loop will wait for the user to toggle SW17.  Note that
	; SCOMP does not have direct access to SW17; it only has access
	; to the SAFETY signal contained in XIO.
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser ; If ready, jump to wait for PB3
	IN     TIMER       ; We'll use the timer value to
	AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
	SHIFT  8           ; Shift over to LED17
	OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP   WaitForSafety
	
WaitForUser:
	; This loop will wait for the user to press PB3, to ensure that
	; they have a chance to prepare for any movement in the main code.
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  5           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue

;***************************************************************
;* Main code
;***************************************************************
Main:
	OUT    RESETPOS    ; reset the odometry to 0,0,0
	; configure timer interrupt for the movement control code
	LOADI  10          ; period = (10 ms * 10) = 0.1s, or 10Hz.
	OUT    CTIMER      ; turn on timer peripheral
	SEI    &B0010      ; enable interrupts from source 2 (timer)
	; at this point, timer interrupts will be firing at 10Hz, and
	; code in that ISR will attempt to control the robot.
	; If you want to take manual control of the robot,
	; execute CLI &B0010 to disable the timer interrupt.
	
	;LOADI  90
	;STORE  DTheta      ; use API to get robot to face 90 degrees
; TurnLoop:
; 	IN     Theta
; 	ADDI   -90
; 	CALL   Abs         ; get abs(currentAngle - 90)
; 	ADDI   -3
; 	JPOS   TurnLoop    ; if angle error > 3, keep checking
; 	; at this point, robot should be within 3 degrees of 90
; 	LOAD   FMid
; 	STORE  DVel        ; use API to move forward

InfLoop: 
	JUMP   InfLoop
	; note that the movement API will still be running during this
	; infinite loop, because it uses the timer interrupt, so the
	; robot will continue to attempt to match DTheta and DVel
	
	

Die:
; Sometimes it's useful to permanently stop execution.
; This will also catch the execution if it accidentally
; falls through from above.
	CLI    &B1111      ; disable all interrupts
	LOAD   Zero        ; Stop everything.
	OUT    LVELCMD
	OUT    RVELCMD
	OUT    SONAREN
	LOAD   DEAD        ; An indication that we are dead
	OUT    SSEG2       ; "dEAd" on the sseg
Forever:
	JUMP   Forever     ; Do this forever.
	DEAD:  DW &HDEAD   ; Example of a "local" variable


; Timer ISR.  Currently just calls the movement control code.
; You could, however, do additional tasks here if desired.
CTimer_ISR:
	; check state then let that state handle movement variables
	LOAD	STATE
	XOR		TEST1
	JZERO	HandleTest1State
	XOR		TEST2
	JZERO	HandleTest2State
	XOR TEST3
	JZERO	HandleTest3State
GoDoMvmt:
	CALL   ControlMovement
	RETI   ; return from ISR

HandleTest1State:
	; move for three seconds then stop for one second
	; edit: stop for 3 seconds then move for 1
	LOAD	counter			; read counter
	ADDI	1				; increment counter
	STORE	counter
	OUT		LCD				; debug
	ADDI 	-30				; check if we've hit 30 (3 seconds)
	JNEG	SkipThis		; if not, keep moving
	OUT		LCD				; debug
	ADDI	-10				; check if we've hit 10 (1 second) 	
	JNEG	SetVel			; if not, don't reset our counter
	OUT		LCD				; debug
	LOADI	0				
	STORE	counter			; reset counter if so
; set movement velocity to 0 (stop)
SetVel:
	LOADI	0				; get zero in case AC isn't zero before - swap this to actual vel
	OUT		LCD				; debug
	STORE 	DVel
	JUMP GoDoMvmt			; let the MoveAPI do all our heavy lifting
SkipThis:					; move forward slowly
	LOAD	FSlow			; swap this to zero
	STORE	DVel
	JUMP	GoDoMvmt		; let the MoveAPI do all our heavy lifting
	;***********************************************************
	;* Local vars for this state
	;***********************************************************
	counter:	DW &H0000
HandleTest2State:
	;; checks sensors 0-5 (not the back two for now)
	;; directs DE2 towards which sensor detects something closest
	;; excluding certain sensors (6 and 7)
	;; if remaining distance is < 1 ft set DVel to 0
	;; do every 1 seconds
	LOAD	stopIt				
	JPOS	StopDetect			
	; added a precheck just in case -- eventually this can be taken out
	; through use of a state change at StopDetect
	; put the range check up here since it should be a precheck
	CALL	VerifyRange			; check how close we are
	LOAD	closeEnough			; 0 if too far, 1 if <= 250
	JZERO	SkipHeadingCheck	; if not within 250, skip the Heading Check
	CALL	VerifyHeading
	LOAD	stopIt				
	JPOS	StopDetect			; 0 if don't stop, 1 if stop
SkipHeadingCheck:
	LOAD	checker
	ADDI	1
	OUT		SSEG1
	STORE	checker
	ADDI	-5
	JNEG	StopTurning
	LOADI	0
	STORE	checker
	CALL	DetectReflector
	; check that the value is not -1
	LOAD	sensor_num
	JNEG	nothingDetected
	; otherwise, figure out which sensor detected stuff
	; and do necessary checks
	;; original spot
	; CALL	VerifyRange			; check how close we are
Check:
	LOAD	sensor_num
	;OUT		LCD					; debug
	XOR		Mask0
	JZERO	Sensor0Detected		; sensor_num => 0	
	LOAD	sensor_num
	XOR		Mask1
	JZERO	Sensor1Detected		; sensor_num => 1
	LOAD	sensor_num
	XOR		Mask2
	JZERO	Sensor2Detected		; sensor_num => 2
	LOAD	sensor_num
	XOR		Mask3
	JZERO	Sensor3Detected		; sensor_num => 3
	LOAD	sensor_num
	XOR		Mask4
	JZERO	Sensor4Detected		; sensor_num => 4
	LOAD	sensor_num
	XOR		Mask5
	JZERO	Sensor5Detected		; sensor_num => 5
	; if it falls through, let's not change the angle
	LOADI	0
	STORE	changeTheta
	JUMP	SetTargetAngle
Sensor0Detected:
	LOADI	90
	;OUT		SSEG1
	STORE	changeTheta
	JUMP	SetTargetAngle
Sensor1Detected:
	LOADI	44
	;OUT		SSEG1
	STORE	changeTheta
	JUMP	SetTargetAngle
Sensor2Detected:
	LOADI	12
	;OUT		SSEG1
	STORE	changeTheta
	; OUT		LCD
	JUMP	SetTargetAngle
Sensor3Detected:
	LOADI	-12
	;OUT		SSEG1
	STORE	changeTheta
	JUMP	SetTargetAngle
Sensor4Detected:
	LOADI	-44
	;OUT		SSEG1
	STORE	changeTheta
	JUMP	SetTargetAngle
Sensor5Detected:
	LOADI	-90
	;OUT		SSEG1
	STORE	changeTheta
SetTargetAngle:
	LOAD	changeTheta
	;OUT		SSEG2
	IN		THETA
	ADD		changeTheta
	; get the new DTheta
	STORE	currTarg
; Set Velocity in case of distance < 1 Ft or error
CheckSensedDistance:
	LOADI	-1
	XOR		min
	JZERO	nothingDetected		;; shouldn't happen; if min = -1 we should've already jumped in earlier check
	;LOAD	min					;; load the measured distance
	;SUB	Ft1					;; subtract 1 ft from that value (if negative then we should set DVel to 0)
	;JNEG	StopTurning
; This is for possible cases where we don't need to change currTarg
SetTargetHeading:
	; assumes that the target angle is stored in currTarg
	; set DVel to 0 just in case
	LOAD	currTarg	
	STORE	DTheta
	LOAD	closeEnough
	JPOS	DontMoveFwd
	LOAD	FSlow
	STORE	DVel				; if not within 250, set DVel to FSlow (move forwards)
;	LOADI	0
;	STORE	DVel
	JUMP	GoDoMvmt
DontMoveFwd:
	LOADI	0					; if within 250, stop moving forwards
	OUT		LCD					; debug
	STORE	DVel
; This is for when the sensors don't detect ANYTHING
nothingDetected:
	LOAD	tempConst
	OUT		LCD
;	LOADI	0
;	STORE	DVel
	JUMP	GoDoMvmt	; not yet decided how to handle this case yet
StopTurning:
;	LOADI	0
;	STORE	DVel
	JUMP SetTargetHeading
StopDetect:
	; the doNothing() code section
	; eventually we'll replace this with
	; a state change
	JUMP	GoDoMvmt
	;***********************************************************
	;* Local vars for this state
	;***********************************************************
	tempConst:	 DW &H1134
	closeEnough: DW &H0000			; reset when we reset back to this state
	stopIt:	 	 DW &H0000			; reset when we reset back to this state
	checker:	 DW	&H0000			; reset when we reset back to this state
	changeTheta: DW &H0000			; reset when we reset back to this state
	sensor_num:	 DW &H0000	
	currTarg:	 DW &H0000			; reset when we reset back to this state
HandleTest3State:
	; attempt to loop in a
	; circle with 1/2 ft radius (293/2 = 146.5)
	; change to 1 foot, half all values
	; 293(pi) mm circumference = ~920.5 mm
	; divide 105 mm/s --> 8.77s ~ 9s ==> 41.05 degrees per sec
		; approximately 4 degrees per 0.1s (2s)
	; 367 mm/s --> 2.51s ~ 3s ==> 143.426 degrees per sec
		; approximately 14 - 15 degrees per 0.1 sec (alternate?) (8)
	; 525 mm/s --> 1.753s ~ 2s ==> 205.362 degrees per sec
		; approximately 20 - 21 degrees per 0.1 sec (alternate?) 10
	LOAD	FSlow
	STORE	DVel
	IN		THETA
	ADD		2
	STORE	DTheta
	; add a check to stop?
	IN		DIST0
	XOR		INTEGER_MAX
	JZERO	WeDoneMessedUp
	JUMP	GoDoMvmt
WeDoneMessedUp:
	LOADI	0
	STORE	DVel
	IN		THETA
	STORE	DTheta
	JUMP	GoDoMvmt	
	;***********************************************************
	;* Local vars for this state
	;***********************************************************
VerifyRange:
	LOADI	-1				;0xFFFF
	OUT		LCD
	LOAD	min
	OUT		SSEG2
	XOR		INTEGER_MAX
	JZERO	doNothing
	LOADI	250
	SUB		min
	JNEG	doNothing
	LOADI	1
	STORE	closeEnough
doNothing:
	RETURN
VerifyHeading:
	IN		DIST2
	STORE	d2
	;SUB		HalfFt
	;JPOS	OverHalf
	; Under half a foot away
	IN		DIST3
	SUB		d2
	CALL	Abs
	SUB		diff1
	JNEG	setTrue
	LOADI	0
	STORE	stopIt
	RETURN
;OverHalf:
;	IN		DIST3
;	SUB		d2
;	CALL	Abs
;	SUB		diff2
;	JNEG	setTrue
;	LOADI	0
;	STORE	stopIt
;	RETURN 
setTrue:
	LOADI	1
	STORE	stopIt
	RETURN
	;***********************************************************
	;* Local vars for this state
	;***********************************************************
	d2:		DW &H0000
	diff1:	DW 90			; 90 mm diff
	;diff2:	DW 75			; 75 mm diff
	
;; set sensor_num and min to -1 if nothing is detected
;; else set sensor_num to closest sensor
;; and min to min distance detected
DetectReflector:
	; only checks sensors 0-5 (we'll use rear sensors for moving away)
	; start by initializing min = DIST0 and sensor_num = 0
	LOADI	0
	OUT		SSEG1
	LOAD	Mask0
	STORE	sensor_num
	IN		DIST0
	STORE	min
	; compare dist0 and dist1
	IN		DIST1
	SUB		min		; dist1 - dist0 (if negative, 1 is closer --
	JPOS	not1	; if positive, 0 is closer
	JZERO	not1	; if 0, it doesn't matter but we'll avoid the mem access
	; otherwise, we'll update our values (min = DIST1 and sensor_num = 1)
	IN		DIST1
	STORE	min
	LOAD	Mask1
	STORE	sensor_num
not1:
	; compare the previous min sensor and value to sensor2
	LOADI	1
	OUT		SSEG1
	IN		DIST2
	SUB		min		; if negative the new one is closer
	JPOS	not2	; if positive, old one is closer
	JZERO	not2	; if 0, it doesn't matter but we'll avoid the mem access
	; otherwise, we'll update our values (min = DIST2 and sensor_num = 2)
	IN		DIST2
	STORE	min
	LOAD	Mask2
	STORE	sensor_num
not2:
	; compare the previous min sensor and value to sensor3
	LOADI	2
	OUT		SSEG1
	IN		DIST3
	SUB		min		; if negative the new one is closer
	JPOS	not3	; if positive, old one is closer
	JZERO	not3	; if 0, it doesn't matter but we'll avoid the mem access
	; otherwise, we'll update our values (min = DIST3 and sensor_num = 3)
	IN		DIST3
	STORE	min
	LOAD	Mask3
	STORE	sensor_num
not3:
	; compare the previous min sensor and value to sensor4
	LOADI	3
	OUT		SSEG1
	IN		DIST4
	SUB		min		; if negative the new one is closer
	JPOS	not4	; if positive, old one is closer
	JZERO	not4	; if 0, it doesn't matter but we'll avoid the mem access
	; otherwise, we'll update our values (min = DIST4 and sensor_num = 4)
	IN		DIST4
	STORE	min
	LOAD	Mask4
	STORE	sensor_num
not4:
	; compare the previous min sensor and value to sensor5
	LOADI	4
	OUT		SSEG1
	IN		DIST5
	SUB		min		; if negative the new one is closer
	JPOS	not5	; if positive, old one is closer
	JZERO	not5	; if 0, it doesn't matter but we'll avoid the mem access
	; otherwise, we'll update our values (min = DIST5 and sensor_num = 5)
	IN		DIST5
	STORE	min
	LOAD	Mask5
	STORE	sensor_num
not5:
	; Lastly perform a check that something was actually sensed
	; "if nothing is sensed, the measured value is set to 0x7FFF" - DE2 handbook
	LOADI	5
	OUT		SSEG1
	LOAD	INTEGER_MAX
	XOR		min			; if they're equal then we haven't sensed anything yet
	JZERO	setFail		; although this shouldn't happen as often
						; because max sensor range is 6 m  = ~20 ft = ~10 tiles
	RETURN
setFail:
	LOADI	-1
	OUT		LCD			; debug
	STORE 	sensor_num
	STORE	min
	RETURN
	;***********************************************************
	;* Local vars for this state
	;***********************************************************
	min:		DW &H7FFF			; set min to INT_MAX
									; when we reset back to this state
									; (detecting next reflector to loop)
									; we need to reset min INTEGER_MAX
; Control code.  If called repeatedly, this code will attempt
; to control the robot to face the angle specified in DTheta
; and match the speed specified in DVel
DTheta:    DW 0
DVel:      DW 0
ControlMovement:
	LOADI  50          ; used for the CapValue subroutine
	STORE  MaxVal
	CALL   GetThetaErr ; get the heading error
	; A simple way to get a decent velocity value
	; for turning is to multiply the angular error by 4
	; and add ~50.
	SHIFT  2
	STORE  CMAErr      ; hold temporarily
	SHIFT  2           ; multiply by another 4
	CALL   CapValue    ; get a +/- max of 50
	ADD    CMAErr
	STORE  CMAErr      ; now contains a desired differential

	
	; For this basic control method, simply take the
	; desired forward velocity and add the differential
	; velocity for each wheel when turning is needed.
	LOADI  510
	STORE  MaxVal
	LOAD   DVel
	CALL   CapValue    ; ensure velocity is valid
	STORE  DVel        ; overwrite any invalid input
	ADD    CMAErr
	CALL   CapValue    ; ensure velocity is valid
	STORE  CMAR
	LOAD   CMAErr
	CALL   Neg         ; left wheel gets negative differential
	ADD    DVel
	CALL   CapValue
	STORE  CMAL

	; ensure enough differential is applied
	LOAD   CMAErr
	SHIFT  1           ; double the differential
	STORE  CMAErr
	LOAD   CMAR
	SUB    CMAL        ; calculate the actual differential
	SUB    CMAErr      ; should be 0 if nothing got capped
	JZERO  CMADone
	; re-apply any missing differential
	STORE  CMAErr      ; the missing part
	ADD    CMAL
	CALL   CapValue
	STORE  CMAL
	LOAD   CMAR
	SUB    CMAErr
	CALL   CapValue
	STORE  CMAR

CMADone:
	LOAD   CMAL
	OUT    LVELCMD
	LOAD   CMAR
	OUT    RVELCMD

	RETURN
	CMAErr: DW 0       ; holds angle error velocity
	CMAL:    DW 0      ; holds temp left velocity
	CMAR:    DW 0      ; holds temp right velocity

; Returns the current angular error wrapped to +/-180
GetThetaErr:
	; convenient way to get angle error in +/-180 range is
	; ((error + 180) % 360 ) - 180
	IN     THETA
	SUB    DTheta      ; actual - desired angle
	CALL   Neg         ; desired - actual angle
	ADDI   180
	CALL   Mod360
	ADDI   -180
	RETURN

; caps a value to +/-MaxVal
CapValue:
	SUB     MaxVal
	JPOS    CapVelHigh
	ADD     MaxVal
	ADD     MaxVal
	JNEG    CapVelLow
	SUB     MaxVal
	RETURN
CapVelHigh:
	LOAD    MaxVal
	RETURN
CapVelLow:
	LOAD    MaxVal
	CALL    Neg
	RETURN
	MaxVal: DW 510


;*******************************************************************************
; Mod360: modulo 360
; Returns AC%360 in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Mod360:
	; easy modulo: subtract 360 until negative then add 360 until not negative
	JNEG   M360N
	ADDI   -360
	JUMP   Mod360
M360N:
	ADDI   360
	JNEG   M360N
	RETURN

;*******************************************************************************
; Abs: 2's complement absolute value
; Returns abs(AC) in AC
; Neg: 2's complement negation
; Returns -AC in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Abs:
	JPOS   Abs_r
Neg:
	XOR    NegOne       ; Flip all bits
	ADDI   1            ; Add one (i.e. negate number)
Abs_r:
	RETURN

;******************************************************************************;
; Atan2: 4-quadrant arctangent calculation                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Original code by Team AKKA, Spring 2015.                                     ;
; Based on methods by Richard Lyons                                            ;
; Code updated by Kevin Johnson to use software mult and div                   ;
; No license or copyright applied.                                             ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; To use: store dX and dY in global variables AtanX and AtanY.                 ;
; Call Atan2                                                                   ;
; Result (angle [0,359]) is returned in AC                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Requires additional subroutines:                                             ;
; - Mult16s: 16x16->32bit signed multiplication                                ;
; - Div16s: 16/16->16R16 signed division                                       ;
; - Abs: Absolute value                                                        ;
; Requires additional constants:                                               ;
; - One:     DW 1                                                              ;
; - NegOne:  DW 0                                                              ;
; - LowByte: DW &HFF                                                           ;
;******************************************************************************;
Atan2:
	LOAD   AtanY
	CALL   Abs          ; abs(y)
	STORE  AtanT
	LOAD   AtanX        ; abs(x)
	CALL   Abs
	SUB    AtanT        ; abs(x) - abs(y)
	JNEG   A2_sw        ; if abs(y) > abs(x), switch arguments.
	LOAD   AtanX        ; Octants 1, 4, 5, 8
	JNEG   A2_R3
	CALL   A2_calc      ; Octants 1, 8
	JNEG   A2_R1n
	RETURN              ; Return raw value if in octant 1
A2_R1n: ; region 1 negative
	ADDI   360          ; Add 360 if we are in octant 8
	RETURN
A2_R3: ; region 3
	CALL   A2_calc      ; Octants 4, 5            
	ADDI   180          ; theta' = theta + 180
	RETURN
A2_sw: ; switch arguments; octants 2, 3, 6, 7 
	LOAD   AtanY        ; Swap input arguments
	STORE  AtanT
	LOAD   AtanX
	STORE  AtanY
	LOAD   AtanT
	STORE  AtanX
	JPOS   A2_R2        ; If Y positive, octants 2,3
	CALL   A2_calc      ; else octants 6, 7
	CALL   Neg          ; Negatge the number
	ADDI   270          ; theta' = 270 - theta
	RETURN
A2_R2: ; region 2
	CALL   A2_calc      ; Octants 2, 3
	CALL   Neg          ; negate the angle
	ADDI   90           ; theta' = 90 - theta
	RETURN
A2_calc:
	; calculates R/(1 + 0.28125*R^2)
	LOAD   AtanY
	STORE  d16sN        ; Y in numerator
	LOAD   AtanX
	STORE  d16sD        ; X in denominator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  AtanRatio
	STORE  m16sA
	STORE  m16sB
	CALL   A2_mult      ; X^2
	STORE  m16sA
	LOAD   A2c
	STORE  m16sB
	CALL   A2_mult
	ADDI   256          ; 256/256+0.28125X^2
	STORE  d16sD
	LOAD   AtanRatio
	STORE  d16sN        ; Ratio in numerator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  m16sA        ; <= result in radians
	LOAD   A2cd         ; degree conversion factor
	STORE  m16sB
	CALL   A2_mult      ; convert to degrees
	STORE  AtanT
	SHIFT  -7           ; check 7th bit
	AND    One
	JZERO  A2_rdwn      ; round down
	LOAD   AtanT
	SHIFT  -8
	ADDI   1            ; round up
	RETURN
A2_rdwn:
	LOAD   AtanT
	SHIFT  -8           ; round down
	RETURN
A2_mult: ; multiply, and return bits 23..8 of result
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8            ; move high word of result up 8 bits
	STORE  mres16sH
	LOAD   mres16sL
	SHIFT  -8           ; move low word of result down 8 bits
	AND    LowByte
	OR     mres16sH     ; combine high and low words of result
	RETURN
A2_div: ; 16-bit division scaled by 256, minimizing error
	LOADI  9            ; loop 8 times (256 = 2^8)
	STORE  AtanT
A2_DL:
	LOAD   AtanT
	ADDI   -1
	JPOS   A2_DN        ; not done; continue shifting
	CALL   Div16s       ; do the standard division
	RETURN
A2_DN:
	STORE  AtanT
	LOAD   d16sN        ; start by trying to scale the numerator
	SHIFT  1
	XOR    d16sN        ; if the sign changed,
	JNEG   A2_DD        ; switch to scaling the denominator
	XOR    d16sN        ; get back shifted version
	STORE  d16sN
	JUMP   A2_DL
A2_DD:
	LOAD   d16sD
	SHIFT  -1           ; have to scale denominator
	STORE  d16sD
	JUMP   A2_DL
AtanX:      DW 0
AtanY:      DW 0
AtanRatio:  DW 0        ; =y/x
AtanT:      DW 0        ; temporary value
A2c:        DW 72       ; 72/256=0.28125, with 8 fractional bits
A2cd:       DW 14668    ; = 180/pi with 8 fractional bits

;*******************************************************************************
; Mult16s:  16x16 -> 32-bit signed multiplication
; Based on Booth's algorithm.
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: does not work with factor B = -32768 (most-negative number).
; To use:
; - Store factors in m16sA and m16sB.
; - Call Mult16s
; - Result is stored in mres16sH and mres16sL (high and low words).
;*******************************************************************************
Mult16s:
	LOADI  0
	STORE  m16sc        ; clear carry
	STORE  mres16sH     ; clear result
	LOADI  16           ; load 16 to counter
Mult16s_loop:
	STORE  mcnt16s      
	LOAD   m16sc        ; check the carry (from previous iteration)
	JZERO  Mult16s_noc  ; if no carry, move on
	LOAD   mres16sH     ; if a carry, 
	ADD    m16sA        ;  add multiplicand to result H
	STORE  mres16sH
Mult16s_noc: ; no carry
	LOAD   m16sB
	AND    One          ; check bit 0 of multiplier
	STORE  m16sc        ; save as next carry
	JZERO  Mult16s_sh   ; if no carry, move on to shift
	LOAD   mres16sH     ; if bit 0 set,
	SUB    m16sA        ;  subtract multiplicand from result H
	STORE  mres16sH
Mult16s_sh:
	LOAD   m16sB
	SHIFT  -1           ; shift result L >>1
	AND    c7FFF        ; clear msb
	STORE  m16sB
	LOAD   mres16sH     ; load result H
	SHIFT  15           ; move lsb to msb
	OR     m16sB
	STORE  m16sB        ; result L now includes carry out from H
	LOAD   mres16sH
	SHIFT  -1
	STORE  mres16sH     ; shift result H >>1
	LOAD   mcnt16s
	ADDI   -1           ; check counter
	JPOS   Mult16s_loop ; need to iterate 16 times
	LOAD   m16sB
	STORE  mres16sL     ; multiplier and result L shared a word
	RETURN              ; Done
c7FFF: DW &H7FFF
m16sA: DW 0 ; multiplicand
m16sB: DW 0 ; multipler
m16sc: DW 0 ; carry
mcnt16s: DW 0 ; counter
mres16sL: DW 0 ; result low
mres16sH: DW 0 ; result high

;*******************************************************************************
; Div16s:  16/16 -> 16 R16 signed division
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: results undefined if denominator = 0.
; To use:
; - Store numerator in d16sN and denominator in d16sD.
; - Call Div16s
; - Result is stored in dres16sQ and dres16sR (quotient and remainder).
; Requires Abs subroutine
;*******************************************************************************
Div16s:
	LOADI  0
	STORE  dres16sR     ; clear remainder result
	STORE  d16sC1       ; clear carry
	LOAD   d16sN
	XOR    d16sD
	STORE  d16sS        ; sign determination = N XOR D
	LOADI  17
	STORE  d16sT        ; preload counter with 17 (16+1)
	LOAD   d16sD
	CALL   Abs          ; take absolute value of denominator
	STORE  d16sD
	LOAD   d16sN
	CALL   Abs          ; take absolute value of numerator
	STORE  d16sN
Div16s_loop:
	LOAD   d16sN
	SHIFT  -15          ; get msb
	AND    One          ; only msb (because shift is arithmetic)
	STORE  d16sC2       ; store as carry
	LOAD   d16sN
	SHIFT  1            ; shift <<1
	OR     d16sC1       ; with carry
	STORE  d16sN
	LOAD   d16sT
	ADDI   -1           ; decrement counter
	JZERO  Div16s_sign  ; if finished looping, finalize result
	STORE  d16sT
	LOAD   dres16sR
	SHIFT  1            ; shift remainder
	OR     d16sC2       ; with carry from other shift
	SUB    d16sD        ; subtract denominator from remainder
	JNEG   Div16s_add   ; if negative, need to add it back
	STORE  dres16sR
	LOADI  1
	STORE  d16sC1       ; set carry
	JUMP   Div16s_loop
Div16s_add:
	ADD    d16sD        ; add denominator back in
	STORE  dres16sR
	LOADI  0
	STORE  d16sC1       ; clear carry
	JUMP   Div16s_loop
Div16s_sign:
	LOAD   d16sN
	STORE  dres16sQ     ; numerator was used to hold quotient result
	LOAD   d16sS        ; check the sign indicator
	JNEG   Div16s_neg
	RETURN
Div16s_neg:
	LOAD   dres16sQ     ; need to negate the result
	CALL   Neg
	STORE  dres16sQ
	RETURN	
d16sN: DW 0 ; numerator
d16sD: DW 0 ; denominator
d16sS: DW 0 ; sign value
d16sT: DW 0 ; temp counter
d16sC1: DW 0 ; carry value
d16sC2: DW 0 ; carry value
dres16sQ: DW 0 ; quotient result
dres16sR: DW 0 ; remainder result

;*******************************************************************************
; L2Estimate:  Pythagorean distance estimation
; Written by Kevin Johnson.  No license or copyright applied.
; Warning: this is *not* an exact function.  I think it's most wrong
; on the axes, and maybe at 45 degrees.
; To use:
; - Store X and Y offset in L2X and L2Y.
; - Call L2Estimate
; - Result is returned in AC.
; Result will be in same units as inputs.
; Requires Abs and Mult16s subroutines.
;*******************************************************************************
L2Estimate:
	; take abs() of each value, and find the largest one
	LOAD   L2X
	CALL   Abs
	STORE  L2T1
	LOAD   L2Y
	CALL   Abs
	SUB    L2T1
	JNEG   GDSwap    ; swap if needed to get largest value in X
	ADD    L2T1
CalcDist:
	; Calculation is max(X,Y)*0.961+min(X,Y)*0.406
	STORE  m16sa
	LOADI  246       ; max * 246
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	STORE  L2T3
	LOAD   L2T1
	STORE  m16sa
	LOADI  104       ; min * 104
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	ADD    L2T3     ; sum
	RETURN
GDSwap: ; swaps the incoming X and Y
	ADD    L2T1
	STORE  L2T2
	LOAD   L2T1
	STORE  L2T3
	LOAD   L2T2
	STORE  L2T1
	LOAD   L2T3
	JUMP   CalcDist
L2X:  DW 0
L2Y:  DW 0
L2T1: DW 0
L2T2: DW 0
L2T3: DW 0


; Subroutine to wait (block) for 1 second
Wait1:
	OUT    TIMER
Wloop:
	IN     TIMER
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	ADDI   -10         ; 1 second at 10Hz.
	JNEG   Wloop
	RETURN

; This subroutine will get the battery voltage,
; and stop program execution if it is too low.
; SetupI2C must be executed prior to this.
BattCheck:
	CALL   GetBattLvl
	JZERO  BattCheck   ; A/D hasn't had time to initialize
	SUB    MinBatt
	JNEG   DeadBatt
	ADD    MinBatt     ; get original value back
	RETURN
; If the battery is too low, we want to make
; sure that the user realizes it...
DeadBatt:
	LOADI  &H20
	OUT    BEEP        ; start beep sound
	CALL   GetBattLvl  ; get the battery level
	OUT    SSEG1       ; display it everywhere
	OUT    SSEG2
	OUT    LCD
	LOAD   Zero
	ADDI   -1          ; 0xFFFF
	OUT    LEDS        ; all LEDs on
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	LOADI  &H140       ; short, high-pitched beep
	OUT    BEEP        ; stop beeping
	LOAD   Zero
	OUT    LEDS        ; LEDs off
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	JUMP   DeadBatt    ; repeat forever
	
; Subroutine to read the A/D (battery voltage)
; Assumes that SetupI2C has been run
GetBattLvl:
	LOAD   I2CRCmd     ; 0x0190 (write 0B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	IN     I2C_DATA    ; get the returned data
	RETURN

; Subroutine to configure the I2C for reading batt voltage
; Only needs to be done once after each reset.
SetupI2C:
	CALL   BlockI2C    ; wait for idle
	LOAD   I2CWCmd     ; 0x1190 (write 1B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD register
	LOAD   Zero        ; 0x0000 (A/D port 0, no increment)
	OUT    I2C_DATA    ; to I2C_DATA register
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	RETURN
	
; Subroutine to block until I2C device is idle
BlockI2C:
	LOAD   Zero
	STORE  Temp        ; Used to check for timeout
BI2CL:
	LOAD   Temp
	ADDI   1           ; this will result in ~0.1s timeout
	STORE  Temp
	JZERO  I2CError    ; Timeout occurred; error
	IN     I2C_RDY     ; Read busy signal
	JPOS   BI2CL       ; If not 0, try again
	RETURN             ; Else return
I2CError:
	LOAD   Zero
	ADDI   &H12C       ; "I2C"
	OUT    SSEG1
	OUT    SSEG2       ; display error message
	JUMP   I2CError

;***************************************************************
;* Variables
;***************************************************************
Temp:     	DW 0 ; "Temp" is not a great name, but can be useful
PositionX: 	DW &H0000
PositionY: 	DW &H0000
STATE:		DW &H0001	; STATE variable -- track the main state

;***************************************************************
;* States
;***************************************************************
TEST1:		DW &B00000000
TEST2:		DW &B00000001
TEST3:		DW &B00000010

;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
NegOne:   		DW -1
Zero:     		DW 0
One:      		DW 1
Two:      		DW 2
Three:    		DW 3
Four:     		DW 4
Five:     		DW 5
Six:   		    DW 6
Seven: 		    DW 7
Eight:  		DW 8
Nine:     		DW 9
Ten:      		DW 10
INTEGER_MAX:	DW &H7FFF

; Some bit masks.
; Masks of multiple bits can be constructed by ORing these
; 1-bit masks together.
Mask0:    DW &B00000001
Mask1:    DW &B00000010
Mask2:    DW &B00000100
Mask3:    DW &B00001000
Mask4:    DW &B00010000
Mask5:    DW &B00100000
Mask6:    DW &B01000000
Mask7:    DW &B10000000
LowByte:  DW &HFF      ; binary 00000000 1111111
LowNibl:  DW &HF       ; 0000 0000 0000 1111
FullMask: DW &HFFFF

; some useful movement values
OneMeter:  DW 961       ; ~1m in 1.04mm units
HalfMeter: DW 481       ; ~0.5m in 1.04mm units
HalfFt:	   DW 147		; ~0.5ft
Ft1:	   DW 293	    ; ~1ft
Ft2:       DW 586       ; ~2ft in 1.04mm units
Ft3:       DW 879
Ft4:       DW 1172
Deg90:     DW 90        ; 90 degrees in odometer units
Deg180:    DW 180       ; 180
Deg270:    DW 270       ; 270
Deg360:    DW 360       ; can never actually happen; for math only
FSlow:     DW 100       ; 100 is about the lowest velocity value that will move
RSlow:     DW -100
FMid:      DW 350       ; 350 is a medium speed
RMid:      DW -350
FFast:     DW 500       ; 500 is almost max speed (511 is max)
RFast:     DW -500

MinBatt:  DW 140       ; 14.0V - minimum safe battery voltage
I2CWCmd:  DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:  DW &H0190    ; write nothing, read one byte, addr 0x90

DataArray:
	DW 0
;***************************************************************
;* IO address space map
;***************************************************************
SWITCHES: EQU &H00  ; slide switches
LEDS:     EQU &H01  ; red LEDs
TIMER:    EQU &H02  ; timer, usually running at 10 Hz
XIO:      EQU &H03  ; pushbuttons and some misc. inputs
SSEG1:    EQU &H04  ; seven-segment display (4-digits only)
SSEG2:    EQU &H05  ; seven-segment display (4-digits only)
LCD:      EQU &H06  ; primitive 4-digit LCD display
XLEDS:    EQU &H07  ; Green LEDs (and Red LED16+17)
BEEP:     EQU &H0A  ; Control the beep
CTIMER:   EQU &H0C  ; Configurable timer for interrupts
LPOS:     EQU &H80  ; left wheel encoder position (read only)
LVEL:     EQU &H82  ; current left wheel velocity (read only)
LVELCMD:  EQU &H83  ; left wheel velocity command (write only)
RPOS:     EQU &H88  ; same values for right wheel...
RVEL:     EQU &H8A  ; ...
RVELCMD:  EQU &H8B  ; ...
I2C_CMD:  EQU &H90  ; I2C module's CMD register,
I2C_DATA: EQU &H91  ; ... DATA register,
I2C_RDY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RDY: EQU &H99  ; UART status
SONAR:    EQU &HA0  ; base address for more than 16 registers....
DIST0:    EQU &HA8  ; the eight sonar distance readings
DIST1:    EQU &HA9  ; ...
DIST2:    EQU &HAA  ; ...
DIST3:    EQU &HAB  ; ...
DIST4:    EQU &HAC  ; ...
DIST5:    EQU &HAD  ; ...
DIST6:    EQU &HAE  ; ...
DIST7:    EQU &HAF  ; ...
SONALARM: EQU &HB0  ; Write alarm distance; read alarm register
SONARINT: EQU &HB1  ; Write mask for sonar interrupts
SONAREN:  EQU &HB2  ; register to control which sonars are enabled
XPOS:     EQU &HC0  ; Current X-position (read only)
YPOS:     EQU &HC1  ; Y-position
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0
RIN:      EQU &HC8
LIN:      EQU &HC9
IR_HI:    EQU &HD0  ; read the high word of the IR receiver (OUT will clear both words)
IR_LO:    EQU &HD1  ; read the low word of the IR receiver (OUT will clear both words)