; This program includes...
; - Robot initialization (checking the battery, stopping motors, etc.).
; - The movement API.
; - Several useful subroutines (ATAN2, Neg, Abs, mult, div).
; - Some useful constants (masks, numbers, robot stuff, etc.)

; This code uses the timer interrupt for the movement control code.
; The ISR jump table is located in mem 0-4.  See manual for details.
ORG 0
	JUMP   Init        ; Reset vector
	JUMP   Sonar_Int   ; Sonar interrupt (unused)
	RETI			   ; Timer interrupt
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
	; LOADI  10          ; period = (10 ms * 10) = 0.1s, or 10Hz.
	; OUT    CTIMER      ; turn on timer peripheral
	; SEI    &B0010      ; enable interrupts from source 2 (timer)
	; LOADI  0
	; STORE  STATE		 ; reset STATE to 0
	; at this point, timer interrupts will be firing at 10Hz, and
	; code in that ISR will attempt to control the robot.
	; If you want to take manual control of the robot,
	; execute CLI &B0010 to disable the timer interrupt.
	LOADI	400
	OUT		SONALARM		 ; write HalfMeter to SONALARM to set interrupt
							 ; to alarm when reflector is within half meter
	LOADI	&B00111111
	OUT		SONARINT		 ; only enable the front and side sonars to interrupt
	SEI		&B0001		 ; enable interrupts from source 1 (sonar)
	JUMP	GoStraight	 ; go straight indefinitely
	
GoStraight:				 ; Go straight with FSlow speed and current direction
	LOAD	Ten
	OUT		SSEG2
	LOAD	FMid
	STORE	DVel
	IN		THETA
	STORE	DTheta
	CALL	ControlMovement
	JUMP	GoStraight

CCNT:       DW 100
OLDVAL:		DW 0	
RCNT:       DW 10

Circling:
	IN 		TIMER
	STORE   OLDVAL
REVERSELOOP:				; Reverse function Lixing&Yida
	LOADI   -200
	OUT     LVELCMD
	OUT     RVELCMD
	IN      TIMER
	SUB     OLDVAL
	SUB     RCNT
	JNEG    REVERSELOOP
	
	IN      TIMER
	STORE   OLDVAL
	
	;LOAD  	CCNT
	;ADDI  	30
	;STORE 	CCNT
		
CIRCLELOOP:
	;IN 		DIST2
	;SUB     150
	;JNEG    FINWALL
	
	;IN      DIST3
	;SUB     150
	;JNEG    FINWALL
	
	;IN      DIST3
	;ADDI    -254
	;JNEG    FINWALL

	IN		DIST5
	SUB		200
	JNEG	DoLarge
	LOAD	Eight
	OUT		SSEG2
	LOADI   510
	OUT     LVELCMD
	LOADI	285
	OUT		RVELCMD
	JUMP	CheckTime
	
DoLarge:
	LOAD	Seven
	OUT		SSEG2
	LOADI   100
	OUT     LVELCMD
	LOADI	100
	OUT		RVELCMD
CheckTime:
	IN		TIMER
	SUB  	CCNT
	SUB     OLDVAL
	
	OUT	    SSEG1
	JNEG	CIRCLELOOP
	
	IN		DIST2
	SUB		HalfMeter
	JNEG	OutLoop2
	IN		DIST1
	SUB		HalfMeter
	JNEG	OutLoop1
	IN		DIST0
	SUB		HalfMeter
	JNEG	OutLoop0

	JUMP    CIRCLELOOP
	

OutLoop2:
; Reset absolute angle odometry to 0
	LOAD	 Zero
	OUT	 THETA
; Set the angle to turn as 90
	ADDI	 12
	STORE	 Angle
; Turn until desired angle met
	CALL	 KeepTurning
	JUMP	 End_Sonar_Int

OutLoop1:
; Reset absolute angle odometry to 0
	LOAD	 Zero
	OUT	 THETA
; Set the angle to turn as 90
	ADDI	 44
	STORE	 Angle
; Turn until desired angle met
	CALL	 KeepTurning
	JUMP	 End_Sonar_Int

OutLoop0:
; Reset absolute angle odometry to 0
	LOAD	 Zero
	OUT	 THETA
; Set the angle to turn as 90
	ADDI	 90
	STORE	 Angle
; Turn until desired angle met
	CALL	 KeepTurning
	JUMP	 End_Sonar_Int
	
FINWALL:
	CALL     TURN180
	JUMP     End_Sonar_Int


; InfLoop: 
; 	JUMP   InfLoop
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


SonarState:
	DW		&H0000
Sonar_Int:
	LOAD	Nine
	OUT		SSEG2
	LOAD	Zero
	OUT	 	SONARINT	 ; close the interrupt during stopping
	LOAD	SonarState
	JUMP	StopBot		 ; State 0 is StopBot
State1:
	JUMP  Closest		 ; State 1 is Closest
State2:
	JUMP  TurnToReflector		 ; State 2 is TurnToReflector
State3:
;	JUMP  Turn90		 ; State 3 is Turn90
State4:	
	JUMP  Circling
End_Sonar_Int:
	LOADI	 &B00111111	 
	OUT	 	 SONARINT	 ; reopen the interrupt
	RETI
	
	
MAXVALUE:	 DW 500
TEMPCNT:     DW 0
CHECKWALL:
	LOADI    0
	STORE    TEMPCNT
	IN       DIST1
	SUB      MAXVALUE
	JPOS     NEXT1
	LOAD     TEMPCNT
	ADDI     1
	STORE    TEMPCNT
NEXT1:
	IN       DIST2
	SUB      MAXVALUE
	JPOS	 NEXT2
	LOAD     TEMPCNT
	ADDI     1
	STORE    TEMPCNT
NEXT2:
	IN       DIST3
	SUB      MAXVALUE
	JPOS	 NEXT3
	LOAD     TEMPCNT
	ADDI     1
	STORE    TEMPCNT
NEXT3:
	IN       DIST4
	SUB      MAXVALUE
	JPOS	 NEXT4
	LOAD     TEMPCNT
	ADDI     1
	STORE    TEMPCNT
NEXT4:
	LOAD     TEMPCNT
	ADDI     -4
	JZERO    State3
	JNEG     State3
	
	CALL     TURN180
	JUMP     End_Sonar_Int

; 0. Stop the robot
StopBot:
	LOAD	 Zero
	STORE	 DVel			 ; set speed to 0
	CALL	 ControlMovement
	IN	 	 LVEL		 ; read in odometry data
	SUB	 Ten			 ; check whether it's above 10
	JPOS	 StopBot		 ; if it's not slow enough, keep stopping the robot
	JUMP	 State1

; 1. Pick out the closest reflector
Closest:
	IN		 DIST0
	STORE	 MinValue	 ; give out AC for next reading
	IN		 DIST1
	SUB	 MinValue	 ; DIST1 - MinValue
	JPOS	 ReadSonar2
	ADD	 MinValue
	STORE	 MinValue	 ; Update MinValue
	LOADI	 1
	STORE  MinIndex	 ; Update MinIndex
ReadSonar2:
	IN		 DIST2
	SUB	 MinValue	 ; DIST2 - MinValue
	JPOS	 ReadSonar3
	ADD	 MinValue
	STORE	 MinValue	 ; Update MinValue
	LOADI	 2
	STORE  MinIndex	 ; Update MinIndex
ReadSonar3:
	IN		 DIST3
	SUB	 MinValue	 ; DIST3 - MinValue
	JPOS	 ReadSonar4
	ADD	 MinValue
	STORE	 MinValue	 ; Update MinValue
	LOADI	 3
	STORE  MinIndex	 ; Update MinIndex
ReadSonar4:
	IN		 DIST4
	SUB	 MinValue	 ; DIST4 - MinValue
	JPOS	 ReadSonar5
	ADD	 MinValue
	STORE	 MinValue	 ; Update MinValue
	LOADI	 4
	STORE  MinIndex	 ; Update MinIndex
ReadSonar5:
	IN		 DIST5
	SUB	 MinValue	 ; DIST5 - MinValue
	JUMP	 State2
	ADD	 MinValue
	STORE	 MinValue	 ; Update MinValue
	LOADI	 5
	STORE  MinIndex	 ; Update MinIndex
	JUMP	 State2
MinValue:
	DW		 &H0000
MinIndex:
	DW		 &H0000

; 2. Turn to the closest reflector
TurnToReflector:
	LOAD	 MinIndex	 ; load the index of the sonar with closest reflector
	OUT    SSEG1
	JZERO	 TurnTo0
	ADDI	 -1
	JZERO  TurnTo1
	ADDI	 -1
	JZERO  TurnTo2
	ADDI	 -1
	JZERO  TurnTo3
	ADDI	 -1
	JZERO  TurnTo4
	ADDI	 -1
	JZERO  TurnTo5
; If out of bound, JUMP back
	JUMP	 State3

SonarData:
	DW		 &H0000

TurnTo2:
; Turn to the angle where head is pointing to the reflector
	LOAD	 Zero
	ADDI	 80
	CALL   	 Mod360
	STORE	 Angle		; prepare parameter for turning
	JUMP	 Turn

TurnTo3:
	LOAD	 Zero
	ADDI	 75
	STORE	 Angle
	JUMP	 Turn

TurnTo1:
	LOAD	 Zero
	ADDI	 110
	STORE	 Angle
	JUMP	 Turn

TurnTo4:
	LOAD	 Zero
	ADDI	 46
	STORE	 Angle
	JUMP	 Turn

TurnTo0:
	LOAD	 Zero
	ADDI	 179
	STORE	 Angle
	JUMP	 Turn

TurnTo5:
	LOAD	 Zero
	ADDI	 0
	STORE	 Angle
	JUMP	 Turn

Turn:
; Reset absolute angle odometry to 0
	LOAD	 Zero
	OUT	 	 THETA
; Turn until desired angle met
	CALL	 KeepTurning
; End this interrupt
	LOAD	 Three
	STORE	 SonarState	 ; set next state to be state 3
	JUMP	 State3

KeepTurning:
	LOAD	 MinIndex
	OUT		 SSEG2
	LOAD	 Angle				; load parameter into AC
	STORE	 DTheta				; put desired angle to DTheta
	LOAD	 FSlow
	STORE	 DVel				; set desired speed to FSlow
	CALL	 ControlMovement	; call API
	IN		 THETA				; read odometry
	STORE	 Temp_THETA
	LOAD	 Angle				; subtract parameter Angle
	SUB		 Temp_THETA
	CALL	 Abs
	OUT		 SSEG1
	;ADDI	 -3					; if the difference is bigger than 3 degrees
	JPOS	 KeepTurning		; keep turning
	RETURN						; otherwise, return
; Local variable for KeepTurning
Angle:
	DW		 &H0000
Temp_THETA:
	DW		 &H0000
	
Turn90:
; Reset absolute angle odometry to 0
	LOAD	 Zero
	OUT	 THETA
; Set the angle to turn as 90
	ADDI	 65
	STORE	 Angle
; Turn until desired angle met
	CALL	 KeepTurning
; Set next state to be state 4
	JUMP	 State4
	
	
TURN180:
; Reset absolute angle odometry to 0
	LOAD	 Zero
	OUT	 THETA
; Set the angle to turn as 90
	ADDI	 135
	STORE	 Angle
; Turn until desired angle met
	CALL	 KeepTurning
; Set next state to be state 4
	RETURN
	
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	

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
	LOAD	counter			; read counter
	ADDI	1				; increment counter
	STORE	counter
	ADDI 	-30				; check if we've hit 30 (3 seconds)
	JNEG	SkipThis		; if not, keep moving
	ADDI	-10				; check if we've hit 10 (1 second) 	
	JNEG	SetVel0			; if not, don't reset our counter
	AND		0				
	STORE	counter			; reset counter if so
; set movement velocity to 0 (stop)
SetVel0:
	AND		0				; get zero in case AC isn't zero before
	STORE 	DVel
	JUMP GoDoMvmt			; let the MoveAPI do all our heavy lifting
SkipThis:					; move forward slowly
	LOAD	FSlow
	STORE	DVel
	JUMP	GoDoMvmt		; let the MoveAPI do all our heavy lifting
	;***********************************************************
	;* Local vars for this state
	;***********************************************************
	counter:	DW &H0000
HandleTest2State:
	; check for an obstacle within four feet (straight ahead)
	; if so, stop for one second
	;     then turn until the correct direction
	;     if so, stop for one second
	;     then proceed until within 1 foot
	; otherwise keep moving forward
	; checks sensors 0-5 in order (not the back two for now)
	;; eventually we should change this to a helper function that returns
	;; which sensor detects something closest
	;; excluding certain sensors
	LOAD	DIST0
	SUB		Ft4
	JNEG	Set90
	JUMP	Check1
Set90:
	LOADI	90
	JUMP	SetTargetAngle
Check1:
	LOAD	DIST1
	SUB		Ft4
	LOADI	44
	JUMP	SetTargetAngle
	LOAD	DIST2
	SUB		Ft4
	LOADI	12
	JUMP	SetTargetAngle
	LOAD	DIST3
	SUB		Ft4
	LOADI	-12
	JUMP	SetTargetAngle
	LOAD	DIST4
	SUB		Ft4
	LOADI	-44
	JUMP	SetTargetAngle
	LOAD	DIST5
	SUB		Ft4
	LOADI	-90

SetTargetAngle:
	; assumes that the target change in angle is currently in AC
	ADD		THETA
	STORE	currTarg

SetTargetHeading:
	; assumes that the target value is stored in currTarg
	LOAD	currTarg	
	STORE	DTheta
	JUMP	GoDoMvmt
	;***********************************************************
	;* Local vars for this state
	;***********************************************************
	currTarg:	DW &H0000
	counter1:	DW &H0000
	counter2:	DW &H0000
HandleTest3State:
	; circle with 1ft radius
	JUMP GoDoMvmt
	;***********************************************************
	;* Local vars for this state
	;***********************************************************

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
STATE:		DW &H0000	; STATE variable -- track the main state

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
NegOne:   DW -1
Zero:     DW 0
One:      DW 1
Two:      DW 2
Three:    DW 3
Four:     DW 4
Five:     DW 5
Six:      DW 6
Seven:    DW 7
Eight:    DW 8
Nine:     DW 9
Ten:      DW 10

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
HalfMeter: DW 481      ; ~0.5m in 1.04mm units
Ft1:	   DW 293	   ; ~1ft
Ft2:       DW 586       ; ~2ft in 1.04mm units
Ft3:       DW 879
Ft4:       DW 1172
Deg90:     DW 90        ; 90 degrees in odometer units
Deg180:    DW 180       ; 180
Deg270:    DW 270       ; 270
Deg360:    DW 360       ; can never actually happen; for math only
FSlow:     DW 100       ; 100 is about the lowest velocity value that will move
RSlow:     DW -100
FMid:      DW 275       ; 350 is a medium speed
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
IR_LO:    EQU &HD1  ; read the low word of the IR receiver (OUT will clear both words)1